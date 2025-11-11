#include "plan_env/sdf_node.hpp"
#include <cmath>
#include <chrono>

SdfNode::SdfNode() : Node("sdf_node"), map_initialized_(false) {
    RCLCPP_INFO(this->get_logger(), "Creating SdfNode");
    
    // Declare parameters - matching map1 config
    this->declare_parameter<double>("resolution", 0.1);  // ESDF resolution: match grid resolution for full detail
    this->declare_parameter<double>("map_size_x", 3.2);
    this->declare_parameter<double>("map_size_y", 6.0);
    this->declare_parameter<double>("map_size_z", 0.01);  // Back to 2D (single layer) for faster processing
    this->declare_parameter<double>("obstacles_inflation", 0.1);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<int>("local_map_margin", 5);
    this->declare_parameter<double>("visualization_truncate_height", 5.0);
    this->declare_parameter<double>("virtual_ceil_height", -1.0);
    this->declare_parameter<double>("ground_height", 0.0);
    this->declare_parameter<double>("esdf_slice_height", 1.0);
    
    // Local update parameters
    this->declare_parameter<double>("local_update_range_x", 5.5);  // Local update range in x
    this->declare_parameter<double>("local_update_range_y", 5.5);  // Local update range in y
    this->declare_parameter<double>("local_update_range_z", 0.01);   // Local update range in z
    
    // Get parameters
    mp_.resolution_ = this->get_parameter("resolution").as_double();
    double x_size = this->get_parameter("map_size_x").as_double();
    double y_size = this->get_parameter("map_size_y").as_double();
    double z_size = this->get_parameter("map_size_z").as_double();
    mp_.obstacles_inflation_ = this->get_parameter("obstacles_inflation").as_double();
    mp_.frame_id_ = this->get_parameter("frame_id").as_string();
    mp_.local_map_margin_ = this->get_parameter("local_map_margin").as_int();
    mp_.visualization_truncate_height_ = this->get_parameter("visualization_truncate_height").as_double();
    mp_.virtual_ceil_height_ = this->get_parameter("virtual_ceil_height").as_double();
    mp_.ground_height_ = this->get_parameter("ground_height").as_double();
    mp_.esdf_slice_height_ = this->get_parameter("esdf_slice_height").as_double();
    
    // Local update parameters
    double local_range_x = this->get_parameter("local_update_range_x").as_double();
    double local_range_y = this->get_parameter("local_update_range_y").as_double();
    double local_range_z = this->get_parameter("local_update_range_z").as_double();
    
    mp_.resolution_inv_ = 1.0 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(0.0, 0.0, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    
    for (int i = 0; i < 3; ++i) {
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    }
    
    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
    
    mp_.map_min_idx_ = Eigen::Vector3i::Zero();
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    
    // Initialize data buffers
    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    
    md_.occupancy_buffer_ = std::vector<double>(buffer_size, 0.0);
    md_.occupancy_buffer_neg = std::vector<char>(buffer_size, 0);
    md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);
    
    md_.distance_buffer_ = std::vector<double>(buffer_size, 10000.0);
    md_.distance_buffer_neg_ = std::vector<double>(buffer_size, 10000.0);
    md_.distance_buffer_all_ = std::vector<double>(buffer_size, 10000.0);
    
    md_.tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);
    md_.tmp_buffer2_ = std::vector<double>(buffer_size, 0.0);
    
    // Initialize local bounds (start with full map)
    md_.local_bound_min_ = Eigen::Vector3i::Zero();
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    
    md_.esdf_need_update_ = false;
    md_.local_updated_ = false;
    md_.occ_need_update_ = false;
    md_.camera_pos_ = Eigen::Vector3d(0.0, 0.0, mp_.ground_height_);  // Initialize at origin
    md_.latest_grid_ = nullptr;
    
    // Initialize local update range - will be used when processing grid
    mp_.local_update_range_ = Eigen::Vector3d(local_range_x, local_range_y, local_range_z);
    
    RCLCPP_INFO(this->get_logger(), "Local update enabled: true, Range: %.2fx%.2fx%.2f", 
        local_range_x, local_range_y, local_range_z);
    
    // Subscribers and Publishers
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map1", 10,
        std::bind(&SdfNode::occupancyGridCallback, this, std::placeholders::_1));
    
    // Odom subscriber for camera position
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_world", 10,
        std::bind(&SdfNode::odomCallback, this, std::placeholders::_1));
    
    esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
    map_inflate_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/inflate", 10);
    
    // Timers
    esdf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz update
        std::bind(&SdfNode::updateEsdfTimer, this));
    
    occ_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz update
        std::bind(&SdfNode::updateOccupancyCallback, this));
    
    vis_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
            this->publishESDF();
            this->publishMapInflate(false);
        });
    
    RCLCPP_INFO(this->get_logger(), "SdfNode initialized");
}

void SdfNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!map_initialized_) {
        map_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Received first occupancy grid");
    }
    
    // Save the occupancy grid to buffer
    md_.latest_grid_ = msg;
    if (md_.has_odom_) {
    md_.occ_need_update_ = true;
    }
}

void SdfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract camera position from odometry
    md_.camera_pos_(0) = msg->pose.pose.position.x;
    md_.camera_pos_(1) = msg->pose.pose.position.y;
    md_.camera_pos_(2) = msg->pose.pose.position.z;
    md_.has_odom_ = true;
}

template <typename F_get_val, typename F_set_val>
void SdfNode::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
    // 使用 std::vector 替代 VLA 以避免编译器警告
    std::vector<int> v(mp_.map_voxel_num_(dim));
    std::vector<double> z(mp_.map_voxel_num_(dim) + 1);
    
    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();
    
    for (int q = start + 1; q <= end; q++) {
        k++;
        double s;
        
        do {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);
        
        k++;
        
        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }
    
    k = start;
    
    for (int q = start; q <= end; q++) {
        while (z[k + 1] < q) k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

void SdfNode::updateESDF3d() {
    Eigen::Vector3i min_esdf = md_.local_bound_min_;
    Eigen::Vector3i max_esdf = md_.local_bound_max_;
    
   
    /* ========== compute positive DT ========== */
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                        0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
                min_esdf[2], max_esdf[2], 2);
        }
    }
    
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                min_esdf[1], max_esdf[1], 1);
        }
    }
    
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }
    
    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                if (md_.occupancy_buffer_inflate_[idx] == 0) {
                    md_.occupancy_buffer_neg[idx] = 1;
                } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
                    md_.occupancy_buffer_neg[idx] = 0;
                }
            }
    
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return md_.occupancy_buffer_neg[toAddress(x, y, z)] == 1 ?
                        0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
                min_esdf[2], max_esdf[2], 2);
        }
    }
    
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                min_esdf[1], max_esdf[1], 1);
        }
    }
    
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }
    
    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];
                
                if (md_.distance_buffer_neg_[idx] > 0.0)
                    md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
            }
}

void SdfNode::updateEsdfTimer() {
    if (!md_.esdf_need_update_) {
        static bool first_log = true;
        if (first_log) {
            RCLCPP_INFO(this->get_logger(), "updateEsdfTimer: esdf_need_update_ is false");
            first_log = false;
        }
        return;
    }   

    
    
    auto start_time = std::chrono::high_resolution_clock::now();
    updateESDF3d();
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "ESDF update took %ld ms", duration.count());
    
    md_.esdf_need_update_ = false;
    md_.local_updated_ = false;
    
}


void SdfNode::resetBuffer() {
    resetBuffer(mp_.map_min_boundary_, mp_.map_max_boundary_);
}

void SdfNode::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);
    
    boundIndex(min_id);
    boundIndex(max_id);
    
    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z) {
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                md_.distance_buffer_[toAddress(x, y, z)] = 10000;
            }
}


void SdfNode::publishESDF() {
    if (!map_initialized_) return;
    
    double dist;
    std::vector<float> xs, ys, zs, intensities;
    
    const double min_dist = 0.0;
    const double max_dist = 3.0;
    
    // Use the update bounds
    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;
    boundIndex(min_cut);
    boundIndex(max_cut);
    
    // Publish ESDF at ground level (z=0)
    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y) {
            
            // Calculate world position from grid indices
            Eigen::Vector3d pos;
            pos(0) = mp_.map_origin_(0) + (x + 0.5) * mp_.resolution_;
            pos(1) = mp_.map_origin_(1) + (y + 0.5) * mp_.resolution_;
            // Query ESDF at ground level (same as obstacles)
            pos(2) = mp_.map_origin_(2);  // ground_height_ = 0.0
            
            dist = getDistance(pos);
            dist = std::min(dist, max_dist);
            dist = std::max(dist, min_dist);
            
            xs.push_back(pos(0));
            ys.push_back(pos(1));
            zs.push_back(mp_.map_origin_(2));  // Display at ground level (z=0)
            intensities.push_back((dist - min_dist) / (max_dist - min_dist));
            
           
        }
    
    // Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = mp_.frame_id_;
    cloud_msg.height = 1;
    cloud_msg.width = xs.size();
    
    // Define fields manually
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.clear();
    modifier.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    
    // Resize container
    modifier.resize(xs.size());
    
    // Fill data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    
    for (size_t i = 0; i < xs.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        *iter_x = xs[i];
        *iter_y = ys[i];
        *iter_z = zs[i];
        *iter_intensity = intensities[i];
    }
    
    esdf_pub_->publish(cloud_msg);
}

void SdfNode::publishMapInflate(bool all_info) {
    if (!map_initialized_) return;
    
    std::vector<float> xs, ys, zs;
    
    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;
    
    if (all_info) {
        int lmm = mp_.local_map_margin_;
        min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
        max_cut += Eigen::Vector3i(lmm, lmm, lmm);
    }
    
    boundIndex(min_cut);
    boundIndex(max_cut);
    
    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z) {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;
                
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height_) continue;
                
                xs.push_back(pos(0));
                ys.push_back(pos(1));
                zs.push_back(pos(2));
            }
    
    // Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = mp_.frame_id_;
    cloud_msg.height = 1;
    cloud_msg.width = xs.size();
    
    // Define fields manually
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.clear();
    modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    
    // Resize container
    modifier.resize(xs.size());
    
    // Fill data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
    for (size_t i = 0; i < xs.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = xs[i];
        *iter_y = ys[i];
        *iter_z = zs[i];
    }
    
    map_inflate_pub_->publish(cloud_msg);
}

void SdfNode::updateOccupancyCallback() {
    if (!md_.occ_need_update_) return;
    
    if (md_.latest_grid_ == nullptr) {
        md_.occ_need_update_ = false;
        return;
    }
    
    // Update local bounds based on camera position and local update range
    Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
    Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

        
        // Clamp to map boundaries
        for (int i = 0; i < 3; ++i) {
            local_range_min(i) = std::max(local_range_min(i), mp_.map_min_boundary_(i));
            local_range_max(i) = std::min(local_range_max(i), mp_.map_max_boundary_(i));
        }
        
        // Additional clamping to grid boundaries for x,y coordinates
        const double grid_origin_x = md_.latest_grid_->info.origin.position.x;
        const double grid_origin_y = md_.latest_grid_->info.origin.position.y;
        const double grid_res = md_.latest_grid_->info.resolution;
        const double grid_max_x = grid_origin_x + md_.latest_grid_->info.width * grid_res;
        const double grid_max_y = grid_origin_y + md_.latest_grid_->info.height * grid_res;
        

        
        local_range_min(0) = std::max(local_range_min(0), grid_origin_x);
        local_range_min(1) = std::max(local_range_min(1), grid_origin_y);
        local_range_max(0) = std::min(local_range_max(0), grid_max_x);
        local_range_max(1) = std::min(local_range_max(1), grid_max_y);
        

        
        // Convert to indices
        Eigen::Vector3i min_id, max_id;
        posToIndex(local_range_min, min_id);
        posToIndex(local_range_max, max_id);
        
        
        
        // Update local bounds
        md_.local_bound_min_ = min_id;
        md_.local_bound_max_ = max_id;
        boundIndex(md_.local_bound_min_);
        boundIndex(md_.local_bound_max_);
        
        
        
        md_.local_updated_ = true;
    
    // Clear and inflate local map
    if (md_.local_updated_) {
        clearAndInflateLocalMap();
    }
    
    md_.occ_need_update_ = false;
    md_.esdf_need_update_ = true;
}

void SdfNode::clearAndInflateLocalMap() {
    if (!md_.latest_grid_) return;
    
    /*clear outside local*/
    const int vec_margin = 5;
    
    Eigen::Vector3i min_cut = md_.local_bound_min_ -
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    boundIndex(min_cut_m);
    boundIndex(max_cut_m);

    // clear data outside the local range
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

            for (int z = min_cut_m(2); z < min_cut(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

            for (int y = min_cut_m(1); y < min_cut(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

            for (int x = min_cut_m(0); x < min_cut(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

    // Extract grid parameters for cleaner code
    const int width = md_.latest_grid_->info.width;
    // const int height = md_.latest_grid_->info.height;
    const double grid_res = md_.latest_grid_->info.resolution;
    const double origin_x = md_.latest_grid_->info.origin.position.x;
    const double origin_y = md_.latest_grid_->info.origin.position.y;
    const std::vector<int8_t>& grid_data = md_.latest_grid_->data;
    
    // inflate occupied voxels to compensate robot size
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    std::vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
    Eigen::Vector3i inf_pt;

    // clear outdated data
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
            for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
            }

    // inflate obstacles - modified condition: check if latest_grid.data > 50
    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
            for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

                // Convert 3D index to 2D grid index
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                
                // Convert world position to grid coordinates
                int grid_x = static_cast<int>((pos(0) - origin_x) / grid_res);
                int grid_y = static_cast<int>((pos(1) - origin_y) / grid_res);
                
                // No need to check bounds since local_bound_min/max already clamped to grid boundaries
                int grid_idx = grid_y * width + grid_x;
                
                // Modified condition: check if grid data > 50 instead of buffer > log
                if (grid_data[grid_idx] > 50) {
                    // RCLCPP_INFO(this->get_logger(), "Inflate obstacle at (%d, %d, %d), grid_idx=%d", x, y, z, grid_idx);
                    inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
                    for (int k = 0; k < static_cast<int>(inf_pts.size()); ++k) {
                        inf_pt = inf_pts[k];
                        int idx_inf = toAddress(inf_pt);
                        if (idx_inf < 0 ||
                            idx_inf >= buffer_size) {
                            continue;
                        }
                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
        }
    }
}

// Additional interface methods for EDTEnvironment compatibility
void SdfNode::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff) {
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    
    Eigen::Vector3d pos_tmp;
    indexToPos(idx, pos_tmp);
    
    diff = (pos - pos_tmp) * mp_.resolution_inv_;
    
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                Eigen::Vector3i tmp_idx = idx + Eigen::Vector3i(x, y, z);
                boundIndex(tmp_idx);
                indexToPos(tmp_idx, pts[x][y][z]);
            }
        }
    }
}

void SdfNode::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

double SdfNode::getResolution() {
    return mp_.resolution_;
}

Eigen::Vector3d SdfNode::getOrigin() {
    return mp_.map_origin_;
}


