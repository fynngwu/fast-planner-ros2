#include "plan_env/sdf_node.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>

SdfNode::SdfNode() : Node("sdf_node"), map_initialized_(false) {
    RCLCPP_INFO(this->get_logger(), "Creating SdfNode");
    
    // Declare parameters - matching map1 config
    this->declare_parameter<double>("resolution", 0.05);  // ESDF resolution: match grid resolution for full detail
    this->declare_parameter<double>("map_size_x", 12.0);
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
    
    md_.occupancy_buffer_neg = std::vector<char>(buffer_size, 0);
    md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);
    
    md_.distance_buffer_ = std::vector<double>(buffer_size, 10000.0);
    md_.distance_buffer_neg_ = std::vector<double>(buffer_size, 10000.0);
    md_.distance_buffer_all_ = std::vector<double>(buffer_size, 10000.0);
    
    md_.tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);
    md_.tmp_buffer2_ = std::vector<double>(buffer_size, 0.0);
    
    // Initialize local bounds (start with full map)
    md_.local_bound_min_ = mp_.map_min_idx_;
    md_.local_bound_max_ = mp_.map_max_idx_;
    md_.map1_grid_ = nullptr;
    md_.map3_grid_ = nullptr;
    md_.camera_pos_ = Eigen::Vector3d::Zero();
    md_.has_odom_ = false;
    md_.esdf_need_update_ = false;
    
    // Initialize local update range - will be used when processing grid
    mp_.local_update_range_ = Eigen::Vector3d(local_range_x, local_range_y, local_range_z);
    
    RCLCPP_INFO(this->get_logger(), "Static map mode enabled, map voxels: [%d, %d, %d]",
        mp_.map_voxel_num_(0), mp_.map_voxel_num_(1), mp_.map_voxel_num_(2));
    
    // Subscribers and Publishers
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    
    occupancy_grid_sub_map1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map1", map_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { this->occupancyGridCallback(msg, 1); });
    
    occupancy_grid_sub_map3_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map3", map_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { this->occupancyGridCallback(msg, 3); });
    
    // Odom subscriber retained for compatibility
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_world", 10,
        std::bind(&SdfNode::odomCallback, this, std::placeholders::_1));
    
    esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
    map_inflate_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/inflate", 10);
    
    vis_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            this->publishESDF();
            this->publishMapInflate(false);
        });
    esdf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz update
        std::bind(&SdfNode::updateEsdfTimer, this));
    
    RCLCPP_INFO(this->get_logger(), "SdfNode initialized");
}

void SdfNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, int map_index) {
    if (map_index == 1) {
        md_.map1_grid_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received /map1 occupancy grid");
    } else if (map_index == 3) {
        md_.map3_grid_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received /map3 occupancy grid");
    } else {
        RCLCPP_WARN(this->get_logger(), "Received occupancy grid with unknown index: %d", map_index);
        return;
    }
    
    rebuildStaticMap();
    md_.esdf_need_update_ = true;
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

}

void SdfNode::rebuildStaticMap() {
    if (!md_.map1_grid_ && !md_.map3_grid_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for occupancy grids before rebuilding static map.");
        return;
    }
    
    md_.local_bound_min_ = mp_.map_min_idx_;
    md_.local_bound_max_ = mp_.map_max_idx_;
    
    map_initialized_ = false;
    
    if (md_.map1_grid_) {
        inflateFromGrid(*md_.map1_grid_);
    }
    if (md_.map3_grid_) {
        inflateFromGrid(*md_.map3_grid_);
        RCLCPP_INFO(this->get_logger(), "Inflated map3");
    }
    
    md_.esdf_need_update_ = true;
    map_initialized_ = true;
}

void SdfNode::inflateFromGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    if (width <= 0 || height <= 0) {
        RCLCPP_WARN(this->get_logger(), "Received occupancy grid with invalid dimensions.");
        return;
    }
    
    const double grid_res = grid.info.resolution;
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const auto& data = grid.data;
    
    const int inf_step = std::max(0, static_cast<int>(std::ceil(mp_.obstacles_inflation_ / mp_.resolution_)));
    const int footprint_size = (2 * inf_step + 1) * (2 * inf_step + 1);
    std::vector<Eigen::Vector3i> inf_pts(footprint_size);
    
    const int buffer_size = static_cast<int>(md_.occupancy_buffer_inflate_.size());
    
    Eigen::Vector3d range_min(origin_x, origin_y, 0.0);
    Eigen::Vector3d range_max(
        origin_x + width * grid_res,
        origin_y + height * grid_res,
        0.0);
    
    Eigen::Vector3i min_idx, max_idx;
    posToIndex(range_min, min_idx);
    posToIndex(range_max, max_idx);
    boundIndex(min_idx);
    boundIndex(max_idx);
    
    RCLCPP_INFO(this->get_logger(), "Inflating from grid: min_idx = [%d, %d, %d], max_idx = [%d, %d, %d]",
        min_idx.x(), min_idx.y(), min_idx.z(), max_idx.x(), max_idx.y(), max_idx.z());
    
    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
        for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, 0), pos);
            
            int grid_x = static_cast<int>(std::floor((pos.x() - origin_x) / grid_res));
            int grid_y = static_cast<int>(std::floor((pos.y() - origin_y) / grid_res));
            
            // 添加边界检查，确保 grid_x 和 grid_y 在有效范围内
            if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
                continue;
            }
            
            int grid_idx = grid_y * width + grid_x;
            
            if (data[grid_idx] > 50) {
                Eigen::Vector3i center(x, y, 0);
                inflatePoint(center, inf_step, inf_pts);
                for (const auto& inf_pt : inf_pts) {
                    int idx_inf = toAddress(inf_pt);
                    if (idx_inf < 0 || idx_inf >= buffer_size) {
                        continue;
                    }
                    md_.occupancy_buffer_inflate_[idx_inf] = 1;
                }
            }
        }
    }
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
    Eigen::Vector3i min_cut = mp_.map_min_idx_;
    Eigen::Vector3i max_cut = mp_.map_max_idx_;
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

void SdfNode::publishMapInflate(bool /*all_info*/) {
    if (!map_initialized_) return;
    
    std::vector<float> xs, ys, zs;
    
    Eigen::Vector3i min_cut = mp_.map_min_idx_;
    Eigen::Vector3i max_cut = mp_.map_max_idx_;
    
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


