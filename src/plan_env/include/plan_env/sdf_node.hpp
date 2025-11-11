#ifndef SDF_NODE_HPP
#define SDF_NODE_HPP

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>
#include <vector>
#include <queue>

// Mapping parameters
struct MappingParameters {
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;
    Eigen::Vector3i map_voxel_num_;
    Eigen::Vector3i map_min_idx_, map_max_idx_;
    Eigen::Vector3d local_update_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    std::string frame_id_;
    int local_map_margin_;
    double virtual_ceil_height_, ground_height_;
    double visualization_truncate_height_;
    double esdf_slice_height_;  // Height for ESDF visualization
    double clamp_min_log_;  // Minimum log odds value
    double unknown_flag_;   // Unknown cell flag
};

// Mapping data
struct MappingData {
    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_neg;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;
    
    Eigen::Vector3i local_bound_min_, local_bound_max_;
    Eigen::Vector3d camera_pos_;  // Camera/robot position for local update
    
    bool esdf_need_update_;
    bool local_updated_;  // Flag to indicate local update
    bool occ_need_update_;  // Flag to indicate occupancy needs update
    bool has_odom_;  // Flag to indicate odometry received
    
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;  // Latest occupancy grid
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SdfNode : public rclcpp::Node {
public:
    SdfNode();
    ~SdfNode() = default;
    
    // Public interface methods for EDTEnvironment compatibility
    void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    double getResolution();
    Eigen::Vector3d getOrigin();
    void publishESDF();
    void publishMapInflate(bool all_info = true);
    
    // Inline methods (matching fast-planner implementation)
    inline int getInflateOccupancy(const Eigen::Vector3d& pos) {
        Eigen::Vector3i idx;
        posToIndex(pos, idx);
        if (!isInMap(idx)) {
            return 0;  // Unknown
        }
        return md_.occupancy_buffer_inflate_[toAddress(idx)];
    }
    
    inline double getDistance(const Eigen::Vector3d& pos) {
        Eigen::Vector3i idx;
        posToIndex(pos, idx);
        if (!isInMap(idx)) {
            return 10000.0;
        }
        return md_.distance_buffer_all_[toAddress(idx)];
    }

private:
    // Callbacks
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateEsdfTimer();
    void updateOccupancyCallback();
    
    // Map management
    void initMap();
    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int x, int y, int z);
    inline bool isInMap(const Eigen::Vector3d& pos);
    inline bool isInMap(const Eigen::Vector3i& idx);
    inline void boundIndex(Eigen::Vector3i& id);
    
    // Inflation functions
    void clearAndInflateLocalMap();
    inline void inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts);
    
    // ESDF functions
    void updateESDF3d();
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
    
    // Utilities
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
    
    MappingParameters mp_;
    MappingData md_;
    
    // ROS2 subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inflate_pub_;
    rclcpp::TimerBase::SharedPtr esdf_timer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::TimerBase::SharedPtr occ_timer_;
    
    bool map_initialized_;
};

// Inline functions
inline int SdfNode::toAddress(const Eigen::Vector3i& id) {
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + 
           id(1) * mp_.map_voxel_num_(2) + 
           id(2);
}

inline int SdfNode::toAddress(int x, int y, int z) {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + 
           y * mp_.map_voxel_num_(2) + 
           z;
}

inline void SdfNode::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline bool SdfNode::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || 
        pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || 
        pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
        return false;
    }
    return true;
}

inline bool SdfNode::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
        return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || 
        idx(1) > mp_.map_voxel_num_(1) - 1 ||
        idx(2) > mp_.map_voxel_num_(2) - 1) {
        return false;
    }
    return true;
}

inline void SdfNode::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) {
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    }
}

inline void SdfNode::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) {
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
    }
}

// plane inflation
inline void SdfNode::inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts) {
    int num = 0;
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y) {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2));
            }
}

#endif  // SDF_NODE_HPP

