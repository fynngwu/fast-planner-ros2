#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher() : Node("odom_publisher") {
        // Publisher for odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_world", 10);
        
        // Subscriber to get map info and calculate center
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map1", 10,
            std::bind(&OdomPublisher::mapCallback, this, std::placeholders::_1));
        
        // Timer to publish odom at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&OdomPublisher::publishOdom, this));
        
        // Initialize circular motion parameters
        radius_ = 2.0;  // 2 meter radius
        angular_velocity_ = 0.2;  // rad/s (adjust for desired speed)
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Odom publisher started, waiting for map to determine center position");
    }
    
private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_initialized_) {
            map_initialized_ = true;
            
            // Calculate map center position
            // Map origin is at (0, 0, 0) with resolution 0.05m
            // Map size is 64x120 cells
            center_x_ = (msg->info.width * msg->info.resolution) / 2.0;
            center_y_ = (msg->info.height * msg->info.resolution) / 2.0;
            center_z_ = 0.0;
            
            RCLCPP_INFO(this->get_logger(), 
                "Map center calculated: x=%.3f, y=%.3f, z=%.3f (Map size: %dx%d, resolution: %.3f)",
                center_x_, center_y_, center_z_, 
                msg->info.width, msg->info.height, msg->info.resolution);
        }
    }
    
    void publishOdom() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        
        // Calculate circular motion position
        double current_time = (this->now() - start_time_).seconds();
        double angle = angular_velocity_ * current_time;
        
        // Calculate position on circle
        double x = center_x_ + radius_ * cos(angle);
        double y = center_y_ + radius_ * sin(angle);
        double z = center_z_;
        
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;
        
        // Calculate orientation (tangent to circle)
        double yaw = angle + M_PI/2;  // Tangent direction
        msg.pose.pose.orientation.w = cos(yaw/2);
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = sin(yaw/2);
        
        // Calculate velocity (tangent velocity)
        double linear_velocity = radius_ * angular_velocity_;
        msg.twist.twist.linear.x = -linear_velocity * sin(angle);
        msg.twist.twist.linear.y = linear_velocity * cos(angle);
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = angular_velocity_;
        
        // Set covariance (small values for known position)
        msg.pose.covariance[0] = 0.01;   // x
        msg.pose.covariance[7] = 0.01;   // y
        msg.pose.covariance[14] = 0.01;  // z
        msg.pose.covariance[21] = 0.01;  // roll
        msg.pose.covariance[28] = 0.01;  // pitch
        msg.pose.covariance[35] = 0.01;  // yaw
        
        // Velocity covariance
        msg.twist.covariance[0] = 0.01;   // linear x
        msg.twist.covariance[7] = 0.01;   // linear y
        msg.twist.covariance[14] = 0.01;  // linear z
        msg.twist.covariance[21] = 0.01;  // angular x
        msg.twist.covariance[28] = 0.01;  // angular y
        msg.twist.covariance[35] = 0.01;  // angular z
        
        odom_pub_->publish(msg);
        
        // Log position every 2 seconds for debugging
        static auto last_log_time = this->now();
        if ((this->now() - last_log_time).seconds() >= 2.0) {
            RCLCPP_INFO(this->get_logger(), 
                "Circular motion: pos=(%.3f, %.3f), angle=%.3f rad, vel=(%.3f, %.3f)",
                x, y, angle, msg.twist.twist.linear.x, msg.twist.twist.linear.y);
            last_log_time = this->now();
        }
    }
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool map_initialized_ = false;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double center_z_ = 0.0;
    
    // Circular motion parameters
    double radius_;
    double angular_velocity_;
    rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

