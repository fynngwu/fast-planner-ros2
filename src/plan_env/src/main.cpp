#include <rclcpp/rclcpp.hpp>
#include "plan_env/sdf_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SdfNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
