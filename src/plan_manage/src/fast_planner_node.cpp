#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <plan_manage/backward.hpp>
#include <plan_manage/kino_replan_fsm.h>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fast_planner_node");

  fast_planner::KinoReplanFSM kino_replan;
  kino_replan.init(node);

  // Get the SdfNode from planner_manager and add it to executor
  // SdfNode is a separate node that needs to be spun
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  
  // Get SdfNode from planner_manager
  auto sdf_node = kino_replan.getSdfNode();
  if (sdf_node) {
    executor->add_node(sdf_node);
  }

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
