/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_node.hpp>
#include <plan_manage/msg/bspline.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

namespace fast_planner {

class KinoReplanFSM {
private:
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  FastPlannerManager::Ptr      planner_manager_;
  PlanningVisualization::Ptr   visualization_;
  rclcpp::Node::SharedPtr      node_;

  double no_replan_thresh_{};
  double replan_thresh_{};
  double end_yaw_{0.0};  // 添加目标yaw角

  bool           trigger_{false};
  bool           have_target_{false};
  bool           have_odom_{false};
  FSM_EXEC_STATE exec_state_{INIT};

  Eigen::Vector3d     odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d     odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond  odom_orient_;
  Eigen::Vector3d     start_pt_, start_vel_, start_acc_, start_yaw_;
  Eigen::Vector3d     end_pt_, end_vel_;

  rclcpp::TimerBase::SharedPtr exec_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr        replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr        new_pub_;
  rclcpp::Publisher<plan_manage::msg::Bspline>::SharedPtr   bspline_pub_;

  bool callKinodynamicReplan();
  void changeFSMExecState(FSM_EXEC_STATE new_state, const std::string& pos_call);
  void printFSMExecState();

  void execFSMCallback();
  void checkCollisionCallback();
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
  KinoReplanFSM() = default;
  ~KinoReplanFSM() = default;

  void init(const rclcpp::Node::SharedPtr& node);
  
  // Get SdfNode for executor
  std::shared_ptr<SdfNode> getSdfNode() {
    if (planner_manager_ && planner_manager_->sdf_map_) {
      return planner_manager_->sdf_map_;
    }
    return nullptr;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
