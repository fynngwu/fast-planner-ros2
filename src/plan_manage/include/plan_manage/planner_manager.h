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

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <memory>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_node.hpp>
#include <plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <limits>

namespace fast_planner {

class FastPlannerManager {
public:
  using Ptr = std::shared_ptr<FastPlannerManager>;

  FastPlannerManager();
  ~FastPlannerManager();

  void initPlanModules(const rclcpp::Node::SharedPtr& node);
  void setGlobalWaypoints(std::vector<Eigen::Vector3d>& waypoints);

  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  void planYaw(const Eigen::Vector3d& start_yaw, const double target_yaw = std::numeric_limits<double>::quiet_NaN());
  bool checkTrajCollision(double& distance);

  PlanParameters          pp_;
  LocalTrajData           local_data_;
  MidPlanData             plan_data_;
  EDTEnvironment::Ptr     edt_environment_;
  std::shared_ptr<SdfNode> sdf_map_;  // Make it accessible

private:
  void updateTrajInfo();
  void calcNextYaw(const double& last_yaw, double& yaw);

  rclcpp::Node::SharedPtr               node_;
  rclcpp::Logger                        logger_;
  std::unique_ptr<KinodynamicAstar>     kino_path_finder_;
  std::vector<std::unique_ptr<BsplineOptimizer>> bspline_optimizers_;
};

}  // namespace fast_planner

#endif
