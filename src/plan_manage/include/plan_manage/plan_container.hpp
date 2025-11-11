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

#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <bspline/non_uniform_bspline.h>

namespace fast_planner {

struct PlanParameters {
  double max_vel_{0.0};
  double max_acc_{0.0};
  double max_jerk_{0.0};
  double local_traj_len_{0.0};
  double ctrl_pt_dist{0.0};
  double clearance_{0.0};
  int    dynamic_{0};

  double time_search_{0.0};
  double time_optimize_{0.0};
  double time_adjust_{0.0};
};

struct LocalTrajData {
  int                 traj_id_{0};
  double              duration_{0.0};
  rclcpp::Time        start_time_;
  Eigen::Vector3d     start_pos_{Eigen::Vector3d::Zero()};
  NonUniformBspline   position_traj_;
  NonUniformBspline   velocity_traj_;
  NonUniformBspline   acceleration_traj_;
  NonUniformBspline   yaw_traj_;
  NonUniformBspline   yawdot_traj_;
  NonUniformBspline   yawdotdot_traj_;
};

class MidPlanData {
public:
  MidPlanData() = default;
  ~MidPlanData() = default;

  std::vector<Eigen::Vector3d> global_waypoints_;
  std::vector<Eigen::Vector3d> local_start_end_derivative_;
  std::vector<Eigen::Vector3d> kino_path_;
  std::vector<double>          path_yaw_;
  double                       dt_yaw_{0.0};
  double                       dt_yaw_path_{0.0};
};

}  // namespace fast_planner

#endif
