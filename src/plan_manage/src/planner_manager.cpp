#include <plan_manage/planner_manager.h>

#include <chrono>

namespace fast_planner {

FastPlannerManager::FastPlannerManager()
  : logger_(rclcpp::get_logger("plan_manage::FastPlannerManager")) {
}

FastPlannerManager::~FastPlannerManager() = default;

void FastPlannerManager::initPlanModules(const rclcpp::Node::SharedPtr& node) {
  node_   = node;
  logger_ = node_->get_logger();

  pp_.max_vel_        = node_->declare_parameter("manager.max_vel", -1.0);
  pp_.max_acc_        = node_->declare_parameter("manager.max_acc", -1.0);
  pp_.max_jerk_       = node_->declare_parameter("manager.max_jerk", -1.0);
  pp_.dynamic_        = node_->declare_parameter("manager.dynamic_environment", 0);
  pp_.clearance_      = node_->declare_parameter("manager.clearance_threshold", 0.3);
  pp_.local_traj_len_ = node_->declare_parameter("manager.local_segment_length", 4.0);
  pp_.ctrl_pt_dist    = node_->declare_parameter("manager.control_points_distance", 0.2);

  const bool use_geometric_path   = node_->declare_parameter("manager.use_geometric_path", false);
  const bool use_kinodynamic_path = node_->declare_parameter("manager.use_kinodynamic_path", true);
  const bool use_optimization     = node_->declare_parameter("manager.use_optimization", true);

  // Initialize mapping interface
  sdf_map_ = std::make_shared<SdfNode>();
  edt_environment_ = std::make_shared<EDTEnvironment>();
  edt_environment_->setMap(sdf_map_);
  edt_environment_->init();

  if (use_geometric_path) {
    RCLCPP_WARN(logger_, "Geometric planner not yet integrated in ROS 2 port");
  }

  if (use_kinodynamic_path) {
    kino_path_finder_ = std::make_unique<KinodynamicAstar>();
    kino_path_finder_->setParam(node_);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(2);
    for (auto& opt : bspline_optimizers_) {
      opt = std::make_unique<BsplineOptimizer>();
      opt->setParam(node_);
      opt->setEnvironment(edt_environment_);
    }
  }

  local_data_.traj_id_ = 0;
}

void FastPlannerManager::setGlobalWaypoints(std::vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  if (!edt_environment_) {
    return true;
  }

  const auto time_now = node_->now();
  const double t_now  = (time_now - local_data_.start_time_).seconds();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {
  RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
                       "[kino replan] start. Start: [%.2f, %.2f, %.2f], Goal: [%.2f, %.2f, %.2f]",
                       start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));

  if (!kino_path_finder_) {
    RCLCPP_ERROR(logger_, "Kinodynamic planner not initialized");
    return false;
  }

  if ((start_pt - end_pt).norm() < 0.2) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                         "Goal too close to start. Start: [%.2f, %.2f, %.2f], Goal: [%.2f, %.2f, %.2f], Distance: %.2f", 
                         start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2), (start_pt - end_pt).norm());
    return false;
  }

  // Check if start or goal is in obstacle
  if (sdf_map_) {
    int start_occ = sdf_map_->getInflateOccupancy(start_pt);
    int goal_occ = sdf_map_->getInflateOccupancy(end_pt);
    double start_dist = sdf_map_->getDistance(start_pt);
    double goal_dist = sdf_map_->getDistance(end_pt);
    
    if (start_occ == 1) {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                           "Start point in obstacle! Start: [%.2f, %.2f, %.2f], Distance: %.2f",
                           start_pt(0), start_pt(1), start_pt(2), start_dist);
    }
    if (goal_occ == 1) {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                           "Goal point in obstacle! Goal: [%.2f, %.2f, %.2f], Distance: %.2f",
                           end_pt(0), end_pt(1), end_pt(2), goal_dist);
    }
    if (start_occ == -1 || goal_occ == -1) {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                           "Start or goal point outside map! Start_occ: %d, Goal_occ: %d",
                           start_occ, goal_occ);
    }
  }

  local_data_.start_time_ = node_->now();
  double t_search = 0.0;
  double t_opt    = 0.0;
  double t_adjust = 0.0;

  auto t1 = node_->now();

  kino_path_finder_->reset();
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                         "Kinodynamic search failed, retry with discontinuous start. Start: [%.2f, %.2f, %.2f], Goal: [%.2f, %.2f, %.2f]",
                         start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1000,
                            "Cannot find kinodynamic path. Start: [%.2f, %.2f, %.2f], Goal: [%.2f, %.2f, %.2f], Distance: %.2f",
                            start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2), (start_pt - end_pt).norm());
      return false;
    }
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);
  t_search              = (node_->now() - t1).seconds();

  double ts = std::max(0.05, pp_.ctrl_pt_dist / std::max(pp_.max_vel_, 1e-3));
  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  t1 = node_->now();
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  if (!bspline_optimizers_.empty() && bspline_optimizers_[0]) {
    ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);
  }
  t_opt = (node_->now() - t1).seconds();

  NonUniformBspline pos(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);

  bool feasible = pos.checkFeasibility(false);
  int  iter_num = 0;
  while (!feasible && iter_num < 3) {
    feasible = pos.reallocateTime();
    ++iter_num;
  }

  if (!feasible) {
    RCLCPP_WARN(logger_, "Trajectory remains infeasible after reallocation");
  }

  t_adjust              = 0.0;  // reallocateTime operates in-place, ignore profiling for now
  local_data_.position_traj_ = pos;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  if (bspline_optimizers_.size() < 2 || !bspline_optimizers_[1]) {
    RCLCPP_WARN(logger_, "Yaw optimizer not initialized");
    return;
  }

  auto t1 = node_->now();

  auto& pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = std::max(0.1, std::min(0.3, duration / 10.0));
  int    seg_num = std::max(3, static_cast<int>(std::ceil(duration / dt_yaw)));
  dt_yaw         = duration / seg_num;

  const double forward_t = 2.0;
  double       last_yaw  = start_yaw(0);
  std::vector<Eigen::Vector3d> waypts;
  std::vector<int>             waypt_idx;

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = std::min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt = Eigen::Vector3d::Zero();
    if (pd.norm() > 1e-6) {
      waypt(0) = std::atan2(pd(1), pd(0));
      calcNextYaw(last_yaw, waypt(0));
      waypt(1) = waypt(2) = 0.0;
      last_yaw = waypt(0);
    } else if (!waypts.empty()) {
      waypt = waypts.back();
    }

    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0,
      -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(std::max(0.0, duration - 0.1));
  Eigen::Vector3d end_yaw(std::atan2(end_v(1), end_v(0)), 0.0, 0.0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  std::vector<double> path_yaw;
  for (const auto& wp : waypts) {
    path_yaw.push_back(wp[0]);
  }

  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  RCLCPP_INFO(logger_, "plan yaw done in %.3f s", (node_->now() - t1).seconds());
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (std::fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
