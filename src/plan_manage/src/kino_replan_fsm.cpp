#include <plan_manage/kino_replan_fsm.h>

#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>

namespace fast_planner {

void KinoReplanFSM::init(const rclcpp::Node::SharedPtr& node) {
  node_ = node;

  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  trigger_     = false;

  target_type_      = node_->declare_parameter<int>("fsm.target_type", PRESET_TARGET);
  replan_thresh_    = node_->declare_parameter<double>("fsm.thresh_replan", 1.0);
  no_replan_thresh_ = node_->declare_parameter<double>("fsm.thresh_no_replan", 0.5);
  waypoint_num_     = node_->declare_parameter<int>("fsm.waypoint_num", 0);

  for (int i = 0; i < waypoint_num_; ++i) {
    const std::string prefix = "fsm.waypoint" + std::to_string(i);
    waypoints_[i][0] = node_->declare_parameter<double>(prefix + ".x", 0.0);
    waypoints_[i][1] = node_->declare_parameter<double>(prefix + ".y", 0.0);
    waypoints_[i][2] = node_->declare_parameter<double>(prefix + ".z", 1.0);
    // 添加yaw角配置（可选）
    double yaw = node_->declare_parameter<double>(prefix + ".yaw", std::numeric_limits<double>::quiet_NaN());
    if (!std::isnan(yaw)) {
      // 如果配置了yaw，可以存储到waypoints数组中，或者单独存储
      // 这里暂时不实现，因为yaw规划是自动的
    }
  }
  
  // 添加全局目标yaw配置（可选）
  end_yaw_ = node_->declare_parameter<double>("fsm.end_yaw", std::numeric_limits<double>::quiet_NaN());

  planner_manager_ = std::make_shared<FastPlannerManager>();
  planner_manager_->initPlanModules(node_);
  visualization_ = std::make_shared<PlanningVisualization>(node_);

  exec_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&KinoReplanFSM::execFSMCallback, this));
  safety_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::checkCollisionCallback, this));

  waypoint_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "waypoint_generator/waypoints", rclcpp::QoS(1),
      std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom_world", rclcpp::SensorDataQoS(),
      std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_ = node_->create_publisher<std_msgs::msg::Empty>("planning/replan", 10);
  new_pub_    = node_->create_publisher<std_msgs::msg::Empty>("planning/new", 10);
  bspline_pub_ = node_->create_publisher<plan_manage::msg::Bspline>("planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    return;
  }

  if (msg->poses.front().pose.position.z < -0.1) {
    return;
  }

  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses.front().pose.position.x, msg->poses.front().pose.position.y, 1.0;
  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ > 0) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  } else if (exec_state_ == EXEC_TRAJ) {
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
  }
}

void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;

  // For PRESET_TARGET mode, automatically set the first waypoint when odom is received
  if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ > 0 && !trigger_) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    have_target_ = true;
    trigger_ = true;
    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    RCLCPP_INFO(node_->get_logger(), "Auto-set preset waypoint %d: [%.2f, %.2f, %.2f]", 
                current_wp_, end_pt_(0), end_pt_(1), end_pt_(2));
  }
  
  // 定期打印当前目标点坐标
  if (have_target_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                         "Current target: [%.2f, %.2f, %.2f], Current position: [%.2f, %.2f, %.2f], Distance: %.2f", 
                         end_pt_(0), end_pt_(1), end_pt_(2), 
                         odom_pos_(0), odom_pos_(1), odom_pos_(2),
                         (end_pt_ - odom_pos_).norm());
  }
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, const std::string& pos_call) {
  static const char* state_str[] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int pre_s = static_cast<int>(exec_state_);
  exec_state_ = new_state;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "[%s] %s -> %s", pos_call.c_str(), state_str[pre_s],
                       state_str[static_cast<int>(new_state)]);
}

void KinoReplanFSM::printFSMExecState() {
  static const char* state_str[] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  RCLCPP_INFO(node_->get_logger(), "[FSM] state %s", state_str[static_cast<int>(exec_state_)]);
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) {
      RCLCPP_WARN(node_->get_logger(), "No odometry");
    }
    if (!trigger_) {
      RCLCPP_INFO(node_->get_logger(), "Waiting for goal");
    }
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_ || !trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_) {
        return;
      }
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }

    case GEN_NEW_TRAJ: {
      if (!have_target_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "GEN_NEW_TRAJ: No target set, waiting...");
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      
      // Check if already at target (avoid planning when start == goal)
      if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_) {
        // Already at target, move to next waypoint
        if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ > 0) {
          current_wp_ = (current_wp_ + 1) % waypoint_num_;
          end_pt_(0) = waypoints_[current_wp_][0];
          end_pt_(1) = waypoints_[current_wp_][1];
          end_pt_(2) = waypoints_[current_wp_][2];
          have_target_ = true;
          trigger_ = true;
          visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
          RCLCPP_INFO(node_->get_logger(), "Already at waypoint, moving to next preset waypoint %d: [%.2f, %.2f, %.2f]", 
                      current_wp_, end_pt_(0), end_pt_(1), end_pt_(2));
          // Continue to plan to new waypoint
        } else {
          // Manual target mode, wait for new target
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }
      }
      
      // Use current odometry position as start
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
      start_yaw_(0)         = std::atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                           start_pt_(0), start_pt_(1), start_pt_(2),
                           end_pt_(0), end_pt_(1), end_pt_(2));

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Planning failed, retrying... Target: [%.2f, %.2f, %.2f]",
                             end_pt_(0), end_pt_(1), end_pt_(2));
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto* info   = &planner_manager_->local_data_;
      auto  t_now  = (node_->now() - info->start_time_).seconds();
      t_now        = std::min(info->duration_, t_now);
      auto  pos    = info->position_traj_.evaluateDeBoorT(t_now);

      if (t_now > info->duration_ - 1e-2) {
        // Trajectory finished
        if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ > 0) {
          // Move to next waypoint - 修改为循环逻辑
          current_wp_ = (current_wp_ + 1) % waypoint_num_;
          
          // 设置下一个waypoint（循环执行，不会停止）
          end_pt_(0) = waypoints_[current_wp_][0];
          end_pt_(1) = waypoints_[current_wp_][1];
          end_pt_(2) = waypoints_[current_wp_][2];
          have_target_ = true;
          trigger_ = true;
          visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
          RCLCPP_INFO(node_->get_logger(), "Moving to next preset waypoint %d: [%.2f, %.2f, %.2f]", 
                      current_wp_, end_pt_(0), end_pt_(1), end_pt_(2));
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        } else {
          // Manual target mode, wait for new target
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        return;
      }

      // Check if reached target waypoint
      if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // Reached current waypoint, move to next
        if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ > 0) {
          current_wp_ = (current_wp_ + 1) % waypoint_num_;
          end_pt_(0) = waypoints_[current_wp_][0];
          end_pt_(1) = waypoints_[current_wp_][1];
          end_pt_(2) = waypoints_[current_wp_][2];
          have_target_ = true;
          trigger_ = true;
          visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
          RCLCPP_INFO(node_->get_logger(), "Reached waypoint, moving to next preset waypoint %d: [%.2f, %.2f, %.2f]", 
                      current_wp_, end_pt_(0), end_pt_(1), end_pt_(2));
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        return;
      }

      if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        return;
      }

      changeFSMExecState(REPLAN_TRAJ, "FSM");
      break;
    }

    case REPLAN_TRAJ: {
      auto* info = &planner_manager_->local_data_;
      auto  t_now = (node_->now() - info->start_time_).seconds();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_now);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_now);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_now);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_now)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_now)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_now)[0];

      std_msgs::msg::Empty msg;
      replan_pub_->publish(msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback() {
  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;
    double dist = edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Goal near collision, trigger replanning. Target: [%.2f, %.2f, %.2f], Distance: %.2f", 
                           end_pt_(0), end_pt_(1), end_pt_(2), dist);
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      std_msgs::msg::Empty msg;
      replan_pub_->publish(msg);
    }
  }

  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Current trajectory in collision");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  
  if (!plan_success) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Generate new traj fail. Target: [%.2f, %.2f, %.2f], Start: [%.2f, %.2f, %.2f]", 
                         end_pt_(0), end_pt_(1), end_pt_(2), start_pt_(0), start_pt_(1), start_pt_(2));
    return false;
  }

  planner_manager_->planYaw(start_yaw_);
  auto info = &planner_manager_->local_data_;

  plan_manage::msg::Bspline bspline_msg;
  bspline_msg.order      = 3;
  const auto start_time_ns = info->start_time_.nanoseconds();
  bspline_msg.start_time.sec     = static_cast<int32_t>(start_time_ns / 1000000000);
  bspline_msg.start_time.nanosec = static_cast<uint32_t>(start_time_ns % 1000000000);
  bspline_msg.traj_id    = info->traj_id_;

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline_msg.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline_msg.knots.push_back(knots(i));
  }

  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    bspline_msg.yaw_pts.push_back(yaw_pts(i, 0));
  }
  bspline_msg.yaw_dt = info->yaw_traj_.getInterval();

  bspline_pub_->publish(bspline_msg);

  // Draw kinodynamic search path (yellow spheres) - id 200
  visualization_->drawGeometricPath(planner_manager_->plan_data_.kino_path_, 0.075,
                                    Eigen::Vector4d(1, 1, 0, 0.4), 200);
  // Draw optimized B-spline trajectory (red spheres) - id 300, control points (red) - id 400
  visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                              Eigen::Vector4d(1, 0, 0, 1), 300, 400);

  std_msgs::msg::Empty new_msg;
  new_pub_->publish(new_msg);

  return true;
}

}  // namespace fast_planner
