#include <algorithm>
#include <cmath>
#include <Eigen/Eigen>
#include <vector>
#include <bspline/non_uniform_bspline.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <plan_manage/msg/bspline.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>

namespace fast_planner {

class TrajServerNode : public rclcpp::Node {
public:
  TrajServerNode()
    : Node("traj_server") {
    declare_parameter<double>("controller.kp", 4.0);
    declare_parameter<double>("controller.kd", 1.5);
    declare_parameter<double>("publish_rate", 100.0);

    kp_ = get_parameter("controller.kp").as_double();
    kd_ = get_parameter("controller.kd").as_double();

    double publish_rate = get_parameter("publish_rate").as_double();
    publish_rate        = std::max(publish_rate, 1.0);
    dt_                 = 1.0 / publish_rate;

    cmd_pub_     = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Visualization markers are published by PlanningVisualization in fast_planner_node
    // traj_vis_pub_ removed to avoid duplication
    ref_path_pub_ = create_publisher<nav_msgs::msg::Path>("traj_server/reference_path", 1);

    bspline_sub_ = create_subscription<plan_manage::msg::Bspline>(
        "planning/bspline", rclcpp::QoS(1),
        std::bind(&TrajServerNode::bsplineCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", rclcpp::SensorDataQoS(),
        std::bind(&TrajServerNode::odomCallback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&TrajServerNode::controlLoop, this));
  }

private:
  void bsplineCallback(const plan_manage::msg::Bspline::SharedPtr msg) {
    if (msg->pos_pts.empty() || msg->knots.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty trajectory");
      return;
    }

    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }

    NonUniformBspline position_traj(pos_pts, msg->order, 0.1);
    position_traj.setKnot(knots);

    NonUniformBspline velocity_traj = position_traj.getDerivative();
    NonUniformBspline acceleration_traj = velocity_traj.getDerivative();

    traj_segments_.clear();
    traj_segments_.push_back(position_traj);
    traj_segments_.push_back(velocity_traj);
    traj_segments_.push_back(acceleration_traj);

    // Parse yaw trajectory if available
    if (!msg->yaw_pts.empty() && msg->yaw_dt > 0.0) {
      Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
      for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
        yaw_pts(i, 0) = msg->yaw_pts[i];
      }
      yaw_traj_ = std::make_unique<NonUniformBspline>(yaw_pts, msg->order, msg->yaw_dt);
      yawdot_traj_ = std::make_unique<NonUniformBspline>(yaw_traj_->getDerivative());
      have_yaw_traj_ = true;
    } else {
      have_yaw_traj_ = false;
    }

    traj_start_time_ = rclcpp::Time(msg->start_time);
    traj_duration_   = position_traj.getTimeSum();
    traj_id_         = msg->traj_id;
    receive_traj_    = true;

    publishReference(msg);
  }

  void publishReference(const plan_manage::msg::Bspline::SharedPtr& msg) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = now();
    path_msg.header.frame_id = "map";

    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }
    NonUniformBspline position_traj(pos_pts, msg->order, 0.1);
    position_traj.setKnot(knots);

    // Publish reference path for trajectory server (used for control)
    // Note: Visualization markers are published by PlanningVisualization in fast_planner_node
    const double dt = 0.1;
    for (double t = 0.0; t <= traj_duration_ + 1e-4; t += dt) {
      auto pose = position_traj.evaluateDeBoorT(t);

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = path_msg.header;
      pose_msg.pose.position.x = pose(0);
      pose_msg.pose.position.y = pose(1);
      pose_msg.pose.position.z = pose(2);
      pose_msg.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose_msg);
    }

    ref_path_pub_->publish(path_msg);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    have_odom_    = true;
  }

  void controlLoop() {
    if (!receive_traj_ || !have_odom_) {
      return;
    }

    auto now_time = now();
    double t = (now_time - traj_start_time_).seconds();

    if (t > traj_duration_) {
      receive_traj_ = false;
      geometry_msgs::msg::Twist stop_cmd;
      cmd_pub_->publish(stop_cmd);
      return;
    }

    auto& position_traj     = traj_segments_[0];
    auto& velocity_traj     = traj_segments_[1];
    auto& acceleration_traj = traj_segments_[2];

    Eigen::Vector3d pos_des = position_traj.evaluateDeBoorT(t);
    Eigen::Vector3d vel_des = velocity_traj.evaluateDeBoorT(t);
    Eigen::Vector3d acc_des = acceleration_traj.evaluateDeBoorT(t);

    Eigen::Vector3d pos_cur(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z);
    Eigen::Vector3d vel_cur(current_odom_.twist.twist.linear.x, current_odom_.twist.twist.linear.y,
                           current_odom_.twist.twist.linear.z);

    Eigen::Vector3d pos_err = pos_des - pos_cur;
    Eigen::Vector3d vel_err = vel_des - vel_cur;

    Eigen::Vector3d desired_acc = acc_des + kp_ * pos_err + kd_ * vel_err;

    geometry_msgs::msg::Twist cmd_msg;
    // Publish global frame velocities (matching Fast-Planner's PositionCommand)
    cmd_msg.linear.x = vel_des(0) + desired_acc(0) * dt_;  // Global X velocity
    cmd_msg.linear.y = vel_des(1) + desired_acc(1) * dt_;  // Global Y velocity
    cmd_msg.linear.z = 0.0;  // 平面机器人：Z轴速度固定为0

    // Add yaw angular velocity if yaw trajectory is available
    if (have_yaw_traj_ && yaw_traj_ && yawdot_traj_) {
      double yaw_des = yaw_traj_->evaluateDeBoorT(t)(0);
      double yawdot_des = yawdot_traj_->evaluateDeBoorT(t)(0);
      
      // Extract current yaw from odometry
      double qx = current_odom_.pose.pose.orientation.x;
      double qy = current_odom_.pose.pose.orientation.y;
      double qz = current_odom_.pose.pose.orientation.z;
      double qw = current_odom_.pose.pose.orientation.w;
      double yaw_cur = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      
      // Normalize yaw error to [-pi, pi]
      double yaw_err = yaw_des - yaw_cur;
      while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
      
      // Add proportional control for yaw (limit the gain to avoid overshoot)
      double yaw_kp = 1.0;  // Reduced yaw proportional gain to avoid spinning too fast
      cmd_msg.angular.z = yawdot_des + yaw_kp * yaw_err;
      
      // Limit angular velocity to reasonable range
      const double max_angular_vel = 2.0;  // rad/s
      if (cmd_msg.angular.z > max_angular_vel) cmd_msg.angular.z = max_angular_vel;
      if (cmd_msg.angular.z < -max_angular_vel) cmd_msg.angular.z = -max_angular_vel;
    } else {
      // If no yaw trajectory, compute yaw from velocity direction
      if (vel_des.norm() > 0.1) {
        double yaw_des = std::atan2(vel_des(1), vel_des(0));
        // Extract current yaw from odometry
        double qx = current_odom_.pose.pose.orientation.x;
        double qy = current_odom_.pose.pose.orientation.y;
        double qz = current_odom_.pose.pose.orientation.z;
        double qw = current_odom_.pose.pose.orientation.w;
        double yaw_cur = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        // Normalize yaw error to [-pi, pi]
        double yaw_err = yaw_des - yaw_cur;
        while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
        
        // Proportional control for yaw (limit the gain)
        double yaw_kp = 1.0;  // Reduced gain
        cmd_msg.angular.z = yaw_kp * yaw_err;
        
        // Limit angular velocity
        const double max_angular_vel = 2.0;  // rad/s
        if (cmd_msg.angular.z > max_angular_vel) cmd_msg.angular.z = max_angular_vel;
        if (cmd_msg.angular.z < -max_angular_vel) cmd_msg.angular.z = -max_angular_vel;
      } else {
        cmd_msg.angular.z = 0.0;
      }
    }

    cmd_pub_->publish(cmd_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  ref_path_pub_;
  rclcpp::Subscription<plan_manage::msg::Bspline>::SharedPtr         bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::TimerBase::SharedPtr                                       control_timer_;

  nav_msgs::msg::Odometry current_odom_;
  bool                    have_odom_{false};
  bool                    receive_traj_{false};

  std::vector<fast_planner::NonUniformBspline> traj_segments_;
  std::unique_ptr<fast_planner::NonUniformBspline> yaw_traj_;
  std::unique_ptr<fast_planner::NonUniformBspline> yawdot_traj_;
  bool                                           have_yaw_traj_{false};
  rclcpp::Time                                 traj_start_time_{0, 0, RCL_ROS_TIME};
  double                                       traj_duration_{0.0};
  int                                          traj_id_{0};

  double kp_{4.0};
  double kd_{1.5};
  double dt_{0.01};
};

}  // namespace fast_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_planner::TrajServerNode>());
  rclcpp::shutdown();
  return 0;
}
