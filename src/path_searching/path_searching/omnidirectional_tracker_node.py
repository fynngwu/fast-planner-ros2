#!/usr/bin/env python3
"""
全向轮路径跟踪节点

功能：
- 订阅path和odom_world，使用PID控制跟踪路径
- 订阅can_go信号，只有为True时才发布cmd_vel
- 当can_go为False时，平滑减速直到停止
- 全向轮可以直接控制x、y方向的线速度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np


class OmnidirectionalTrackerNode(Node):
    """全向轮路径跟踪节点"""
    
    def __init__(self):
        super().__init__('omnidirectional_tracker')
        
        # 参数
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('target_distance_threshold', 0.1)
        self.declare_parameter('deceleration_rate', 0.95)  # 减速系数
        
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        self.kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        self.kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.target_distance_threshold = self.get_parameter('target_distance_threshold').get_parameter_value().double_value
        self.deceleration_rate = self.get_parameter('deceleration_rate').get_parameter_value().double_value
        
        # 状态
        self.have_path = False
        self.have_odom = False
        self.can_go = False
        self.current_path = None
        self.current_path_index = 0
        
        # 当前位姿
        self.current_pos = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.current_yaw = 0.0
        
        # PID控制状态
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_yaw = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0
        
        # 当前速度（用于平滑减速）
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_yaw = 0.0
        
        # 创建订阅者（使用TRANSIENT_LOCAL QoS）
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.path_callback,
            path_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )
        
        self.can_go_sub = self.create_subscription(
            Bool,
            '/can_go',
            self.can_go_callback,
            10
        )
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 创建控制定时器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_callback)
        
        self.get_logger().info('Omnidirectional Tracker Node started')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel}m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel}rad/s')
    
    def path_callback(self, msg):
        """处理路径消息"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.current_path_index = 0
        self.have_path = True
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} points')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # 从四元数提取yaw角
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.have_odom = True
    
    def can_go_callback(self, msg):
        """处理can_go信号"""
        self.can_go = msg.data
        if not self.can_go:
            self.get_logger().info('Received can_go=False, will decelerate')
    
    def get_next_target(self):
        """获取下一个目标点"""
        if not self.have_path or self.current_path is None:
            return None
        
        if self.current_path_index >= len(self.current_path.poses):
            return None
        
        pose = self.current_path.poses[self.current_path_index].pose
        target = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # 检查是否到达当前目标点
        dist = np.linalg.norm(target[:2] - self.current_pos[:2])
        if dist < self.target_distance_threshold:
            # 到达目标点，移动到下一个点
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path.poses):
                # 路径完成
                return None
            pose = self.current_path.poses[self.current_path_index].pose
            target = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        return target
    
    def compute_pid_control(self, target):
        """计算PID控制指令"""
        if target is None:
            return 0.0, 0.0, 0.0
        
        # 计算位置误差（世界坐标系）
        error_x = target[0] - self.current_pos[0]
        error_y = target[1] - self.current_pos[1]
        
        # 计算期望航向角（朝向目标点）
        desired_yaw = math.atan2(error_y, error_x)
        error_yaw = desired_yaw - self.current_yaw
        
        # 归一化角度误差到[-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2.0 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2.0 * math.pi
        
        # PID控制
        dt = 1.0 / self.control_frequency
        
        # X方向PID
        self.integral_error_x += error_x * dt
        derivative_error_x = (error_x - self.last_error_x) / dt
        vel_x = self.kp_linear * error_x + self.ki_linear * self.integral_error_x + self.kd_linear * derivative_error_x
        
        # Y方向PID
        self.integral_error_y += error_y * dt
        derivative_error_y = (error_y - self.last_error_y) / dt
        vel_y = self.kp_linear * error_y + self.ki_linear * self.integral_error_y + self.kd_linear * derivative_error_y
        
        # 航向角PID
        self.integral_error_yaw += error_yaw * dt
        derivative_error_yaw = (error_yaw - self.last_error_yaw) / dt
        vel_yaw = self.kp_angular * error_yaw + self.ki_angular * self.integral_error_yaw + self.kd_angular * derivative_error_yaw
        
        # 更新上次误差
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_yaw = error_yaw
        
        # 限制积分饱和
        self.integral_error_x = max(-1.0, min(1.0, self.integral_error_x))
        self.integral_error_y = max(-1.0, min(1.0, self.integral_error_y))
        self.integral_error_yaw = max(-1.0, min(1.0, self.integral_error_yaw))
        
        return vel_x, vel_y, vel_yaw
    
    def control_callback(self):
        """控制回调函数"""
        if not self.have_path or not self.have_odom:
            return
        
        if self.current_path is None or len(self.current_path.poses) == 0:
            return
        
        # 获取目标点
        target = self.get_next_target()
        
        if target is None:
            # 路径完成，停止
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.have_path = False
            self.current_vel_x = 0.0
            self.current_vel_y = 0.0
            self.current_vel_yaw = 0.0
            self.get_logger().info('Path completed, stopping')
            return
        
        # 计算PID控制指令
        vel_x, vel_y, vel_yaw = self.compute_pid_control(target)
        
        # 根据can_go信号决定是否发布速度
        if not self.can_go:
            # can_go为False，平滑减速
            self.current_vel_x *= self.deceleration_rate
            self.current_vel_y *= self.deceleration_rate
            self.current_vel_yaw *= self.deceleration_rate
            
            # 如果速度很小，直接设为0
            if abs(self.current_vel_x) < 0.01:
                self.current_vel_x = 0.0
            if abs(self.current_vel_y) < 0.01:
                self.current_vel_y = 0.0
            if abs(self.current_vel_yaw) < 0.01:
                self.current_vel_yaw = 0.0
        else:
            # can_go为True，正常控制
            # 限幅
            vel_x = max(-self.max_linear_vel, min(self.max_linear_vel, vel_x))
            vel_y = max(-self.max_linear_vel, min(self.max_linear_vel, vel_y))
            vel_yaw = max(-self.max_angular_vel, min(self.max_angular_vel, vel_yaw))
            
            # 更新当前速度
            self.current_vel_x = vel_x
            self.current_vel_y = vel_y
            self.current_vel_yaw = vel_yaw
        
        # 发布cmd_vel
        cmd = Twist()
        cmd.linear.x = float(self.current_vel_x)
        cmd.linear.y = float(self.current_vel_y)
        cmd.angular.z = float(self.current_vel_yaw)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

