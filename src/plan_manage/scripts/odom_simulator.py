#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time


class OdomSimulator(Node):
    """Simple omni-directional base simulator that integrates received cmd_vel."""

    def __init__(self) -> None:
        super().__init__('odom_simulator')

        self.declare_parameter('odom_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 50.0)

        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        publish_rate = max(1.0, self.get_parameter('publish_rate').value)

        self._odom_frame = odom_frame or 'odom'
        self._base_frame = base_frame or 'base_link'
        self._dt = 1.0 / publish_rate

        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_z = 0.0
        self._yaw = 0.0

        self._vel_cmd: Twist = Twist()

        self._last_update: Optional[Time] = None

        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_callback, 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, '/odom_world', 10
        )
        self.create_timer(self._dt, self._publish_odometry)

        self.get_logger().info(
            f'Odom simulator started. Publishing {publish_rate:.1f} Hz on /odom_world'
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self._vel_cmd = msg

    def _publish_odometry(self) -> None:
        now = self.get_clock().now().to_msg()
        if self._last_update is None:
            self._last_update = now
            return

        dt = self._dt
        # cmd_vel contains global frame velocities (matching Fast-Planner's PositionCommand)
        vx_global = self._vel_cmd.linear.x  # Global X velocity
        vy_global = self._vel_cmd.linear.y  # Global Y velocity
        wz = self._vel_cmd.angular.z        # Angular velocity (yaw rate)

        # Directly integrate global velocities to update global position
        self._pose_x += vx_global * dt
        self._pose_y += vy_global * dt
        # 平面机器人：固定Z坐标为0
        self._pose_z = 0.0
        self._yaw += wz * dt

        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame

        odom_msg.pose.pose.position.x = self._pose_x
        odom_msg.pose.pose.position.y = self._pose_y
        odom_msg.pose.pose.position.z = self._pose_z

        half_yaw = self._yaw * 0.5
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw)

        odom_msg.twist.twist = self._vel_cmd

        self._odom_pub.publish(odom_msg)


def main() -> None:
    rclpy.init()
    node = OdomSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

