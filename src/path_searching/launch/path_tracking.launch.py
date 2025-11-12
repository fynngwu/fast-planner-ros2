#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动路径跟踪系统的launch文件
    
    同时启动：
    - astar_planner: A*路径规划节点
    - omnidirectional_tracker: 全向轮跟踪节点
    - path_decision: 路径决策节点
    """
    
    # A*路径规划节点
    astar_planner_node = Node(
        package='path_searching',
        executable='astar_planner_node.py',
        name='astar_planner',
        output='screen',
        parameters=[]
    )
    
    # 全向轮跟踪节点
    omnidirectional_tracker_node = Node(
        package='path_searching',
        executable='omnidirectional_tracker_node.py',
        name='omnidirectional_tracker',
        output='screen',
        parameters=[
            {'kp_linear': 1.0},
            {'ki_linear': 0.0},
            {'kd_linear': 0.1},
            {'kp_angular': 1.0},
            {'ki_angular': 0.0},
            {'kd_angular': 0.1},
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.0},
            {'control_frequency': 50.0},
            {'target_distance_threshold': 0.1},
            {'deceleration_rate': 0.95}
        ]
    )
    
    # 路径决策节点
    path_decision_node = Node(
        package='path_searching',
        executable='path_decision_node.py',
        name='path_decision',
        output='screen',
        parameters=[]
    )
    
    # 里程计模拟器节点
    odom_simulator_node = Node(
        package='path_searching',
        executable='odom_simulator.py',
        name='odom_simulator',
        output='screen',
        parameters=[
            {'odom_frame': 'map'},
            {'base_frame': 'base_link'},
            {'publish_rate': 50.0}
        ]
    )
    
    return LaunchDescription([
        astar_planner_node,
        omnidirectional_tracker_node,
        path_decision_node,
        odom_simulator_node,
    ])

