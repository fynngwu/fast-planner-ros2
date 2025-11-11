from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('plan_manage'),
            'config',
            'fast_planner_params.yaml'
        ),
        description='Path to the parameter file for fast_planner and traj_server'
    )

    fast_planner_node = Node(
        package='plan_manage',
        executable='fast_planner_node',
        name='fast_planner_node',
        output='screen',
        parameters=[params_file]
    )

    traj_server_node = Node(
        package='plan_manage',
        executable='traj_server_node',
        name='traj_server',
        output='screen',
        parameters=[params_file]
    )

    odom_sim_node = Node(
        package='plan_manage',
        executable='odom_simulator.py',
        name='odom_simulator',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        declare_params,
        fast_planner_node,
        traj_server_node,
        odom_sim_node,
    ])
