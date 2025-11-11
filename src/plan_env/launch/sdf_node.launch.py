from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('plan_env')
    
    # Path to the parameter file (without config/ subdirectory since files are installed to share)
    params_file = os.path.join(package_dir, 'config', 'sdf_node_params.yaml')
    
    # SDF Node with parameters from YAML file
    sdf_node = Node(
        package='plan_env',
        executable='sdf_node',
        name='sdf_node',
        parameters=[params_file],
        output='screen'
    )
    
    return LaunchDescription([
        sdf_node,
    ])

