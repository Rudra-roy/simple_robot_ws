#!/usr/bin/env python3
"""
Launch file for the Navigation Stack
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('nav_stack')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'global_planner_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Global Planner Node
    global_planner_node = Node(
        package='nav_stack',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add remappings if needed
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        global_planner_node,
    ])
