#!/usr/bin/env python3
"""
Launch file for Costmap-Only Navigation Stack
No dependency on static /map - uses only /costmap
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
    config_file = os.path.join(pkg_dir, 'config', 'costmap_planner_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Costmap-Only Global Planner Node
    costmap_global_planner_node = Node(
        package='nav_stack',
        executable='costmap_global_planner_node',
        name='costmap_global_planner_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Uses /costmap instead of /map
        ]
    )
    
    # Costmap-Only Local Planner Node
    costmap_local_planner_node = Node(
        package='nav_stack',
        executable='costmap_local_planner_node',
        name='costmap_local_planner_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Uses /costmap instead of /map
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        costmap_global_planner_node,
        costmap_local_planner_node,
    ])
