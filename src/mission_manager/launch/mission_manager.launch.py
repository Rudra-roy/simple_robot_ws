#!/usr/bin/env python3
"""
Mission Manager Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Mission Manager Node
    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        mission_manager_node,
    ])
