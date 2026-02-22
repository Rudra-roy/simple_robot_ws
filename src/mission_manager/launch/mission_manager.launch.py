#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mission_manager')
    
    # Declare launch arguments
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description='Path to mission YAML file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start mission on launch'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'mission_config.yaml'),
        description='Path to configuration file'
    )
    
    # Create mission coordinator node
    mission_coordinator_node = Node(
        package='mission_manager',
        executable='mission_coordinator_node',
        name='mission_coordinator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'mission_file': LaunchConfiguration('mission_file'),
                'auto_start': LaunchConfiguration('auto_start')
            }
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        mission_file_arg,
        auto_start_arg,
        config_file_arg,
        mission_coordinator_node
    ])
