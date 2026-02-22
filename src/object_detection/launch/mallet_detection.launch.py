#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('object_detection')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'mallet_params.yaml'),
        description='Path to configuration file'
    )
    
    # Create node
    mallet_detector_node = Node(
        package='object_detection',
        executable='mallet_detector_node',
        name='mallet_detector_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        mallet_detector_node
    ])
