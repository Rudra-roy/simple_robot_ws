#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('aruco_detection')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'aruco_params.yaml'),
        description='Path to configuration file'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',  # Default to camera index 0
        description='Camera device index or path'
    )
    
    # Create node
    aruco_detector_node = Node(
        package='aruco_detection',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'camera_device': LaunchConfiguration('camera_device')}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        camera_device_arg,
        aruco_detector_node
    ])
