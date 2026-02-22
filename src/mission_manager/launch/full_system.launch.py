#!/usr/bin/env python3
"""
Full Autonomous System Launch
Brings up all components: mission manager, detection nodes, and prepares system for operation.
Note: nav_stack should be launched separately as it's in a different package.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    mission_mgr_dir = get_package_share_directory('mission_manager')
    aruco_dir = get_package_share_directory('aruco_detection')
    object_dir = get_package_share_directory('object_detection')
    
    # Launch arguments
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(mission_mgr_dir, 'missions', 'full_mission.yaml'),
        description='Path to mission YAML file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start mission on launch'
    )
    
    # Include mission manager
    mission_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_mgr_dir, 'launch', 'mission_manager.launch.py')
        ),
        launch_arguments={
            'mission_file': LaunchConfiguration('mission_file'),
            'auto_start': LaunchConfiguration('auto_start')
        }.items()
    )
    
    # Include aruco detection
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_dir, 'launch', 'aruco_detection.launch.py')
        )
    )
    
    # Include all object detection
    object_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_dir, 'launch', 'all_object_detection.launch.py')
        )
    )
    
    return LaunchDescription([
        mission_file_arg,
        auto_start_arg,
        mission_manager_launch,
        aruco_launch,
        object_launch
    ])
