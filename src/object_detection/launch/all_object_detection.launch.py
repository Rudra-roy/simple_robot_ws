#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('object_detection')
    
    # Include mallet detection launch
    mallet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mallet_detection.launch.py')
        )
    )
    
    # Include bottle detection launch
    bottle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'bottle_detection.launch.py')
        )
    )
    
    return LaunchDescription([
        mallet_launch,
        bottle_launch
    ])
