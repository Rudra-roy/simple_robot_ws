#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    target_lat_arg = DeclareLaunchArgument(
        'target_latitude',
        default_value='0.0',
        description='Target latitude coordinate'
    )
    
    target_lon_arg = DeclareLaunchArgument(
        'target_longitude',
        default_value='0.0',
        description='Target longitude coordinate'
    )
    
    gps_navigation_node = Node(
        package='simple_gps_nav',
        executable='gps_navigation_node',
        name='gps_navigation_node',
        output='screen',
        parameters=[],
    )
    
    target_publisher_node = Node(
        package='simple_gps_nav',
        executable='target_publisher_node',
        name='target_publisher_node',
        output='screen',
        parameters=[
            {'target_latitude': LaunchConfiguration('target_latitude')},
            {'target_longitude': LaunchConfiguration('target_longitude')},
            {'publish_rate': 1.0}
        ],
    )
    
    return LaunchDescription([
        target_lat_arg,
        target_lon_arg,
        gps_navigation_node,
        target_publisher_node,
    ])
