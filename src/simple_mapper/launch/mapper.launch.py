#!/usr/bin/env python3
"""
Launch file for the simple mapper node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('simple_mapper'),
            'config',
            'mapper_params.yaml'
        ]),
        description='Path to mapper configuration file'
    )
    
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/camera/depth/color/points',
        description='Point cloud topic to subscribe to'
    )
    
    # Mapper node
    mapper_node = Node(
        package='simple_mapper',
        executable='mapper_node',
        name='mapper_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'cloud_topic': LaunchConfiguration('cloud_topic')}
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        cloud_topic_arg,
        mapper_node
    ])
