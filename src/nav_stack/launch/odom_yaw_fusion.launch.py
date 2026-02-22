#!/usr/bin/env python3
"""
Launch file for Odometry-Yaw Fusion Node

Fuses ZED camera odometry with WitMotion IMU yaw (with offset correction).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset_deg',
        default_value='-38.0',
        description='Yaw offset in degrees (magnetic declination + mounting offset)'
    )
    
    robot_heading_offset_arg = DeclareLaunchArgument(
        'robot_heading_offset_deg',
        default_value='180.0',
        description='Camera-to-robot forward offset in degrees (180 if camera faces robot rear)'
    )
    
    zed_odom_arg = DeclareLaunchArgument(
        'zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='ZED odometry input topic'
    )
    
    witmotion_yaw_arg = DeclareLaunchArgument(
        'witmotion_yaw_topic',
        default_value='/witmotion_eular/yaw',
        description='WitMotion yaw input topic'
    )
    
    fused_odom_arg = DeclareLaunchArgument(
        'fused_odom_topic',
        default_value='/odom_fused',
        description='Fused odometry output topic'
    )
    
    # Fusion node
    fusion_node = Node(
        package='nav_stack',
        executable='odom_yaw_fusion_node',
        name='odom_yaw_fusion_node',
        output='screen',
        parameters=[{
            'yaw_offset_deg': LaunchConfiguration('yaw_offset_deg'),
            'robot_heading_offset_deg': LaunchConfiguration('robot_heading_offset_deg'),
            'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
            'witmotion_yaw_topic': LaunchConfiguration('witmotion_yaw_topic'),
            'fused_odom_topic': LaunchConfiguration('fused_odom_topic'),
            'output_frame_id': 'odom',
            'output_child_frame_id': 'base_link',
        }]
    )
    
    return LaunchDescription([
        yaw_offset_arg,
        robot_heading_offset_arg,
        zed_odom_arg,
        witmotion_yaw_arg,
        fused_odom_arg,
        fusion_node,
    ])
