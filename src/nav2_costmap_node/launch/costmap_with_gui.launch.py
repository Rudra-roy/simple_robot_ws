#!/usr/bin/env python3
"""
Launch file for costmap node WITH dynamic reconfigure GUI.
This will start the costmap node and automatically open rqt_reconfigure
for live parameter tuning.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    params = {
        'global_frame_id': 'odom',  # Costmap is in global frame (odom/map)
        'base_frame_id': 'zed_camera_link',
        'point_cloud_topic': '/zed/zed_node/point_cloud/cloud_registered',
        'costmap_topic': '/costmap',
        'publish_frequency': 7.0,
        'resolution': 0.05,
        'width': 1000,
        'height': 1000,
        'origin_x': 0.0,
        'origin_y': 0.0,
        'inflation_radius': 0.3,
        'cost_scaling_factor': 10.0,
        'obstacle_max_range': 20.0,
        'obstacle_min_range': 0.3,
        'min_obstacle_z': 0.2,  # Ground is BELOW camera - filter points below -0.8m
        'max_obstacle_z': 4.0,   # Camera at Z=0 - obstacles between ground and camera
        'costmap_origin_z': -0.83,  # PUT COSTMAP AT GROUND LEVEL (adjust to your camera height!)
    }

    costmap_node = Node(
        package='nav2_costmap_node',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[params],
    )

    # Launch rqt_reconfigure GUI after a short delay to allow node to start
    rqt_reconfigure = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
                output='screen',
                shell=False,
            )
        ]
    )

    return LaunchDescription([
        costmap_node,
        rqt_reconfigure,
    ])
