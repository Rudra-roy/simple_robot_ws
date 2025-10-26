#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static TF Publisher
    static_tf = Node(
        package='simple_nav2_planner',
        executable='static_tf_publisher',
        name='static_tf_publisher',
        output='screen'
    )
    
    # Costmap parameters
    costmap_params = {
        'global_frame_id': 'odom',
        'base_frame_id': 'base_link',
        'point_cloud_topic': '/rosbot/camera_depth/point_cloud',
        'costmap_topic': '/costmap',
        'publish_frequency': 15.0,
        'resolution': 0.05,
        'width': 300,
        'height': 300,
        'origin_x': -2.5,
        'origin_y': -2.5,
        'inflation_radius': 0.5,
    }
    
    # Costmap Node
    costmap_node = Node(
        package='nav2_costmap_node',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[costmap_params],
    )
    
    # Nav2 Planner Wrapper
    planner_node = Node(
        package='simple_nav2_planner',
        executable='nav2_planner_wrapper',
        name='nav2_planner_wrapper',
        output='screen',
        parameters=[{
            'linear_speed': 0.3,
            'angular_speed': 0.5,
            'goal_tolerance': 0.15,
            'angle_tolerance': 0.1,
            'waypoint_tolerance': 0.3,
            'planner_plugin': 'nav2_navfn_planner::NavfnPlanner',
            'replan_period': 1.0,
            'replan_distance': 3.0,
            'obstacle_threshold': 50,
            'cost_factor': 10.0,
            'distance_weight': 1.0,
        }]
    )
    
    return LaunchDescription([
        static_tf,
        costmap_node,
        planner_node,
    ])
