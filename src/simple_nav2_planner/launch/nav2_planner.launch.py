#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
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
                'replan_period': 3.0,  # Replan every 1 second
                'replan_distance': 3.0,  # Check 3m ahead for obstacles
                'obstacle_threshold': 50,  # Costmap values >= 50 are obstacles
                'cost_factor': 10.0,  # Weight for costmap costs (higher = stay farther from obstacles)
                'distance_weight': 1.0,  # Weight for distance minimization
            }]
        ),
    ])
