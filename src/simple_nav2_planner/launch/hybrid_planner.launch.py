#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the hybrid navigation planner
    """
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    obstacle_radius_arg = DeclareLaunchArgument(
        'obstacle_detection_radius',
        default_value='4.0',
        description='Radius (meters) for obstacle detection triggering survey'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Maximum linear speed (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.3',
        description='Angular speed for rotation (rad/s)'
    )
    
    inflation_radius_arg = DeclareLaunchArgument(
        'obstacle_inflation_radius',
        default_value='0.8',
        description='Safety margin around obstacles (meters)'
    )
    
    # Hybrid navigation planner node
    planner_node = Node(
        package='simple_nav2_planner',
        executable='hybrid_navigation_planner',
        name='hybrid_navigation_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'obstacle_detection_radius': LaunchConfiguration('obstacle_detection_radius'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'obstacle_inflation_radius': LaunchConfiguration('obstacle_inflation_radius'),
            'goal_tolerance': 0.2,
            'survey_angular_speed': 0.3,
            'clearance_margin': 2.0,
        }],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        obstacle_radius_arg,
        linear_speed_arg,
        angular_speed_arg,
        inflation_radius_arg,
        planner_node,
    ])
