#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_navigation',          # ← your package
            executable='adaptive_planner',        # console_script / node entry
            name='adaptive_planner',
            output='screen',
            parameters=[{
                # core speeds & tolerances
                'linear_speed': 0.3,
                'angular_speed': 0.5,
                'goal_tolerance': 0.15,
                'angle_tolerance': 0.1,

                # straight-line behavior
                'obstacle_threshold': 50,         # cells >= this are obstacles
                'straight_check_horizon_m': 1.5,  # lookahead along the ray
                'los_hysteresis': 3,              # ticks to confirm LoS

                # edge-follow (Bug2-style) tuning
                'treat_unknown_as_free': False,
                'edge_ring_radius_cells': 3,
                'edge_ring_step': 16,
                'edge_side_lock_secs': 2.0,
                'edge_stuck_cycles': 12,
                'edge_worse_ratio': 1.08,

                # visualization frame (change to 'map' if you prefer)
                'path_frame_id': 'odom',
            }]
        ),
    ])
