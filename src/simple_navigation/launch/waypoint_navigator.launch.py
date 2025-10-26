from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_navigator = Node(
        package='simple_navigation',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'linear_speed': 0.3,
            'angular_speed': 0.5,
            'goal_tolerance': 0.15,
            'angle_tolerance': 0.1,
        }]
    )
    
    return LaunchDescription([waypoint_navigator])
