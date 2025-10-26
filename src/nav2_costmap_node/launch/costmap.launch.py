from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
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

    costmap_node = Node(
        package='nav2_costmap_node',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([costmap_node])
