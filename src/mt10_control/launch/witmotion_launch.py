import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # same-workspace package
    witmoiton_launch = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'launch', 'wt905.launch.py'
    )

    heading = Node(
        package='heading_node',
        executable='heading_node',
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(witmoiton_launch)
        ),
        heading

    ])
