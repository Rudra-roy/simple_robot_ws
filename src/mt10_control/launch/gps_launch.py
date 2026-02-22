import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # same-workspace package
    sbg_launch = os.path.join(
        get_package_share_directory('mt10_control'),
        'launch', 'mt_sbg_launch.py'
    )

    best_gps = Node(
        package='mt10_control',
        executable='best_gps',
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sbg_launch)
        ),
        best_gps

    ])
