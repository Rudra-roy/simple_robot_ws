import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():



    autonomous = Node(
        package='mt10_control',
        executable='auto_witmotion',
        output='screen'
    )



    return LaunchDescription([
        autonomous
    ])