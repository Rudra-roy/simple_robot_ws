import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():


    # Aruco detection node
    aruco_node = Node(
        package='mt10_control',
        executable='aruco_new',
        output='screen'
    )

    # Mallet detection node
    mallet_node = Node(
        package='mt10_control',
        executable='mallet',
        output='screen'
    )

    # Bottle detection node
    bottle_node = Node(
        package='mt10_control',
        executable='bottle',
        output='screen'
    )

    


    return LaunchDescription([
        aruco_node,
        mallet_node,
        bottle_node
    ])