#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value='/media/mt/vol_2/Rover_sim/Assets/uirover_description/urdf/uirover_sim.urdf.xacro',
        description='Path to the robot URDF/xacro file',
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/points_unity',
        description='Input point cloud topic from Unity'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/camera/points',
        description='Output transformed point cloud topic'
    )
    
    # Get launch configurations
    xacro_file = LaunchConfiguration('xacro_file')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    
    # Process the xacro file to get robot description
    robot_description_content = Command(['xacro ', xacro_file])
    
    # 1. Include ROS TCP Endpoint launch file
    ros_tcp_endpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_tcp_endpoint'),
                'launch',
                'endpoint.py'
            ])
        ])
    )
    
    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # 3. Static Transform Publisher: camera_link -> camera_optical_frame
    # Standard optical frame convention: X=right, Y=down, Z=forward
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_frame_publisher',
        arguments=[
            '0', '0', '0',  # x, y, z
            '-1.5708', '0', '-1.5708',  # roll, pitch, yaw (90° rotations)
            'camera_link',
            'camera_optical_frame'
        ],
        output='screen'
    )
    
    # 4. Point Cloud Transform Node
    pointcloud_transform_node = Node(
        package='mt_unity_sim',
        executable='pointcloud_transform_node',
        name='pointcloud_transform_node',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        # Arguments
        xacro_file_arg,
        input_topic_arg,
        output_topic_arg,
        
        # Nodes and launch files
        ros_tcp_endpoint_launch,
        robot_state_publisher,
        static_tf_publisher,
        pointcloud_transform_node,
    ])
