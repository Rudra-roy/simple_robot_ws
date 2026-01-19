#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='uirover_description').find('uirover_description')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/uirover_sim.urdf.xacro')
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')
    
    # Set GAZEBO_MODEL_PATH to include this package
    gazebo_model_path = os.path.join(pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_model_path
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )
    
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': ParameterValue(
                Command(['xacro ', model]),
                value_type=str
            )
        }],
        output='screen')

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'drubotara', 
                   '-topic', 'robot_description',
                   '-x', '0',
                   '-y', '0', 
                   '-z', '0.5'],
        output='screen')

    # EKF node for sensor fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom/filtered')]
    )

    # Delayed controller spawners to ensure controller manager is ready (started by Gazebo)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # Diff Drive Controller with correct topic remapping
    diff_drive_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner", 
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(set_gazebo_model_path)

    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(ekf_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)

    return ld