#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package='uirover_description').find('uirover_description')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/uirover_sim.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation_config.rviz')
    nav2_params_path = os.path.join(pkg_share, 'config/nav2_params.yaml')
    rtabmap_params_path = os.path.join(pkg_share, 'config/rtabmap.yaml')
    
    # Set up environment variables for Gazebo to find resources
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_share
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )
    
    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    
    # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
        
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_use_depth_camera_cmd = DeclareLaunchArgument(
        name='use_depth_camera',
        default_value='True',
        description='Whether to start the depth camera')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    # Robot State Publisher - handled by other launch file
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time, 
    #     'robot_description': Command(['xacro ', model])}],
    #     arguments=[default_urdf_model_path])

    # Gazebo Simulation
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
    #     launch_arguments={
    #         'world': os.path.join(pkg_share, 'worlds', 'marsyard2024.world'),
    #         'verbose': 'true'
    #     }.items(),
    #     condition=IfCondition(use_simulator)
    # )

    # # Launch the robot
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'uirover', 
    #                '-topic', 'robot_description',
    #                '-x', '0',
    #                '-y', '0', 
    #                '-z', '1.2'],
    #     output='screen')

    # # Controller manager
    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    #     ],
    #     output="screen",
    # )

    # # Controller spawners
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # diff_drive_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner", 
    #     arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # Depth image to laser scan converter
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_height': 10,
            'scan_time': 0.033,
            'range_min': 0.45,
            'range_max': 6.0,
            'output_frame': 'depth_camera_optical'
        }],
        remappings=[
            ('depth', '/depth_camera/depth/image_raw'),
            ('depth_camera_info', '/depth_camera/depth/camera_info'),
            ('scan', '/scan')
        ]
    )

    # RTABMap SLAM
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        parameters=[rtabmap_params_path],
        remappings=[
            ('rgb/image', '/depth_camera/image_raw'),
            ('depth/image', '/depth_camera/depth/image_raw'),
            ('rgb/camera_info', '/depth_camera/camera_info'),
            ('odom', '/diff_drive_controller/odom'),
        ],
        arguments=['-d']  # Delete database on start
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        parameters=[rtabmap_params_path],
        remappings=[
            ('rgb/image', '/depth_camera/image_raw'),
            ('depth/image', '/depth_camera/depth/image_raw'),
            ('rgb/camera_info', '/depth_camera/camera_info'),
            ('odom', '/diff_drive_controller/odom'),
        ],
        condition=IfCondition(use_rviz)
    )

    # Nav2 Launch for SLAM mode (with empty map)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': os.path.join(pkg_share, 'maps/empty_map.yaml'),
            'params_file': nav2_params_path,
            'use_sim_time': 'true'
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)

    # Add perception and SLAM nodes
    ld.add_action(depthimage_to_laserscan_node)
    ld.add_action(rtabmap_slam_node)
    ld.add_action(rtabmap_viz_node)

    # Add navigation
    ld.add_action(nav2_launch)

    # Add RViz
    ld.add_action(rviz_node)

    return ld
