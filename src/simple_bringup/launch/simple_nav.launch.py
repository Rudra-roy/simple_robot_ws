from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "false"
    enable_pointcloud = LaunchConfiguration("enable_pointcloud").perform(context).lower() == "true"
    
    # Get package paths as strings
    realsense_share_path = str(get_package_share_path("realsense2_camera"))
    simple_clouds_share_path = str(get_package_share_path("simple_clouds"))
    simple_slam_share_path = str(get_package_share_path("simple_slam"))
    simple_nav2_share_path = str(get_package_share_path("simple_nav2"))
    simple_bringup_share_path = str(get_package_share_path("simple_bringup"))
    
    description = []
    
    # RealSense camera
    description.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share_path, "launch", "rs_launch.py")
            ),
            launch_arguments={
                "camera_name": "camera",
                "camera_namespace": "camera",
                "pointcloud.enable": str(enable_pointcloud),
                "enable_sync": "true",
                "align_depth.enable": "false",
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "enable_fisheye1": "false",
                "enable_fisheye2": "false",
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "2",
                "clip_distance": "5.0",
            }.items(),
        )
    )
    
    # Static transform from camera_link to camera_color_optical_frame
    # description.append(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="camera_base_link",
    #         arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_link"],
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     )
    # )
    
    # Point cloud processing (delayed to let camera start)
    if enable_pointcloud:
        description.append(
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(simple_clouds_share_path, "launch", "clouds.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": str(use_sim_time),
                        }.items(),
                    )
                ]
            )
        )
    
    # # SLAM (delayed to let camera start)
    description.append(
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(simple_slam_share_path, "launch", "slam.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": str(use_sim_time),
                    }.items(),
                )
            ]
        )
    )
    
    # # Navigation (delayed to let SLAM start)
    description.append(
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(simple_nav2_share_path, "launch", "nav2.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": str(use_sim_time),
                    }.items(),
                )
            ]
        )
    )
    
    # # RViz (optional)
    rviz_config_path = os.path.join(simple_bringup_share_path, "config", "simple_nav.rviz")
    if os.path.exists(rviz_config_path):
        description.append(
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", rviz_config_path],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    )
                ]
            )
        )
    
    return description


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        ),
        DeclareLaunchArgument(
            "enable_pointcloud",
            default_value="true",
            description="Enable point cloud generation for obstacle detection",
        ),
        OpaqueFunction(function=launch_setup),
    ])
