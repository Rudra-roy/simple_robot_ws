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
                "align_depth.enable": "true",
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "2",
                "clip_distance": "5.0",
            }.items(),
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
