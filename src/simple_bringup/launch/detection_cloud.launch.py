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
    simple_clouds_share_path = str(get_package_share_path("simple_clouds"))
    
    description = []
    
    
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
