from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    
    # Get package path as string
    simple_clouds_dir = str(get_package_share_path("simple_clouds"))
    
    description = []
    
    # Point cloud generation from RGB-D (using realsense driver's built-in pointcloud)
    # The realsense2_camera driver can generate point clouds directly
    
    
    # Obstacle detection node (Python version)
    description.append(
        Node(
            package="simple_clouds",
            executable="obstacle_detection_node",
            name="obstacle_detection",
            parameters=[
                os.path.join(simple_clouds_dir, "config", "obstacle_detection.yaml"),
                {"use_sim_time": use_sim_time}
            ],
            remappings=[
                ("input", "/camera/camera/depth/color/points"),  # Point cloud from RealSense
                ("output", "/camera/obstacles"),
            ],
            arguments=["--ros-args", "--log-level", "info"],
            output="screen",
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
        OpaqueFunction(function=launch_setup),
    ])
