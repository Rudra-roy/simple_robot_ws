from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    
    # Get package path as string
    simple_slam_dir = str(get_package_share_path("simple_slam"))
    
    description = []
    
    # Visual odometry node
    description.append(
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            parameters=[
                os.path.join(simple_slam_dir, "config", "rgbd_odometry.yaml"),
                {"use_sim_time": use_sim_time}
            ],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/depth/image_rect_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("odom", "/camera/camera/odom"),
            ],
            arguments=["--ros-args", "--log-level", "info"],
        )
    )
    
    # EKF localization node
    description.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[
                os.path.join(simple_slam_dir, "config", "ekf_filter.yaml"),
                {"use_sim_time": use_sim_time}
            ],
            remappings=[
                ("odometry/filtered", "/odometry/filtered"),
            ],
            arguments=["--ros-args", "--log-level", "info"],
        )
    )
    
    # RTAB-Map SLAM node
    description.append(
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            parameters=[
                os.path.join(simple_slam_dir, "config", "rtabmap_slam.yaml"),
                {"use_sim_time": use_sim_time}
            ],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/depth/image_rect_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("odom", "/odometry/filtered"),
            ],
            arguments=["-d", "--ros-args", "--log-level", "info"],  # Delete database on start
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
