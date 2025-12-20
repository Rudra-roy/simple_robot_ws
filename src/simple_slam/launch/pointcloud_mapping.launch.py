from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """
    Complete mapping solution with RTAB-Map using point clouds
    Includes TF tree setup and visualization
    """
    
    simple_slam_dir = str(get_package_share_path("simple_slam"))
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )
    
    # RTAB-Map SLAM node (mapping mode)
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            os.path.join(simple_slam_dir, "config", "rtabmap_ground_filtered.yaml"),  # Use ground-filtered config
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"database_path": ""},  # Don't save database
        ],
        remappings=[
            ("scan_cloud", "/camera/points"),  # Point cloud topic from depth camera
            ("odom", "/odom"),  # External odometry from rosbot
            ("grid_map", "/map"),
        ],
        arguments=[
            "-d",  # Delete database on start
            "--ros-args", 
            "--log-level", 
            "info"
        ],
    )
    
    # RTAB-Map visualization
    # rtabmap_viz_node = Node(
    #     package="rtabmap_viz",
    #     executable="rtabmap_viz",
    #     name="rtabmap_viz",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
    #     ],
    #     remappings=[
    #         ("scan_cloud", "/camera/camera/depth/color/points"),
    #         ("odom", "/odom"),
    #     ],
    # )
    
    # Static transform from base_link to camera_depth (adjust based on your robot)
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
            "--x", "0.1",
            "--y", "0.0", 
            "--z", "0.3",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0"
        ],
    )
    
    # Map to odom transform (published by RTAB-Map, but this ensures it exists)
    # Comment this out if RTAB-Map already publishes map->odom
    # map_to_odom_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_odom_tf",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    # )
    
    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_node,
        # rtabmap_viz_node,
        base_to_camera_tf,
    ])
