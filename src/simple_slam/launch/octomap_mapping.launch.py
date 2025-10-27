from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Alternative mapping using Octomap for 3D point cloud mapping
    with 2D projection for navigation
    """
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
    )
    
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/camera/camera/depth/color/points",
        description="Point cloud topic from sensor",
    )
    
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Octomap resolution in meters",
    )
    
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Map frame ID",
    )
    
    # Octomap server node
    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "resolution": LaunchConfiguration("resolution"),
                "frame_id": LaunchConfiguration("frame_id"),
                "sensor_model/max_range": 5.0,
                "sensor_model/min_range": 0.2,
                "latch": True,
                "height_map": True,  # Generate 2D height map projection
                "colored_map": False,
                "occupancy_min_z": -0.5,  # Minimum height to consider
                "occupancy_max_z": 2.0,   # Maximum height to consider
                "filter_ground": True,
                "ground_filter/distance": 0.04,
                "ground_filter/angle": 0.15,
                "ground_filter/plane_distance": 0.07,
                "compress_map": True,
                "incremental_2D_projection": True,
            }
        ],
        remappings=[
            ("cloud_in", LaunchConfiguration("cloud_topic")),
            ("projected_map", "/map"),  # 2D occupancy grid for navigation
        ],
    )
    
    # Point cloud filter (optional - downsample point cloud before mapping)
    voxel_grid_node = Node(
        package="pcl_ros",
        executable="voxel_grid",
        name="voxel_grid_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "leaf_size": 0.02,  # 2cm voxel size
                "filter_field_name": "z",
                "filter_limit_min": -1.0,
                "filter_limit_max": 3.0,
                "filter_limit_negative": False,
            }
        ],
        remappings=[
            ("input", LaunchConfiguration("cloud_topic")),
            ("output", "/cloud_filtered"),
        ],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        cloud_topic_arg,
        resolution_arg,
        frame_id_arg,
        voxel_grid_node,
        octomap_server_node,
    ])
