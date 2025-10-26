from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import os


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    
    # Paths - convert to strings explicitly
    nav2_bringup_dir = str(get_package_share_path("nav2_bringup"))
    simple_nav2_dir = str(get_package_share_path("simple_nav2"))
    
    # Parameters file
    params_file = os.path.join(simple_nav2_dir, "config", "nav2_params.yaml")
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={},
        convert_types=True
    )
    
    description = []
    
    # Path follower service
    # description.append(
    #     Node(
    #         package="simple_nav2",
    #         executable="path_follower_node.py",
    #         name="path_follower",
    #         parameters=[
    #             os.path.join(simple_nav2_dir, "config", "path_follower.yaml"),
    #             {"use_sim_time": use_sim_time}
    #         ],
    #         output="screen",
    #     )
    # )
    
    # Nav2 navigation (without map server since we use SLAM)
    description.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": str(use_sim_time),
                "params_file": configured_params,
                "use_composition": "False",
                "autostart": "true",
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
        OpaqueFunction(function=launch_setup),
    ])
