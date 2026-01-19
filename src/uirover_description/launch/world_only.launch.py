import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('uirover_description')
    world_path = os.path.join(pkg_share, 'worlds', 'marsyard2024.world')
    
    # Gazebo model:// URI resolution:
    # For model://uirover_description/meshes/file.stl
    # Gazebo looks in GAZEBO_MODEL_PATH/uirover_description/meshes/file.stl
    # So we need to add the parent directory of our package to the path
    
    ros2_ws_install = os.path.dirname(os.path.dirname(pkg_share))  # install/
    ros2_share = os.path.join(ros2_ws_install, 'share')  # install/share/
    
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Build the new model path with the share directory first
    new_paths = [
        ros2_share,  # For model://uirover_description resolution
        os.path.join(pkg_share, 'models'),  # For standalone models
    ]
    
    # Add existing paths
    if existing_model_path:
        new_paths.append(existing_model_path)
    
    new_model_path = ':'.join(new_paths)
    
    # Also set ROS_PACKAGE_PATH to help with package:// URI resolution
    existing_ros_path = os.environ.get('ROS_PACKAGE_PATH', '')
    new_ros_path = pkg_share
    if existing_ros_path:
        new_ros_path = new_ros_path + ':' + existing_ros_path
    
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )
    
    ros_package_path = SetEnvironmentVariable(
        name='ROS_PACKAGE_PATH',
        value=new_ros_path
    )
    
    # Set GAZEBO_PLUGIN_PATH to load gazebo_ros path plugin for package:// support
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    gazebo_ros_lib = os.path.join(ros2_ws_install, 'gazebo_ros', 'lib')
    new_plugin_path = gazebo_ros_lib
    if existing_plugin_path:
        new_plugin_path = new_plugin_path + ':' + existing_plugin_path
    
    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=new_plugin_path
    )
    
    # Important: Set AMENT_PREFIX_PATH so Gazebo can find package resources
    ament_prefix = SetEnvironmentVariable(
        name='AMENT_PREFIX_PATH',
        value=os.environ.get('AMENT_PREFIX_PATH', ros2_ws_install)
    )
    
    return LaunchDescription([
        gazebo_model_path,
        ros_package_path,
        gazebo_plugin_path,
        ament_prefix,
        # Launch Gazebo with ROS plugins
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
                 '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so',
                 world_path],
            output='screen',
            # Pass environment with all the paths set
            additional_env={'GAZEBO_MODEL_PATH': new_model_path,
                          'ROS_PACKAGE_PATH': new_ros_path}
        )
    ])
