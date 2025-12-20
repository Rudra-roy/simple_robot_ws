# MT Unity Sim Package

ROS 2 package for Unity simulation integration with point cloud transformation.

## Features

1. **ROS TCP Endpoint** - Launches the ROS-Unity communication bridge
2. **Robot State Publisher** - Publishes robot TF transforms from URDF/xacro
3. **Static TF Publisher** - Publishes transform between `camera_link` and `camera_optical_frame`
4. **Point Cloud Transform Node** - Converts Unity point cloud from YZX to standard XYZ format

## Installation

Build the package in your ROS 2 workspace:

```bash
cd ~/simple_robot_ws
colcon build --packages-select mt_unity_sim
source install/setup.bash
```

## Usage

Launch the complete Unity simulation bringup:

```bash
ros2 launch mt_unity_sim unity_bringup.launch.py xacro_file:=/path/to/your/robot.urdf.xacro
```

### Launch Arguments

- `xacro_file` - Path to your robot URDF/xacro file (required)
- `input_topic` - Input point cloud topic from Unity (default: `/camera/points_unity`)
- `output_topic` - Output transformed point cloud topic (default: `/camera/points`)

### Example

```bash
ros2 launch mt_unity_sim unity_bringup.launch.py \
    xacro_file:=/path/to/robot.urdf.xacro \
    input_topic:=/camera/points_unity \
    output_topic:=/camera/points
```

## Point Cloud Transform Node

The `pointcloud_transform_node` subscribes to Unity's point cloud data (in YZX format) and republishes it in standard ROS XYZ format.

### Topics

- **Subscribed**: `/camera/points_unity` (sensor_msgs/PointCloud2)
  - Unity format: fields are [y, z, x, rgba]
- **Published**: `/camera/points` (sensor_msgs/PointCloud2)
  - Standard format: fields are [x, y, z, rgba]

### Running Standalone

```bash
ros2 run mt_unity_sim pointcloud_transform_node
```

## Dependencies

- rclpy
- sensor_msgs
- tf2_ros
- robot_state_publisher
- ros_tcp_endpoint

## Notes

- The static transform between `camera_link` and `camera_optical_frame` uses the standard optical frame convention
- All nodes use `use_sim_time: True` for simulation compatibility
