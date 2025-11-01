# Navigation Stack (nav_stack)

A professional ROS2 navigation stack implementation with modular architecture.

## Overview

This package implements a professional navigation stack with separate global and local planners for robust autonomous navigation.

## Features

### Global Planner Node
- **Goal-based navigation**: Receives goal poses from `/goal_pose` topic
- **Intelligent alignment**: Aligns robot to target direction before moving forward
- **Obstacle detection**: Monitors `/costmap` topic for obstacles in the direct path
- **Safe operation**: Stops robot when obstacles are detected
- **Visual feedback**: Publishes robot boundary visualization to RViz2

## Architecture

```
nav_stack/
├── nav_stack/
│   ├── global_planner_node.py    # Global path planning
│   └── (future: local_planner_node.py)  # Local obstacle avoidance
├── config/
│   └── global_planner_params.yaml
├── launch/
│   └── nav_stack.launch.py
└── README.md
```

## Nodes

### global_planner_node

**Subscribed Topics:**
- `/goal_pose` (geometry_msgs/PoseStamped) - Target goal pose
- `/costmap` (nav_msgs/OccupancyGrid) - Costmap for obstacle detection
- `/odom` (nav_msgs/Odometry) - Robot odometry

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/robot_boundary` (visualization_msgs/Marker) - Robot boundary circle visualization

**Parameters:**
- `robot_radius` (double, default: 0.3) - Robot radius in meters
- `safety_margin` (double, default: 0.2) - Additional safety clearance
- `linear_velocity` (double, default: 0.5) - Forward speed in m/s
- `angular_velocity` (double, default: 0.5) - Rotation speed in rad/s
- `goal_tolerance` (double, default: 0.1) - Distance to consider goal reached
- `angular_tolerance` (double, default: 0.1) - Angle alignment tolerance in radians
- `obstacle_check_distance` (double, default: 2.0) - Look-ahead distance for obstacles
- `costmap_obstacle_threshold` (int, default: 50) - Costmap value threshold (0-100)

## Behavior

1. **Goal Reception**: When a goal pose is received on `/goal_pose`
2. **Alignment**: Robot rotates in place to face the goal direction
3. **Path Checking**: Continuously checks for obstacles in the direct path using costmap
4. **Forward Motion**: Once aligned and path is clear, moves forward toward goal
5. **Obstacle Handling**: If obstacle detected:
   - Logs: "Obstacle found in the path"
   - Logs: "Preparing local planner for obstacle avoidance"
   - Stops the robot and waits for local planner

## Visualization

The package publishes a **robot boundary circle** visualization that appears in RViz2:
- **Frame**: base_link
- **Topic**: /robot_boundary
- **Type**: Cylinder marker (thin disk)
- **Color**: Semi-transparent blue
- **Size**: (robot_radius + safety_margin) * 2

This circle defines the robot's virtual limits for safe navigation.

## Installation

```bash
cd ~/simple_robot_ws/src
# Package already created

cd ~/simple_robot_ws
colcon build --packages-select nav_stack
source install/setup.bash
```

## Usage

### Launch Navigation Stack

```bash
ros2 launch nav_stack nav_stack.launch.py
```

### Send Goal Pose

```bash
# Using command line
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Or use RViz2's "2D Goal Pose" tool (make sure it's remapped to `/goal_pose`).

### Visualize in RViz2

1. Launch RViz2:
```bash
rviz2
```

2. Add the following displays:
   - **RobotModel**: To see the robot URDF
   - **Map**: Subscribe to `/costmap` topic
   - **Marker**: Subscribe to `/robot_boundary` topic
   - **TF**: To see coordinate frames

3. Set **Fixed Frame** to `map` or `odom`

## Configuration

Edit `config/global_planner_params.yaml` to customize:
- Robot dimensions
- Velocity limits
- Tolerance values
- Obstacle detection parameters

## Future Enhancements

- **Local Planner**: Dynamic obstacle avoidance with DWA or TEB
- **Path Visualization**: Display planned path in RViz2
- **Recovery Behaviors**: Handling when robot gets stuck
- **Multi-waypoint Navigation**: Sequential goal execution
- **Dynamic Reconfigure**: Runtime parameter updates

## Dependencies

- rclpy
- geometry_msgs
- nav_msgs
- std_msgs
- tf2_ros
- tf2_geometry_msgs
- sensor_msgs
- visualization_msgs
- numpy

## License

Apache-2.0

## Author

Created for professional mobile robot navigation applications.
