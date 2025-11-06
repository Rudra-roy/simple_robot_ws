# Navigation Stack (nav_stack)

Professional ROS2 navigation with coordinated global and local planners.

## Overview

Dual-planner navigation system: Global planner for direct navigation, Local planner for tangential obstacle avoidance.

## Architecture

```
nav_stack/
├── nav_stack/
│   ├── global_planner_node.py    # Direct path navigation with bee-line collision checking
│   └── local_planner_node.py     # Tangential obstacle avoidance
├── config/
│   └── global_planner_params.yaml
├── launch/
│   └── nav_stack.launch.py       # Launches both planners
└── README.md
```

## Coordination

- **Global Planner** navigates directly to goal, monitors bee-line (swept circle) for obstacles within 2m
- Triggers **Local Planner** when obstacle blocks bee-line
- **Local Planner** performs 360° scan, finds free tangent direction, navigates around obstacle
- After 5s clear path, checks if direct goal path is clear
- Signals Global Planner to resume when clear

## Topics

**Coordination:**
- `/local_planner_trigger` (Bool) - Global → Local activation
- `/planner_state` (String) - Coordination: "global_active", "local_active"
- `/goal_reached` (Bool) - Waypoint completion signal

**Navigation:**
- `/goal_pose` (PoseStamped) - Target goals
- `/costmap` (OccupancyGrid) - Local obstacle detection
- `/map` (OccupancyGrid) - Global map for analysis
- `/odom` (Odometry) - Robot pose
- `/cmd_vel` (Twist) - Velocity commands

## Nodes

### global_planner_node
Direct navigation with bee-line collision checking (swept circle at robot_radius=0.3m along path).

### local_planner_node
Tangential approach: 360° rotation → free direction analysis → tangent movement → 5s straight test → goal check.
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
