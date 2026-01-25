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

The package publishes extensive **visualization markers** for RViz2:

### Global Planner Visualizations
- **Robot Boundary Cloud** (`/costmap_global/robot_boundary_cloud`)
  - PointCloud2 showing 0.3m radius circle around robot
  - Frame: odom
  - Updates at 5Hz
  
- **Planned Path Line** (`/costmap_global/planned_path_marker`)
  - LINE_STRIP from robot to goal
  - Color: Green (clear path) or Yellow (obstacle detected)
  - Shows direct navigation path

- **Obstacle Check Zone** (`/costmap_global/obstacle_check_zone`)
  - ARROW showing 4m obstacle detection ray
  - Color: Cyan (clear) or Red (obstacle detected)
  - Indicates forward-looking collision detection

- **Goal Marker** (`/costmap_global/goal_marker`)
  - Green SPHERE at goal position
  - Size: 0.5m diameter

### Local Planner Visualizations
- **Scan Direction Rays** (`/incremental_local/scan_directions`)
  - LINE_LIST showing -120 to +120 degree scan range
  - 25 rays color-coded by free distance
  - Color gradient: Red (blocked) to Green (clear)
  - Shows direction analysis during ANALYZING state

- **Chosen Direction Arrow** (`/incremental_local/chosen_direction`)
  - Large ARROW (1.5m) pointing in selected turn direction
  - Color: Blue (left) or Orange (right)
  - Shows active avoidance strategy

- **Rotation Progress** (`/incremental_local/rotation_progress`)
  - LINE_STRIP arc showing accumulated rotation
  - Color: Green (< 60 degrees) to Yellow (60-96 degrees) to Red (> 96 degrees)
  - Displays progress toward 120 degree rotation limit

- **Forward Path Preview** (`/incremental_local/forward_path_preview`)
  - LINE_STRIP showing predicted 2-second trajectory
  - Color: Cyan (rotation phase) or Green (forward movement)
  - Shows planned forward movement path

- **Starting Position** (`/incremental_local/starting_position`)
  - Yellow CYLINDER (disk) at trigger location
  - Semi-transparent marker showing where local planner was activated
  - Reference point for return-to-start behavior

- **Goal Bias Arrow** (`/incremental_local/goal_bias_arrow`)
  - Magenta ARROW from robot toward goal
  - Semi-transparent, length proportional to goal distance (max 2m)
  - Shows goal bias influence on direction selection

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
   - **TF**: To see coordinate frames
   
   **Global Planner Markers:**
   - **PointCloud2**: Subscribe to `/costmap_global/robot_boundary_cloud`
   - **Marker**: Subscribe to `/costmap_global/planned_path_marker`
   - **Marker**: Subscribe to `/costmap_global/obstacle_check_zone`
   - **Marker**: Subscribe to `/costmap_global/goal_marker`
   
   **Local Planner Markers:**
   - **Marker**: Subscribe to `/incremental_local/scan_directions`
   - **Marker**: Subscribe to `/incremental_local/chosen_direction`
   - **Marker**: Subscribe to `/incremental_local/rotation_progress`
   - **Marker**: Subscribe to `/incremental_local/forward_path_preview`
   - **Marker**: Subscribe to `/incremental_local/starting_position`
   - **Marker**: Subscribe to `/incremental_local/goal_bias_arrow`

3. Set **Fixed Frame** to `odom`

**Pro Tip**: Group all `/costmap_global/*` markers in one RViz Display Group and all `/incremental_local/*` markers in another for easy toggling.

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
