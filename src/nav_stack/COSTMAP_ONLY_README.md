# Costmap-Only Navigation Stack

## Overview

This is an alternative navigation system that **does NOT require a static `/map`**. It relies entirely on the local `/costmap` for navigation and obstacle avoidance. This makes it ideal for:

- **Unknown environments** where no map exists
- **Dynamic environments** with frequently changing obstacles
- **SLAM-based navigation** using rolling costmaps
- **Exploration tasks** without pre-built maps

## Architecture

```
┌────────────────────────────────────────────────────┐
│           COSTMAP-ONLY NAVIGATION                  │
├────────────────────────────────────────────────────┤
│                                                     │
│  Input: /costmap ONLY (no /map required)          │
│                                                     │
│  ┌──────────────────────────────────────────┐    │
│  │  Costmap Global Planner                   │    │
│  │  • Plans direct path to goal              │    │
│  │  • Uses rolling costmap window            │    │
│  │  • Checks obstacles in costmap only       │    │
│  │  • Triggers local planner when blocked    │    │
│  └──────────────────────────────────────────┘    │
│                    ↓                               │
│           (obstacle detected)                      │
│                    ↓                               │
│  ┌──────────────────────────────────────────┐    │
│  │  Costmap Local Planner                    │    │
│  │  • Analyzes left/right free space         │    │
│  │  • Uses costmap data only                 │    │
│  │  • Tangential obstacle avoidance          │    │
│  │  • Returns control when path clear        │    │
│  └──────────────────────────────────────────┘    │
│                                                     │
└────────────────────────────────────────────────────┘
```

## Key Differences from Map-Based System

| Feature | Map-Based System | Costmap-Only System |
|---------|------------------|---------------------|
| **Map Requirement** | Requires `/map` | No map needed |
| **Planning Horizon** | Global (entire map) | Local (costmap window) |
| **Environment Type** | Known, static | Unknown, dynamic |
| **Obstacle Detection** | `/map` + `/costmap` | `/costmap` only |
| **Path Planning** | Long-range optimal | Short-range reactive |
| **Use Case** | Pre-mapped areas | Exploration, SLAM |

## Components

### 1. Costmap Global Planner (`costmap_global_planner_node.py`)

**Responsibilities:**
- Plans direct "bee-line" path to goal
- Monitors costmap for obstacles along path
- Triggers local planner when obstacles detected
- Resumes control when local planner signals clear

**Topics:**
- **Subscribes:**
  - `/goal_pose` (PoseStamped) - Goal position
  - `/costmap` (OccupancyGrid) - Local costmap
  - `/odom` (Odometry) - Robot position
  - `/costmap_planner_state` (String) - Planner coordination

- **Publishes:**
  - `/cmd_vel` (Twist) - Velocity commands
  - `/costmap_local_planner_trigger` (Bool) - Trigger local planner
  - `/costmap_planner_state` (String) - Current planner state
  - `/goal_reached` (Bool) - Goal achievement signal
  
  **Visualization Topics:**
  - `/costmap_global/robot_boundary_cloud` (PointCloud2) - Robot footprint
  - `/costmap_global/planned_path_marker` (Marker) - Path line to goal
  - `/costmap_global/obstacle_check_zone` (Marker) - Detection ray
  - `/costmap_global/goal_marker` (Marker) - Goal sphere

**Parameters:**
- `robot_radius`: 0.6m - Robot footprint radius
- `linear_velocity`: 0.5 m/s - Forward speed
- `angular_velocity`: 0.5 rad/s - Turning speed
- `goal_tolerance`: 0.5m - Goal reached threshold
- `obstacle_check_distance`: 4.0m - Lookahead distance
- `costmap_obstacle_threshold`: 70 - Obstacle cost threshold

### 2. Costmap Local Planner (`costmap_local_planner_node.py`)

**Responsibilities:**
- Activated when global planner encounters obstacles
- Analyzes left/right free space in costmap
- Performs tangential movement around obstacles
- Monitors for clear path to goal
- Returns control to global planner

**Topics:**
- **Subscribes:**
  - `/costmap_local_planner_trigger` (Bool) - Activation signal
  - `/goal_pose` (PoseStamped) - Goal position
  - `/costmap` (OccupancyGrid) - Local costmap
  - `/odometry/filtered` (Odometry) - Robot position

- **Publishes:**
  - `/cmd_vel` (Twist) - Velocity commands
  - `/costmap_planner_state` (String) - Planner state

**Parameters:**
- `robot_radius`: 0.6m
- `linear_velocity`: 0.3 m/s - Slower for safety
- `angular_velocity`: 0.3 rad/s
- `min_free_distance`: 1.0m - Minimum free space needed
- `straight_clear_duration`: 5.0s - Clear path verification time
- `tangent_scan_distance`: 5.0m - Side scan range

## State Machine

### Local Planner States:

```
IDLE ──[triggered]──> ANALYZING ──[direction chosen]──> MOVING_TANGENT
                                                              │
                                                              ├──[path blocked]──┐
                                                              │                  │
                                                              v                  │
                                                        STRAIGHT_CLEAR ──────────┘
                                                              │
                                                              └──[clear for duration]──> signal global
```

## Logic Flow

### Global Planner:
1. Receive goal pose
2. Align robot to goal direction
3. Check path in costmap (bee-line with robot radius)
4. If clear → move forward
5. If obstacle → stop, trigger local planner
6. Wait for local planner to signal clear
7. Resume navigation

### Local Planner:
1. Triggered by global planner
2. Analyze left/right free space in costmap
3. Choose direction based on:
   - Free space distance
   - Angle toward goal
   - Minimum free distance threshold
4. Execute tangential movement (rotate + forward)
5. Monitor for clear path to goal
6. When clear for 5 seconds → signal global planner
7. Return to IDLE

## Usage

### Launch the Costmap-Only Navigation Stack:

```bash
ros2 launch nav_stack costmap_nav_stack.launch.py
```

### With simulation time:

```bash
ros2 launch nav_stack costmap_nav_stack.launch.py use_sim_time:=true
```

### Prerequisites:

You need a costmap provider running. This can be:
- `nav2_costmap_node` package
- SLAM system generating costmaps
- Any node publishing to `/costmap` topic

### Send a Goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## Configuration

Edit `config/costmap_planner_params.yaml`:

```yaml
costmap_global_planner_node:
  ros__parameters:
    robot_radius: 0.6              # Adjust to your robot size
    linear_velocity: 0.5           # Adjust to your robot's max speed
    obstacle_check_distance: 4.0   # Lookahead distance

costmap_local_planner_node:
  ros__parameters:
    tangent_scan_distance: 5.0     # How far to scan sides
    straight_clear_duration: 5.0   # Verification time
    min_free_distance: 1.0         # Minimum space needed
```

## Advantages

✅ **No Static Map Required** - Works in completely unknown environments  
✅ **Dynamic Obstacle Handling** - Reacts to real-time costmap changes  
✅ **Simple Setup** - Only needs costmap node, no map server  
✅ **SLAM Compatible** - Works with rolling SLAM costmaps  
✅ **Exploration Friendly** - Suitable for unmapped areas  

## Limitations

⚠️ **Limited Lookahead** - Only sees within costmap range (typically 10-20m)  
⚠️ **Suboptimal Paths** - No global planning context  
⚠️ **Local Minima Risk** - Can get stuck without global map knowledge  
⚠️ **Slower Navigation** - More reactive, less predictive  

## When to Use

**Use Costmap-Only System:**
- Exploring unknown environments
- No pre-built map available
- Highly dynamic environments
- SLAM-based navigation
- Testing/prototyping without mapping

**Use Map-Based System:**
- Known, static environments
- Pre-built maps available
- Need optimal paths
- Long-range navigation
- Efficiency is critical

## Monitoring

### Check planner state:
```bash
ros2 topic echo /costmap_planner_state
```

### Monitor velocity commands:
```bash
ros2 topic echo /cmd_vel
```

### Check if goal reached:
```bash
ros2 topic echo /goal_reached
```

### Visualize in RViz:

**Basic Displays:**
- Add `/costmap` - OccupancyGrid
- Add `/odom` - Odometry

**Global Planner Visualizations:**
- Add `/costmap_global/robot_boundary_cloud` - PointCloud2 (robot footprint circle)
- Add `/costmap_global/planned_path_marker` - Marker (path line to goal)
- Add `/costmap_global/obstacle_check_zone` - Marker (4m detection ray)
- Add `/costmap_global/goal_marker` - Marker (green goal sphere)

**Local Planner Visualizations:**
- Add `/incremental_local/scan_directions` - Marker (25 scan rays)
- Add `/incremental_local/chosen_direction` - Marker (blue/orange turn arrow)
- Add `/incremental_local/rotation_progress` - Marker (rotation arc)
- Add `/incremental_local/forward_path_preview` - Marker (predicted trajectory)
- Add `/incremental_local/starting_position` - Marker (yellow trigger disk)
- Add `/incremental_local/goal_bias_arrow` - Marker (magenta goal direction)

Set **Fixed Frame** to `odom` for all displays.

## Comparison Example

### Scenario: Navigate around obstacle to goal 10m away

**Map-Based System:**
1. Plans optimal path using global map
2. Follows planned path
3. Uses local planner only when unexpected obstacles appear
4. **Result:** Faster, more efficient

**Costmap-Only System:**
1. Moves toward goal using costmap visibility
2. Detects obstacle in costmap
3. Analyzes left/right using costmap data
4. Performs tangential avoidance
5. Resumes when clear
6. **Result:** More reactive, works without map

## Troubleshooting

**Robot not moving:**
- Check `/costmap` is being published
- Verify `/odom` is available
- Check goal is within reasonable distance

**Robot getting stuck:**
- Increase `tangent_scan_distance`
- Decrease `min_free_distance`
- Check costmap size is adequate

**Excessive turning:**
- Decrease `angular_velocity`
- Increase `straight_clear_duration`
- Tune `costmap_obstacle_threshold`

## Integration with Existing System

Both systems can coexist:

```bash
# Launch map-based system
ros2 launch nav_stack nav_stack.launch.py

# Or launch costmap-only system
ros2 launch nav_stack costmap_nav_stack.launch.py
```

Choose based on your environment and requirements!
