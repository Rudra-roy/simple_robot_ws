# Mission Manager Package

A ROS 2 package for sequential waypoint navigation management. The Mission Manager receives a list of waypoints, sends them one-by-one to the global planner, and coordinates task-specific actions based on waypoint types.

## Overview

The Mission Manager acts as a coordinator between high-level mission planning and low-level navigation. It handles:

- Sequential waypoint execution
- Task-specific routing based on waypoint names
- Mission state management
- Coordination with the global planner and task subsystems

## Architecture

```
                                    /waypoints (nav_msgs/Path)
                                           |
                                           v
                              +------------------------+
                              |   Mission Manager Node |
                              +------------------------+
                                           |
                     +---------------------+---------------------+
                     |                                           |
                     v                                           v
            /goal_pose (PoseStamped)                   /current_waypoint_name
                     |
                     v
         +------------------------+
         |    Global Planner      |
         +------------------------+
                     |
    +----------------+----------------+----------------+----------------+
    |                |                |                |                |
    v                v                v                v                v
/waypoint_reached  /aruco         /mallet         /bottle        /rock_pick
  (to Mission     (continue)     (continue)      (continue)      (continue)
   Manager)
```

## Waypoint Message Format

### Input Topic: `/waypoints`
- **Type:** `nav_msgs/Path`
- **Format:** Each `PoseStamped` in the path contains:
  - `pose.position.x`: X coordinate (meters)
  - `pose.position.y`: Y coordinate (meters)
  - `pose.orientation`: Yaw as quaternion
  - `header.frame_id`: Waypoint name (used for task routing)

### Waypoint Naming Convention

Waypoint names determine the action taken when the robot reaches that waypoint:

| Keyword in Name | Action | Target Topic |
|-----------------|--------|--------------|
| `GNSS` | Send confirmation to mission manager | `/waypoint_reached` |
| `Aruco` | Send "continue" signal | `/aruco` |
| `Mallet` | Send "continue" signal | `/mallet` |
| `Bottle` | Send "continue" signal | `/bottle` |
| `Rock Pick` | Send "continue" signal | `/rock_pick` |

**Note:** If no keyword is matched, the waypoint is treated as GNSS (default behavior).

### Example Waypoint Names
- `GNSS_waypoint_1` - Navigation waypoint, proceeds to next automatically
- `Aruco_marker_detection` - Triggers ArUco detection subsystem
- `Mallet_pickup_location` - Triggers mallet handling subsystem
- `Bottle_drop_zone` - Triggers bottle handling subsystem
- `Rock Pick_site_A` - Triggers rock picking subsystem

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/waypoints` | `nav_msgs/Path` | List of waypoints to navigate |
| `/waypoint_reached` | `std_msgs/Bool` | Confirmation from global planner |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Current waypoint goal for navigation |
| `/current_waypoint_name` | `std_msgs/String` | Name of the current waypoint |
| `/mission_status` | `std_msgs/String` | Mission state (IDLE, NAVIGATING, COMPLETE) |

### Task Subsystem Topics (Published by Global Planner)

| Topic | Type | Description |
|-------|------|-------------|
| `/aruco` | `std_msgs/String` | Continue signal for ArUco subsystem |
| `/mallet` | `std_msgs/String` | Continue signal for mallet subsystem |
| `/bottle` | `std_msgs/String` | Continue signal for bottle subsystem |
| `/rock_pick` | `std_msgs/String` | Continue signal for rock pick subsystem |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal_timeout` | float | 300.0 | Timeout in seconds per waypoint |
| `use_sim_time` | bool | false | Use simulation time |

## Usage

### Launch the Mission Manager

```bash
ros2 launch mission_manager mission_manager.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch mission_manager mission_manager.launch.py goal_timeout:=600.0
```

### Run Node Directly

```bash
ros2 run mission_manager mission_manager_node
```

### Send Waypoints via Command Line

```bash
ros2 topic pub --once /waypoints nav_msgs/Path "{
  header: {frame_id: 'map'},
  poses: [
    {header: {frame_id: 'GNSS_start'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'Aruco_detect'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'GNSS_end'}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  ]
}"
```

## Mission State Machine

```
     +-------+
     | IDLE  |<--------------------------+
     +-------+                           |
         |                               |
         | /waypoints received           |
         v                               |
   +-----------+                         |
   | NAVIGATING|                         |
   +-----------+                         |
         |                               |
         | All waypoints completed       |
         v                               |
   +----------+                          |
   | COMPLETE |---> Reset ---------------+
   +----------+
```

## Integration with Navigation Stack

The Mission Manager works in conjunction with the navigation stack:

1. **Mission Manager** receives waypoints and manages sequential execution
2. **Global Planner** (`costmap_global_planner_node`) navigates to each waypoint
3. On goal reach, **Global Planner** routes to appropriate subsystem based on waypoint name
4. For GNSS waypoints, **Global Planner** sends confirmation back to **Mission Manager**
5. **Mission Manager** proceeds to next waypoint or completes mission

### Complete Launch Sequence

```bash
# Terminal 1: Unity Simulation Bridge
ros2 launch mt_unity_sim unity_bringup.launch.py

# Terminal 2: Costmap Generation
ros2 launch nav2_costmap_node costmap.launch.py

# Terminal 3: Navigation Stack
ros2 launch nav_stack costmap_nav_stack.launch.py

# Terminal 4: Mission Manager
ros2 launch mission_manager mission_manager.launch.py

# Terminal 5: Send waypoints
ros2 topic pub --once /waypoints nav_msgs/Path "..."
```

## Troubleshooting

### Mission Not Starting
- Verify `/waypoints` message is being received: `ros2 topic echo /waypoints`
- Check that waypoints list is not empty
- Ensure global planner is running and responding to `/goal_pose`

### Waypoints Not Advancing
- Check `/waypoint_reached` topic: `ros2 topic echo /waypoint_reached`
- Verify global planner is publishing confirmation on goal reach
- Check for navigation failures in global planner logs

### Wrong Subsystem Triggered
- Verify waypoint name contains correct keyword
- Keywords are case-sensitive: `Aruco` not `aruco`
- Check for typos in waypoint names

### Mission Stuck
- Check mission status: `ros2 topic echo /mission_status`
- Monitor current waypoint: `ros2 topic echo /current_waypoint_name`
- Verify robot is making progress toward goal

## License

GPL-3.0
