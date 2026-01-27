# Simple Robot Workspace

A ROS 2 workspace for autonomous navigation with Unity simulation support using costmap-based reactive navigation.

## Overview

This workspace provides a costmap-only navigation stack for mobile robots operating in Unity simulation environments. The system operates without requiring pre-built maps, making it ideal for unknown or dynamic environments. It uses real-time costmap data for reactive obstacle avoidance and path planning.

## Active Packages

- **mt_unity_sim** - Unity-ROS2 TCP communication bridge and robot state publisher
- **nav2_costmap_node** - Layered costmap generator with ray tracing for obstacle detection
- **nav_stack** - Costmap-based reactive navigation planners (global and local)
- **mission_manager** - Sequential waypoint navigation with task-specific routing
- **uirover_description** - Robot URDF/xacro description files

## Prerequisites

- ROS 2 (Humble/Iron recommended)
- Unity Hub with Unity Editor (2021.3 or later)
- Python 3.8+
- Required ROS 2 packages:
  - `geometry_msgs`
  - `nav_msgs`
  - `sensor_msgs`
  - `tf2_ros`

## Getting Started with Unity Simulation

### Step 1: Launch Unity Simulation

1. Open **Unity Hub**
2. Select and launch the project named **"Rover_sim"**
3. Once the Unity Editor opens, press the **Play** button in the Scene view to start the simulation

### Step 2: Build the ROS 2 Workspace

Open a terminal and navigate to the workspace directory:

```bash
cd ~/simple_robot_ws
colcon build
```

Wait for the build process to complete successfully.

### Step 3: Source the Workspace

After building, source the workspace to overlay the packages:

```bash
source install/setup.sh
```

### Step 4: Launch Unity-ROS2 Communication

Start the Unity simulation bridge to establish communication between Unity and ROS 2:

```bash
ros2 launch mt_unity_sim unity_bringup.launch.py
```

This launch file will:
- Start the ROS-TCP endpoint on `0.0.0.0:10000`
- Publish robot state and TF transforms
- Transform point cloud data from Unity format to ROS standard format

**Troubleshooting:** If you encounter connection issues with the local address `0.0.0.0:10000`:
- The port may be blocked by another process
- Identify and kill any process using port 10000:
  ```bash
  sudo lsof -i :10000
  sudo kill -9 <PID>
  ```
- Re-run the launch command

### Step 5: Verify Topic Communication

After a successful launch, verify that all topics are being published:

```bash
ros2 topic list
```

You should see topics related to sensor data, odometry, and control commands. Monitor specific topics to ensure data flow:

```bash
ros2 topic echo <topic_name>
```

### Step 6: Launch Navigation Stack

Once communication is established and verified, launch the navigation components:

**Launch Costmap Generation:**
```bash
ros2 launch nav2_costmap_node costmap.launch.py
```

This generates a layered costmap with:
- Ray-traced obstacle detection
- Inflation layers for safety margins
- Real-time updates from point cloud sensor data

**Launch Navigation Stack:**
```bash
ros2 launch nav_stack costmap_nav_stack.launch.py
```

This starts the reactive navigation system that:
- Plans paths using only costmap data (no pre-built map required)
- Performs real-time obstacle avoidance
- Executes tangential maneuvering around obstacles

**Optional - Launch with RViz Visualization:**
```bash
ros2 launch nav_stack costmap_nav_stack_with_rviz.launch.py
```

## Building

Build the workspace:

```bash
cd ~/simple_robot_ws
colcon build
source install/setup.sh
```

## Navigation System Features

### Costmap-Only Navigation
This workspace uses a **costmap-only** navigation approach, which means:
- ✅ No pre-built map required
- ✅ Works in completely unknown environments
- ✅ Handles dynamic obstacles in real-time
- ✅ Reactive local planning with obstacle avoidance
- ✅ Uses rolling costmap window for path planning

### Available Navigation Modes

**1. Standard Costmap Navigation** (`costmap_nav_stack.launch.py`)
- Direct path planning using costmap data
- Tangential obstacle avoidance
- Suitable for general navigation tasks

**2. Incremental Navigation** (`incremental_nav_stack.launch.py`)
- 20-degree rotation increments for smoother turns
- Better for environments with tight spaces
- More deliberate path execution

**3. Map-Based Navigation** (`nav_stack.launch.py`) - Legacy
- Requires pre-built map
- Traditional A* global planning
- Use only if you have a static map available

## Sending Navigation Goals

After launching the navigation stack, send goals using:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Or use RViz's **2D Nav Goal** tool when running with visualization.

## Mission-Based Navigation

For sequential waypoint missions with task-specific actions, use the Mission Manager:

### Launch Mission Manager

```bash
ros2 launch mission_manager mission_manager.launch.py
```

### Send Waypoint Mission

```bash
ros2 topic pub --once /waypoints nav_msgs/Path "{
  header: {frame_id: 'map'},
  poses: [
    {header: {frame_id: 'GNSS_start'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'Aruco_marker'}, pose: {position: {x: 5.0, y: 2.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'Mallet_pickup'}, pose: {position: {x: 8.0, y: 3.0}, orientation: {w: 1.0}}},
    {header: {frame_id: 'GNSS_end'}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}
  ]
}"
```

### Waypoint Types

| Keyword | Action on Goal Reach |
|---------|---------------------|
| `GNSS` | Proceed to next waypoint |
| `Aruco` | Signal ArUco detection via `/aruco` |
| `Mallet` | Signal mallet handling via `/mallet` |
| `Bottle` | Signal bottle handling via `/bottle` |
| `Rock Pick` | Signal rock picking via `/rock_pick` |

## Usage

### Complete Unity Simulation Workflow

Follow the **Getting Started with Unity Simulation** section above for the complete step-by-step workflow.

### Quick Reference - Launch Sequence

1. **Unity**: Start "Rover_sim" project and press Play
2. **ROS Bridge**: `ros2 launch mt_unity_sim unity_bringup.launch.py`
3. **Verify Topics**: `ros2 topic list`
4. **Costmap**: `ros2 launch nav2_costmap_node costmap.launch.py`
5. **Navigation**: `ros2 launch nav_stack costmap_nav_stack.launch.py`
6. **Mission Manager** (optional): `ros2 launch mission_manager mission_manager.launch.py`

### Alternative Launch Configurations

**With RViz Visualization:**
```bash
ros2 launch nav_stack costmap_nav_stack_with_rviz.launch.py
```

**Incremental Navigation (Smoother Turns):**
```bash
ros2 launch nav_stack incremental_nav_stack.launch.py
```

**With Simulation Time:**
```bash
ros2 launch nav_stack costmap_nav_stack.launch.py use_sim_time:=true
```

## Monitoring and Debugging

### Check Active Topics
```bash
ros2 topic list
```

Expected topics include:
- `/cmd_vel` - Velocity commands to robot
- `/odom` - Odometry data
- `/costmap` - Generated costmap
- `/camera/points` - Point cloud data
- `/tf` and `/tf_static` - Transform data

### Monitor Specific Topics
```bash
# Check velocity commands
ros2 topic echo /cmd_vel

# View costmap updates
ros2 topic echo /costmap

# Monitor odometry
ros2 topic echo /odom
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
```

## Configuration

Configuration files are located in each package's `config/` directory:

- **nav_stack/config/**
  - `costmap_planner_params.yaml` - Navigation planner parameters
  - `global_planner_params.yaml` - Global planner settings (legacy)
  - `*.rviz` - RViz visualization configurations

- **nav2_costmap_node/config/**
  - Costmap layer configurations
  - Ray tracing parameters
  - Inflation settings

## Package Details

### mt_unity_sim
Handles Unity-ROS2 communication via TCP endpoint. Publishes robot transforms and converts Unity point cloud format to ROS standard.

**Key Features:**
- TCP endpoint on port 10000
- Robot state publisher from URDF
- Point cloud coordinate transformation (YZX to XYZ)

### nav2_costmap_node
Generates multi-layered costmaps using ray tracing algorithm for accurate obstacle representation.

**Key Features:**
- Bresenham ray tracing for free space marking
- Layered architecture (obstacle, inflation, combined)
- Real-time updates from sensor data

### nav_stack
Reactive navigation planners that work without pre-built maps.

**Key Features:**
- Costmap-only global planning
- Tangential local obstacle avoidance
- Multiple planning modes (standard, incremental)
- Real-time path adjustment
- Waypoint type routing for task coordination

### mission_manager
Sequential waypoint mission coordination.

**Key Features:**
- Receives waypoint lists via `/waypoints` topic
- Sequential waypoint execution
- Task-specific routing based on waypoint names
- Coordinates with global planner for mission completion

## Troubleshooting

### Robot Not Moving
- Verify `/costmap` is being published: `ros2 topic echo /costmap`
- Check odometry: `ros2 topic echo /odom`
- Ensure goal is within costmap bounds
- Check for TF transform errors

### Unity Connection Failed
- Verify Unity is running and in Play mode
- Check if port 10000 is available
- Kill conflicting processes: `sudo lsof -i :10000`
- Restart `unity_bringup.launch.py`

### No Costmap Data
- Confirm point cloud is being published: `ros2 topic list | grep points`
- Check sensor data in Unity simulation
- Verify camera transforms in TF tree
- Review nav2_costmap_node logs for errors

### Navigation Stuck or Erratic
- Increase costmap size in configuration
- Adjust inflation radius
- Tune tangential scan distance
- Check for obstacles blocking all paths

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    UNITY SIMULATION                      │
│                  (Rover_sim Project)                     │
└────────────────────┬────────────────────────────────────┘
                     │ TCP (port 10000)
                     ▼
┌─────────────────────────────────────────────────────────┐
│              mt_unity_sim                                │
│  • ROS-TCP Endpoint                                      │
│  • Robot State Publisher                                 │
│  • Point Cloud Transformer                               │
└────────────────────┬────────────────────────────────────┘
                     │ ROS Topics
          ┌──────────┴───────────┐
          ▼                      ▼
┌─────────────────────┐  ┌─────────────────────┐
│  nav2_costmap_node  │  │    /odom, /tf       │
│  • Ray Tracing      │  │   Transform Data    │
│  • Layer Fusion     │  └─────────────────────┘
│  • Inflation        │
└──────────┬──────────┘
           │ /costmap
           ▼
┌─────────────────────────────────────────────────────────┐
│              mission_manager                             │
│  • Receives /waypoints (nav_msgs/Path)                   │
│  • Sequential waypoint execution                         │
│  • Task routing based on waypoint names                  │
└────────────────────┬────────────────────────────────────┘
                     │ /goal_pose
                     ▼
┌─────────────────────────────────────────────────────────┐
│              nav_stack                                   │
│  • Costmap Global Planner (path planning)                │
│  • Costmap Local Planner (obstacle avoidance)            │
│  • Waypoint type routing on goal reach                   │
└────────────────────┬────────────────────────────────────┘
        ┌────────────┼────────────────────────┐
        │            │                        │
        ▼            ▼                        ▼
   /cmd_vel    /waypoint_reached      /aruco, /mallet,
  [Robot]     [Mission Manager]       /bottle, /rock_pick
                                      [Task Subsystems]
```

## License

GPL-3

## Additional Resources

For detailed information about specific packages, refer to:
- [mt_unity_sim/README.md](src/mt_unity_sim/README.md) - Unity integration details
- [nav2_costmap_node/README.md](src/nav2_costmap_node/README.md) - Costmap architecture and ray tracing
- [nav_stack/COSTMAP_ONLY_README.md](src/nav_stack/COSTMAP_ONLY_README.md) - Costmap-only navigation details
- [mission_manager/README.md](src/mission_manager/README.md) - Sequential waypoint mission management

## Contributing

When contributing to this workspace:
1. Ensure all builds complete without errors: `colcon build`
2. Test with Unity simulation before committing
3. Update relevant documentation
4. Follow ROS 2 coding standards

## Maintainer

**Hironmoy Roy Rudra**  
*AI & Autonomous Sub-Team*
