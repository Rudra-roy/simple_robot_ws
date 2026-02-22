# Autonomous Navigation System - Package Reorganization

## Overview

The monolithic `mt10_control` package has been reorganized into four specialized packages for improved modularity, maintainability, and clarity:

1. **detection_interfaces** - Custom ROS2 message definitions
2. **aruco_detection** - ArUco marker detection and tracking
3. **object_detection** - YOLO-based object detection (mallet, bottle)
4. **mission_manager** - Mission coordination and waypoint management

## Package Structure

```
src/
├── detection_interfaces/     # Message definitions (CMake)
│   ├── msg/
│   │   ├── DetectionResult.msg
│   │   ├── MissionState.msg
│   │   └── TaskCommand.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── aruco_detection/          # ArUco marker detection
│   ├── aruco_detection/
│   │   ├── aruco_detector_node.py
│   │   └── __init__.py
│   ├── config/
│   │   └── aruco_params.yaml
│   ├── launch/
│   │   └── aruco_detection.launch.py
│   ├── package.xml
│   └── setup.py
│
├── object_detection/         # YOLO object detection
│   ├── object_detection/
│   │   ├── yolo_detector_base.py
│   │   ├── mallet_detector_node.py
│   │   ├── bottle_detector_node.py
│   │   └── __init__.py
│   ├── config/
│   │   ├── mallet_params.yaml
│   │   └── bottle_params.yaml
│   ├── launch/
│   │   ├── mallet_detection.launch.py
│   │   ├── bottle_detection.launch.py
│   │   └── all_object_detection.launch.py
│   ├── package.xml
│   └── setup.py
│
└── mission_manager/          # Mission coordination
    ├── mission_manager/
    │   ├── mission_coordinator_node.py
    │   ├── pattern_generator_node.py
    │   └── __init__.py
    ├── config/
    │   └── mission_config.yaml
    ├── missions/
    │   ├── simple_mission.yaml
    │   ├── full_mission.yaml
    │   └── test_mission.yaml
    ├── launch/
    │   ├── mission_manager.launch.py
    │   └── full_system.launch.py
    ├── package.xml
    └── setup.py
```

## Communication Architecture

### Topic Structure

```
/mission/
  ├── task_command        # TaskCommand - Mission → Detection nodes
  ├── state               # MissionState - Mission status
  └── start/abort         # Bool - Mission control

/detection/
  ├── aruco/
  │   ├── result          # DetectionResult
  │   └── status          # String
  ├── mallet/
  │   ├── result          # DetectionResult
  │   └── status          # String
  └── bottle/
      ├── result          # DetectionResult
      └── status          # String

/waypoints                # String "lat,lon,type" → Nav stack
/nav_stack/mission_manager  # String "reached" ← Nav stack
/mt_led_indicator/rover   # String "red"|"green" - LED control
/cmd_vel                  # Twist - Robot commands (no arbitration)
/witmotion_eular/yaw      # Float64 - IMU heading
```

### Message Flow

```
Mission File (YAML)
   ↓
Mission Coordinator
   ├─→ /waypoints → Nav Stack (global planner)
   │                    ↓
   │              Navigates to goal
   │                    ↓
   ├←─ /mission_manager "reached"
   │
   ├─→ /mt_led_indicator/rover "red" (waypoint sent)
   ├─→ /mt_led_indicator/rover "green" (goal reached)
   │        [10 second delay]
   │
   ├─→ /mission/task_command → Detection Node
   │                              ↓
   │                         Searches/tracks
   │                              ↓
   └←─ /detection/*/result (success/failed)
         ↓
    Next waypoint
```

## Building the Packages

```bash
cd /home/demos/simple_robot_ws

# Build message interfaces first
colcon build --packages-select detection_interfaces

# Source to make messages available
source install/setup.bash

# Build all detection and mission packages
colcon build --packages-select aruco_detection object_detection mission_manager

# Source again
source install/setup.bash
```

## Usage

### 1. Launch Full System

Launch all detection nodes and mission manager:

```bash
ros2 launch mission_manager full_system.launch.py \
    mission_file:=/path/to/mission.yaml \
    auto_start:=true
```

### 2. Launch Individual Components

**ArUco Detection:**
```bash
ros2 launch aruco_detection aruco_detection.launch.py camera_device:=2
```

**Mallet Detection:**
```bash
ros2 launch object_detection mallet_detection.launch.py
```

**Bottle Detection:**
```bash
ros2 launch object_detection bottle_detection.launch.py
```

**All Object Detection:**
```bash
ros2 launch object_detection all_object_detection.launch.py
```

**Mission Manager Only:**
```bash
ros2 launch mission_manager mission_manager.launch.py \
    mission_file:=/path/to/mission.yaml
```

### 3. Run Individual Nodes

```bash
# ArUco detector
ros2 run aruco_detection aruco_detector_node

# Mallet detector
ros2 run object_detection mallet_detector_node

# Bottle detector
ros2 run object_detection bottle_detector_node

# Mission coordinator
ros2 run mission_manager mission_coordinator_node \
    --ros-args -p mission_file:=/path/to/mission.yaml
```

## Mission Definition Format

### Simple Format (Pure Navigation)

```yaml
waypoints:
  - lat: 40.712776
    lon: -74.005974
  
  - lat: 40.712800
    lon: -74.005900
```

### Structured Format (With Tasks)

```yaml
mission_name: "Campus Survey"

waypoints:
  # Pure navigation
  - lat: 40.712776
    lon: -74.005974
    task_type: null
    tolerance: 1.0
  
  # ArUco search after reaching
  - lat: 40.712800
    lon: -74.005900
    task_type: "aruco"
    tolerance: 1.5
  
  # Mallet detection
  - lat: 40.712850
    lon: -74.005850
    task_type: "mallet"
    tolerance: 2.0
  
  # Bottle detection
  - lat: 40.712900
    lon: -74.005800
    task_type: "bottle"
    tolerance: 2.0
```

**Task Types:**
- `null` or omitted: Pure navigation, no detection
- `"aruco"`: ArUco marker search and tracking
- `"mallet"`: Mallet object detection
- `"bottle"`: Bottle object detection

## Configuration

### ArUco Detection Parameters

Edit `aruco_detection/config/aruco_params.yaml`:

```yaml
camera_device: 2                # Camera index or /dev/v4l/by-id/...
marker_size: 0.15               # Marker size in meters
aruco_dict: "4x4_50"            # ArUco dictionary
linear_speed: 80.0              # Forward speed
angular_speed: 60.0             # Rotation speed
stop_distance: 1.5              # Stop distance (meters)
focal_length: 941               # Camera focal length (pixels)
websocket_port: 8000            # Video streaming port
```

### Object Detection Parameters

Edit `object_detection/config/mallet_params.yaml` or `bottle_params.yaml`:

```yaml
model_path: "/path/to/model.pt"
camera_device: 0
reference_height: 0.10          # Object height (meters)
focal_length: 941               # Camera focal length
linear_speed: 90.0
angular_speed: 60.0
stop_distance: 1.5
confidence_threshold: 0.65      # YOLO confidence
websocket_port: 8010            # Video streaming port
```

### Mission Manager Parameters

Edit `mission_manager/config/mission_config.yaml`:

```yaml
goal_reached_delay: 10.0        # Wait time after reaching goal (seconds)
task_timeout: 120.0             # Max task duration (seconds)
auto_start: false               # Auto-start on launch
```

## LED Indicator Behavior

The mission coordinator controls the LED via `/mt_led_indicator/rover`:

- **RED**: Waypoint sent, robot navigating
- **GREEN**: Goal reached, waiting 10 seconds before task or next waypoint

Detection nodes do NOT control the LED.

## Integration with Nav Stack

The system integrates with the existing GPS-based navigation stack:

1. Mission coordinator publishes waypoints → `/waypoints` (String "lat,lon,type")
2. Global planner (`costmap_global_planner_node`) navigates to GPS coordinates
3. On goal reached, global planner publishes → `/nav_stack/mission_manager` ("reached")
4. Mission coordinator receives signal, sets LED green, waits 10s
5. If waypoint has task, mission coordinator triggers detection node
6. On detection complete, mission coordinator sends next waypoint

**Note:** Detection nodes publish directly to `/cmd_vel` during their operation. The mission state machine ensures only one component controls the robot at a time (no cmd_vel arbitration needed).

## Migration from mt10_control

### Old vs New

| Old (mt10_control) | New Package | New Topic |
|--------------------|-------------|-----------|
| `/continue_search_ar` | aruco_detection | `/mission/task_command` |
| `/continue_search_mallet` | object_detection | `/mission/task_command` |
| `/continue_search_bottle` | object_detection | `/mission/task_command` |
| `/autonomous_status` | All detection | `/detection/*/result` |
| `/light_status` | mission_manager | `/mt_led_indicator/rover` |
| `/aruco_detection_status` | aruco_detection | `/detection/aruco/status` |
| `/mallet_detection_status` | object_detection | `/detection/mallet/status` |

### Deprecated Nodes

The following mt10_control nodes are replaced:

- `aruco_new` → `ros2 run aruco_detection aruco_detector_node`
- `mallet` → `ros2 run object_detection mallet_detector_node`
- `bottle` → `ros2 run object_detection bottle_detector_node`
- `auto_witmotion` → Functionality split between nav_stack and mission_manager
- `gps_txt` → `ros2 run mission_manager mission_coordinator_node` (with YAML files)
- `point_follower` → `ros2 run mission_manager pattern_generator_node`

## Troubleshooting

### Detection node can't find camera

Check camera device parameter:
```bash
ls /dev/v4l/by-id/
# Use full path in config file
```

### YOLO model not found

Update model_path in config file:
```bash
# For mallet
nano ~/simple_robot_ws/src/object_detection/config/mallet_params.yaml

# For bottle
nano ~/simple_robot_ws/src/object_detection/config/bottle_params.yaml
```

### Mission coordinator not starting

Check mission file path and format:
```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('missions/full_mission.yaml'))"
```

### LED not changing

Verify topic name:
```bash
ros2 topic echo /mt_led_indicator/rover
```

### Detection task not triggering

Check mission coordinator state:
```bash
ros2 topic echo /mission/state
```

Check task command is published:
```bash
ros2 topic echo /mission/task_command
```

## Testing

### Test ArUco Detection

```bash
# Terminal 1: Launch node
ros2 run aruco_detection aruco_detector_node

# Terminal 2: Send start command
ros2 topic pub --once /mission/task_command detection_interfaces/msg/TaskCommand \
    "{task_type: 'aruco', command: 'start'}"

# Terminal 3: Monitor results
ros2 topic echo /detection/aruco/result
```

### Test Mission Execution

```bash
# Launch with test mission
ros2 launch mission_manager mission_manager.launch.py \
    mission_file:=`ros2 pkg prefix mission_manager`/share/mission_manager/missions/test_mission.yaml \
    auto_start:=true

# Monitor state
ros2 topic echo /mission/state

# Monitor LED
ros2 topic echo /mt_led_indicator/rover
```

## Dependencies

**Python Packages:**
- `opencv-python` or `opencv-contrib-python` (for ArUco)
- `ultralytics` (for YOLO)
- `numpy`
- `pyyaml`
- `websockets` (for video streaming)

**ROS2 Packages:**
- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `sbg_driver` (for GPS)
- `nav_stack` (existing package)

## Future Enhancements

1. **Service-based mission loading**: Add ROS2 service to load missions dynamically
2. **Mission pause/resume**: Implement pause button via `/mission/pause`
3. **Progress visualization**: Create RViz plugin for mission progress
4. **Detection result logging**: Save detection results to database
5. **Pattern generator service**: Expose pattern generation as ROS2 service
6. **Multi-robot coordination**: Extend for fleet management

## Contact

For issues or questions, contact: mahir@todo.todo

## License

Apache-2.0
