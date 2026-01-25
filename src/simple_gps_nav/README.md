# simple_gps_nav

Point-to-point GPS navigation system for rover traversal using haversine distance calculations and bearing angles.

## Overview

This package provides a simple GPS-based navigation system that calculates the distance and bearing to a target GPS coordinate. It uses real-time GPS position data and IMU orientation to determine navigation commands.

## Nodes

### gps_navigation_node

Main algorithm node that processes GPS and IMU data to compute navigation commands.

**Subscribed Topics:**
- `/gps_fix` (sensor_msgs/NavSatFix): Current GPS position
- `/imu` (sensor_msgs/Imu): Current orientation from IMU
- `/gps_target` (geometry_msgs/PoseStamped): Target GPS coordinates (latitude in x, longitude in y)

**Published Topics:**
- None (logging only)

**Algorithm:**
1. Calculates distance to target using Haversine formula
2. Calculates bearing angle to target
3. Compares current heading with target bearing
4. Outputs navigation commands with explicit logging

**Logging Output:**
- Current and target GPS positions
- Distance to target in meters
- Current heading and target bearing in degrees
- Angle difference
- Direction command (TURN LEFT, TURN RIGHT, or ALIGNED)

### target_publisher_node

Publishes target GPS coordinates to the navigation node.

**Published Topics:**
- `/gps_target` (geometry_msgs/PoseStamped): Target GPS coordinates

**Parameters:**
- `target_latitude` (float, default: 0.0): Target latitude
- `target_longitude` (float, default: 0.0): Target longitude
- `publish_rate` (float, default: 1.0): Publishing rate in Hz

## Launch Files

### gps_nav.launch.py

Launches both the navigation and target publisher nodes.

**Launch Arguments:**
- `target_latitude` (default: 0.0): Target latitude coordinate
- `target_longitude` (default: 0.0): Target longitude coordinate

**Usage:**
```bash
ros2 launch simple_gps_nav gps_nav.launch.py target_latitude:=37.7749 target_longitude:=-122.4194
```

## Building

```bash
cd ~/simple_robot_ws
colcon build --packages-select simple_gps_nav
source install/setup.bash
```

## Running

Run with default target (0.0, 0.0):
```bash
ros2 launch simple_gps_nav gps_nav.launch.py
```

Run with specific target coordinates:
```bash
ros2 launch simple_gps_nav gps_nav.launch.py target_latitude:=37.7749 target_longitude:=-122.4194
```

Run nodes individually:
```bash
ros2 run simple_gps_nav gps_navigation_node
ros2 run simple_gps_nav target_publisher_node --ros-args -p target_latitude:=37.7749 -p target_longitude:=-122.4194
```

## Testing

Ensure you have GPS and IMU data being published to `/gps_fix` and `/imu` topics respectively. You can verify the topics:

```bash
ros2 topic list
ros2 topic echo /gps_fix
ros2 topic echo /imu
ros2 topic echo /gps_target
```

## Dependencies

- rclpy
- sensor_msgs
- geometry_msgs
- std_msgs

## Notes

- Distance calculations use the Haversine formula assuming Earth radius of 6371 km
- Bearing angles are calculated relative to true north (0-360 degrees)
- Navigation considers the rover aligned when angle difference is less than 5 degrees
- Target is considered reached when distance is less than 1 meter
