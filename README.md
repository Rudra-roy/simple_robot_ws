# Simple Robot Workspace

A ROS 2 workspace for autonomous navigation using a single RealSense D435i camera with SLAM and Nav2 integration.

## Overview

This workspace provides a complete navigation stack for mobile robots equipped with Intel RealSense D435i cameras. It integrates visual SLAM, point cloud processing, and autonomous navigation capabilities.

## Packages

- **simple_bringup** - Launch files for the complete navigation stack
- **simple_slam** - RTAB-Map based visual SLAM using RealSense D435i
- **simple_nav2** - Nav2 integration and configuration
- **simple_nav2_planner** - Custom navigation planner implementations
- **simple_navigation** - Waypoint navigation and adaptive planning
- **simple_clouds** - Point cloud processing and management
- **simple_mapper** - Mapping utilities
- **nav2_costmap_node** - Custom costmap generation

## Prerequisites

- ROS 2 (Humble/Iron recommended)
- Intel RealSense SDK 2.0
- RTAB-Map ROS 2
- Nav2
- Robot Localization

## Building

```bash
cd ~/simple_robot_ws
colcon build --symlink-install
source install/setup.zsh
```

## Usage

### Launch Full Navigation Stack

```bash
ros2 launch simple_bringup simple_nav.launch.py
```

### Launch Individual Components

**SLAM Only:**
```bash
ros2 launch simple_slam slam.launch.py
```

**Navigation Only:**
```bash
ros2 launch simple_nav2 nav2.launch.py
```

**Point Cloud Processing:**
```bash
ros2 launch simple_clouds clouds.launch.py
```

## Configuration

Configuration files are located in each package's `config/` directory. Key parameters include:
- SLAM settings in `simple_slam/config/`
- Nav2 parameters in `simple_nav2/config/`
- Sensor configurations in `simple_bringup/config/`

## License

MIT

## Maintainer

ROS User (user@example.com)
