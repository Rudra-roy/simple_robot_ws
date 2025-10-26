# Simple Navigation Stack for Single RealSense D435i

This is a simplified implementation of the Kalman robot navigation stack, adapted for a single RealSense D435i camera setup.

## Overview

The stack provides:
- **SLAM**: RTAB-Map visual SLAM with single camera
- **Localization**: EKF sensor fusion (visual odometry + IMU)
- **Obstacle Detection**: Point cloud processing with ground plane removal
- **Navigation**: Nav2 with custom path follower and STVL costmaps

## Frame Configuration

- Primary frame: `camera_link` (replaces `base_link`)
- TF tree: `map -> odom -> camera_link`
- All nodes configured to use `camera_link` as robot base frame

## Hardware Requirements

- Intel RealSense D435i camera
- Camera should be mounted facing forward on top of rover

## Usage

### Quick Start

```bash
# Launch the complete navigation stack
ros2 launch simple_bringup simple_nav.launch.py

# For simulation
ros2 launch simple_bringup simple_nav.launch.py use_sim_time:=true
```

### Individual Components

```bash
# Launch only SLAM
ros2 launch simple_slam slam.launch.py

# Launch only navigation
ros2 launch simple_nav2 nav2.launch.py

# Launch only point cloud processing
ros2 launch simple_clouds clouds.launch.py
```

## Key Topics

- `/camera/color/image_raw` - RGB image from camera
- `/camera/depth/image_rect_raw` - Depth image from camera
- `/camera/imu` - IMU data from camera
- `/camera/depth/color/points` - Point cloud from camera
- `/camera/obstacles` - Processed obstacle point cloud
- `/odometry/filtered` - Fused odometry from EKF
- `/map` - SLAM-generated occupancy grid
- `/cmd_vel` - Velocity commands for robot

## Configuration

Key configuration files:
- `simple_slam/config/` - SLAM and localization parameters
- `simple_nav2/config/` - Navigation and path following parameters
- `simple_clouds/config/` - Point cloud processing parameters

## Architecture

```
RealSense D435i → RGBD + IMU → Visual Odometry
                                      ↓
                               EKF Localization
                                      ↓
Point Cloud → Obstacle Detection → Nav2 Costmaps → Path Planning
                                                        ↓
                                              Custom Path Follower
```

The system uses:
1. **Visual odometry** from RTAB-Map for motion estimation
2. **EKF** for sensor fusion (visual odometry + IMU)
3. **RTAB-Map SLAM** for mapping and localization
4. **Custom obstacle detection** for point cloud processing
5. **Nav2** with STVL costmaps for navigation
6. **Pure pursuit path follower** for motion control

## Dependencies

- ROS2 Humble
- realsense2_camera
- rtabmap_ros
- robot_localization
- nav2_bringup
- spatio_temporal_voxel_layer
- pcl_ros
- tf2_ros

## Notes

- The camera_link frame serves as the robot's base frame
- No GPS integration (removed from original Kalman implementation)
- Optimized for single camera operation
- Includes rotate-in-place behavior for large heading errors
- Adaptive speed control based on path curvature
