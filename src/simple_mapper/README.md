# Simple Mapper

A ROS 2 package that creates a **persistent/growing 2D occupancy grid map** from point cloud data and publishes the map→odom transform.

## Features

- **Persistent Map Building**: Accumulates observations over time (map grows as robot explores)
- **Probabilistic Occupancy**: Uses hit/miss counting for robust obstacle detection
- **Ray Tracing**: Marks free space between robot and obstacles
- **Height-based Filtering**: Distinguishes ground from obstacles
- **TF Publishing**: Publishes map→odom transform for global localization
- **RViz Visualization**: Real-time map visualization on `/map` topic

## Architecture Overview

This package is designed to work in a **two-layer mapping/navigation system**:

### 1. Global Map (this package)
- **Purpose**: Build and maintain a persistent global map from startup
- **Updates**: Accumulative - remembers all previously mapped areas
- **Frame**: `map` frame (global reference)
- **Usage**: Long-term planning, global path planning

### 2. Local Costmap (nav2_costmap_node)
- **Purpose**: Real-time obstacle detection for navigation
- **Updates**: Fast, real-time updates from sensors
- **Frame**: `odom` or `base_link` (local reference)
- **Usage**: Immediate obstacle avoidance, local planning

### Workflow
```
1. simple_mapper builds global map → publishes to /map
2. Local costmap detects obstacle in real-time
3. Navigation system gathers more data about obstacle
4. Planner creates trajectory using both global + local maps
```

## How It Works

### Map Building
1. **Point Cloud Input**: Receives 3D point cloud from depth camera
2. **Height Filtering**: Separates ground from obstacles based on height
3. **Ray Tracing**: For each point, marks cells from robot to point as free
4. **Hit/Miss Counting**: Tracks obstacle observations vs free space observations
5. **Probabilistic Update**: Calculates occupancy probability:
   - Free: < 40% hit rate → value 0
   - Occupied: > 60% hit rate → value 100
   - Uncertain: 40-60% hit rate → value 50
   - Unknown: No observations → value -1

### Persistence
Unlike simple projection, this creates a **persistent map**:
- Map cells remember past observations
- Previously seen areas stay in the map even when not currently visible
- More observations = higher confidence in occupancy

## Usage

### Launch the mapper

```bash
ros2 launch simple_mapper mapper.launch.py
```

### With custom point cloud topic

```bash
ros2 launch simple_mapper mapper.launch.py cloud_topic:=/your/cloud/topic
```

### View in RViz
Add a `Map` display with topic `/map` to see the growing global map.

## Parameters

- `cloud_topic`: Input point cloud topic (default: `/camera/depth/color/points`)
- `map_resolution`: Map resolution in meters per cell (default: 0.05 = 5cm)
- `map_width`: Map width in cells (default: 400 = 20m wide)
- `map_height`: Map height in cells (default: 400 = 20m tall)
- `min_height`: Minimum z height to consider (default: -0.5m)
- `max_height`: Maximum z height to consider (default: 2.0m)
- `obstacle_threshold`: Height above which points are obstacles (default: 0.3m)
- `update_rate`: Map update rate in Hz (default: 2.0)
- `publish_map`: Whether to publish the map (default: true)
- `map_frame`: Map frame ID (default: 'map')
- `odom_frame`: Odom frame ID (default: 'odom')
- `base_frame`: Base frame ID (default: 'base_link')

## Published Topics

- `/map` (nav_msgs/OccupancyGrid): The persistent 2D occupancy grid map

## Published TF

- `map` → `odom`: Transform from map to odometry frame (currently static identity)

## Subscribed Topics

- Point cloud topic (sensor_msgs/PointCloud2): 3D point cloud data

## Next Steps for Your Navigation System

1. **Run this mapper** to build global map
2. **Run local costmap** for real-time obstacle detection
3. **When obstacle detected**:
   - Stop or slow down
   - Rotate camera/robot to gather more point cloud data
   - Update both global and local maps
   - Replan trajectory with updated information
4. **Use Nav2 planner** with the global map for path planning
5. **Use local costmap** for immediate collision avoidance

## Limitations

- **No loop closure**: Doesn't correct for drift when revisiting areas
- **Static map→odom TF**: Assumes odometry is drift-free (for better localization, integrate with AMCL or SLAM Toolbox)
- **Fixed map size**: Map bounds are fixed at startup
- **Simple transforms**: Assumes minimal rotation (works well for ground robots)

For production systems with loop closure and drift correction, consider using SLAM Toolbox or Cartographer instead.
