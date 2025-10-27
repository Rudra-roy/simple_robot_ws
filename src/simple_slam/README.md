# Simple SLAM - Point Cloud Mapping

This package provides mapping solutions using 3D point clouds with external odometry from the rosbot.

## Mapping Modes

### 1. RTAB-Map Point Cloud Mapping (Recommended)
Uses RTAB-Map with 3D point clouds to create a 2D occupancy grid map for navigation.

**Launch:**
```bash
ros2 launch simple_slam pointcloud_mapping.launch.py
```

**Features:**
- Uses external odometry from rosbot (`/odom` topic)
- Processes 3D point clouds from RealSense
- Generates 2D occupancy grid map on `/map` topic
- Includes loop closure detection
- Visualizable in RViz

### 2. Mapping Only (No Visualization)
Lightweight mapping without visualization GUI.

**Launch:**
```bash
ros2 launch simple_slam mapping_only.launch.py \
  odom_topic:=/odom \
  scan_cloud_topic:=/camera/camera/depth/color/points
```

### 3. Octomap Alternative
Uses Octomap for 3D voxel-based mapping with 2D projection.

**Launch:**
```bash
ros2 launch simple_slam octomap_mapping.launch.py
```

**Note:** Requires `octomap_server` package:
```bash
sudo apt install ros-${ROS_DISTRO}-octomap-server
```

## Topics

### Subscribed Topics:
- `/odom` or `/odometry/filtered` - Odometry from rosbot wheels/IMU
- `/rosbot/camera_depth/point_cloud` - Point cloud from RealSense D435i

### Published Topics:
- `/map` - 2D occupancy grid (nav_msgs/OccupancyGrid)
- `/rtabmap/grid_map` - Grid map with metadata
- `/rtabmap/cloud_map` - 3D point cloud map
- `/rtabmap/mapData` - RTAB-Map graph data

## Configuration

### Key Parameters (rtabmap_mapping.yaml):

**Grid Resolution:**
```yaml
Grid/CellSize: '0.05'  # 5cm cells
```

**Obstacle Heights:**
```yaml
Grid/MaxObstacleHeight: '2.0'  # Objects up to 2m considered obstacles
Grid/MaxGroundHeight: '0.1'    # Ground plane threshold
```

**Detection Rate:**
```yaml
Rtabmap/DetectionRate: '2.0'  # Process at 2 Hz (faster mapping)
```

**Mapping Range:**
```yaml
Grid/RangeMax: '15.0'  # Map up to 15 meters distance
```

## RViz Visualization

Add these displays in RViz:
1. **Map** - Topic: `/map`, Type: Map
2. **Point Cloud** - Topic: `/rtabmap/cloud_map`, Type: PointCloud2
3. **TF** - Show robot frames
4. **Robot Model** - If URDF available

## Frame Structure

```
map
└── odom (from rosbot odometry)
    └── base_link (robot base)
        └── camera_depth (RealSense point cloud frame)
```

## Troubleshooting

**No map published:**
- Verify odometry is published on `/odom`
- Check point cloud on `/camera/camera/depth/color/points`
- Ensure camera TF is correct

**Map quality issues:**
- Adjust `Grid/CellSize` for resolution
- Tune `Grid/MaxObstacleHeight` for your environment
- Check `ICP/VoxelSize` for point cloud matching

**Performance:**
- Adjust `Rtabmap/DetectionRate` for faster/slower processing
- Increase `Grid/RangeMax` to map larger areas (current: 15m)
- Lower `RGBD/LinearUpdate` for more detailed maps (current: 5cm updates)
- Use `mapping_only.launch.py` without visualization for best performance

## Integration with Nav2

The generated `/map` topic is compatible with Nav2's navigation stack. Use it as the global costmap source:

```yaml
# nav2 params
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: true
    map_topic: /map
```
