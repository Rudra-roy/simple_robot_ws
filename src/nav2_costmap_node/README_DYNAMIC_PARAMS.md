# Dynamic Parameter Tuning for Costmap Node

## Overview
The costmap node now supports **live parameter adjustment** without restarting! This is extremely helpful for debugging and tuning.

## Available Dynamic Parameters

### Ground/Height Filtering (CRITICAL!)
- **`min_obstacle_z`** (default: 0.05m)
  - Minimum Z height in **global frame (odom/map)** to consider as obstacle
  - Points below this are filtered as "ground"
  - **Tune this if ground is being marked as obstacles!**
  - If your robot is elevated, increase this value
  - If ground is at different heights, adjust accordingly

- **`max_obstacle_z`** (default: 4.0m)
  - Maximum Z height to consider as obstacle
  - Filters ceiling/high objects

### Inflation Parameters
- **`inflation_radius`** (default: 0.3m)
  - How far to inflate obstacles (robot safety buffer)
  - Larger = more conservative, more space around obstacles

- **`cost_scaling_factor`** (default: 10.0)
  - How quickly cost decreases from obstacle center
  - Higher = steeper cost gradient

### Range Filtering
- **`obstacle_min_range`** (default: 0.1m)
  - Minimum sensor range (filters points too close)

- **`obstacle_max_range`** (default: 10.0m)
  - Maximum sensor range (filters distant points)

### Performance
- **`publish_frequency`** (default: 15.0 Hz)
  - How often to publish the costmap
  - Lower = less CPU/bandwidth, higher = more responsive

- **`resolution`** (default: 0.05m/cell)
  - Map resolution (requires restart for full effect)

## How to Use

### Method 1: Launch with GUI (Easiest!)
```bash
cd ~/simple_robot_ws
ros2 launch nav2_costmap_node costmap_with_gui.launch.py
```
This will automatically open the rqt_reconfigure GUI.

### Method 2: Manual Launch
```bash
# Terminal 1: Launch costmap
ros2 launch nav2_costmap_node costmap.launch.py

# Terminal 2: Open GUI
ros2 run rqt_reconfigure rqt_reconfigure
```

### Method 3: Command Line
```bash
# Set a parameter
ros2 param set /costmap_node min_obstacle_z 0.1

# Get current value
ros2 param get /costmap_node min_obstacle_z

# List all parameters
ros2 param list /costmap_node
```

## Debugging Workflow

### Problem: Ground is being marked as obstacles
**Solution:** Increase `min_obstacle_z`
1. Open rqt_reconfigure
2. Select `/costmap_node`
3. Adjust `min_obstacle_z` slider up (try 0.1, 0.2, 0.3...)
4. Watch costmap in RViz - ground obstacles should disappear

### Problem: Real obstacles not detected
**Solution:** Decrease `min_obstacle_z` or increase `max_obstacle_z`
1. Check if obstacles are in the Z range
2. Adjust `min_obstacle_z` down or `max_obstacle_z` up

### Problem: Costmap too conservative/aggressive
**Solution:** Adjust inflation
1. Decrease `inflation_radius` for tighter navigation
2. Increase for more safety margin
3. Adjust `cost_scaling_factor` for gradient steepness

### Problem: Performance issues
**Solution:** Reduce `publish_frequency`
1. Try 10Hz or 5Hz
2. Monitor CPU usage

## Frame Configuration

The costmap is generated in the **global_frame_id** (odom or map). This means:
- Ground plane filtering (`min_obstacle_z`) is relative to the **global frame origin**
- If ground is at Z=0 in odom frame, use `min_obstacle_z: 0.05`
- If your robot starts at height H, ground might be at Z=-H, so adjust accordingly

## Tips

1. **Always tune with live data** - Run your sensor and watch in RViz
2. **Use rqt_reconfigure GUI** - Much easier than command line
3. **Start with ground filtering** - Get `min_obstacle_z` right first
4. **Then tune inflation** - Adjust safety margins
5. **Monitor performance** - Check CPU usage and adjust `publish_frequency`

## Verification

Check current parameters:
```bash
ros2 param dump /costmap_node
```

Watch parameter changes in real-time:
```bash
ros2 param get /costmap_node min_obstacle_z
```

## See Also
- `RVIZ_SETUP.md` - How to visualize costmap in RViz
- `RAY_TRACING_EXPLAINED.md` - How costmap processes point clouds
