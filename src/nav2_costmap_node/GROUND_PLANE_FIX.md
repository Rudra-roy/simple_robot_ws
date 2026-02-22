# Costmap Ground Plane Fix & Dynamic Tuning

## What Was Fixed

### 1. **Ground Plane Height Issue** ✅
**Problem:** Costmap was generating at an incorrect height because ground filtering used a hardcoded value (0.4m) that didn't match your setup.

**Solution:** 
- Changed ground filtering to use the dynamic `min_obstacle_z` parameter
- Costmap correctly uses **global frame (odom/map)** for height measurements
- Default changed from 0.4m to 0.05m (5cm above ground plane)
- Now fully adjustable via GUI!

### 2. **Dynamic Parameter Control** ✅
**Problem:** No way to tune parameters without restarting the node - very painful for debugging!

**Solution:**
- Added full dynamic reconfigure support
- Created GUI launcher for easy parameter adjustment
- All critical parameters now tunable in real-time

## How to Use

### Quick Start (with GUI)
```bash
cd ~/simple_robot_ws
source install/setup.bash

# Option 1: Use the launch file
ros2 launch nav2_costmap_node costmap_with_gui.launch.py

# Option 2: Use the helper script
./src/nav2_costmap_node/launch_costmap_gui.sh
```

The rqt_reconfigure GUI will open automatically. Select `/costmap_node` and adjust parameters!

### Manual Launch (no GUI)
```bash
# Terminal 1: Launch costmap
ros2 launch nav2_costmap_node costmap.launch.py

# Terminal 2: Open GUI
ros2 run rqt_reconfigure rqt_reconfigure
```

## Dynamic Parameters Available

| Parameter | Default | Description | When to Adjust |
|-----------|---------|-------------|----------------|
| **min_obstacle_z** | 0.05m | Ground filter (global frame) | Ground marked as obstacles |
| **max_obstacle_z** | 4.0m | Ceiling filter | High objects ignored |
| **inflation_radius** | 0.3m | Safety buffer around obstacles | Navigation too conservative/risky |
| **cost_scaling_factor** | 10.0 | Cost gradient steepness | Fine-tune planner behavior |
| **publish_frequency** | 15.0 Hz | Costmap update rate | Performance issues |
| **obstacle_max_range** | 10.0m | Maximum sensor range | Limit detection distance |
| **obstacle_min_range** | 0.1m | Minimum sensor range | Filter noise near robot |
| **resolution** | 0.05m | Map cell size | Change grid granularity |

## Debugging Workflow

### Issue: Ground shows as obstacles in costmap

**Quick Fix:**
1. Launch with GUI: `ros2 launch nav2_costmap_node costmap_with_gui.launch.py`
2. In rqt_reconfigure, select `/costmap_node`
3. Increase `min_obstacle_z` slider (try 0.1, 0.2, 0.3...)
4. Watch RViz - ground obstacles should disappear!

### Issue: Real obstacles not detected

**Quick Fix:**
1. In rqt_reconfigure:
   - Check `min_obstacle_z` - might be too high
   - Check `max_obstacle_z` - might be too low
   - Check `obstacle_max_range` - might be too small
2. Adjust values and watch RViz in real-time

### Issue: Costmap too conservative (robot won't go through gaps)

**Quick Fix:**
1. Decrease `inflation_radius` (try 0.2, 0.15, 0.1)
2. Adjust `cost_scaling_factor` (lower = gentler gradient)

### Issue: Performance/lag

**Quick Fix:**
1. Decrease `publish_frequency` (try 10Hz or 5Hz)
2. Reduce `obstacle_max_range`
3. Increase `resolution` (e.g., 0.1m instead of 0.05m)

## Technical Details

### Frame Configuration
- **Costmap Frame:** `odom` (global_frame_id)
- **Robot Frame:** `zed_camera_link` (base_frame_id)
- **Height Filtering:** Applied in **global frame** (odom), not robot frame
  - This ensures ground plane is at consistent Z regardless of robot tilt
  - If ground is at Z=0 in odom, use `min_obstacle_z: 0.05`

### What Changed in Code
1. **costmap_node.cpp:**
   - Added `rcl_interfaces` for parameter callbacks
   - Implemented `parameters_callback()` method
   - Removed hardcoded `0.4` value, now uses `min_obstacle_z_` parameter
   - Added parameter validation and logging

2. **Launch files:**
   - Updated `costmap.launch.py` with all required parameters
   - Created `costmap_with_gui.launch.py` that auto-launches GUI
   - Added better defaults (min_obstacle_z: 0.05m instead of 0.40m)

3. **Config files:**
   - Updated `costmap.yaml` with explanatory comments
   - Changed defaults to more reasonable values

## Command Line Usage

```bash
# Get current parameter value
ros2 param get /costmap_node min_obstacle_z

# Set parameter value
ros2 param set /costmap_node min_obstacle_z 0.15

# List all parameters
ros2 param list /costmap_node

# Dump all parameters to file
ros2 param dump /costmap_node
```

## Files Created/Modified

### Modified:
- `src/costmap_node.cpp` - Added dynamic reconfigure support
- `config/costmap.yaml` - Updated defaults and comments
- `launch/costmap.launch.py` - Added all parameters with better defaults

### Created:
- `launch/costmap_with_gui.launch.py` - Easy GUI launcher
- `launch_costmap_gui.sh` - Helper script
- `README_DYNAMIC_PARAMS.md` - Detailed parameter guide
- `GROUND_PLANE_FIX.md` - This file

## Verification

After launching, you should see:
```
[costmap_node]: Simple costmap node up. Frame=odom base=zed_camera_link res=0.050 size=500x500
[costmap_node]: Dynamic reconfigure enabled. Use 'ros2 run rqt_reconfigure rqt_reconfigure' to adjust parameters.
```

When you change a parameter in the GUI, you'll see:
```
[costmap_node]: Updated min_obstacle_z (ground filter): 0.15
```

## Next Steps

1. **Test with your robot:**
   ```bash
   ros2 launch nav2_costmap_node costmap_with_gui.launch.py
   ```

2. **Visualize in RViz:**
   - Add `/costmap` (OccupancyGrid)
   - Add `/zed/zed_node/point_cloud/cloud_registered` (PointCloud2)
   - Compare ground filtering before/after adjusting `min_obstacle_z`

3. **Find optimal parameters:**
   - Adjust `min_obstacle_z` until ground is correctly filtered
   - Tune `inflation_radius` for your robot size
   - Adjust `publish_frequency` for performance

4. **Save optimal settings:**
   ```bash
   ros2 param dump /costmap_node > my_costmap_params.yaml
   ```

## Troubleshooting

### GUI doesn't open
```bash
# Install rqt_reconfigure if missing
sudo apt install ros-humble-rqt-reconfigure
```

### Parameters not changing
- Check node is running: `ros2 node list`
- Check parameters exist: `ros2 param list /costmap_node`
- Try command line: `ros2 param set /costmap_node min_obstacle_z 0.1`

### Still seeing ground as obstacles
- Increase `min_obstacle_z` more (try 0.3, 0.4, 0.5)
- Check your TF tree: ground might not be at Z=0 in odom frame
- Use `ros2 topic echo /tf` to see actual transforms

## Summary

✅ **Ground plane issue fixed** - Now uses dynamic parameter in global frame  
✅ **GUI control added** - Adjust all parameters live without restart  
✅ **Better defaults** - min_obstacle_z changed from 0.4m to 0.05m  
✅ **Easy debugging** - Real-time parameter tuning via rqt_reconfigure  
✅ **Full documentation** - Multiple guides and examples provided  

You can now tune your costmap in real-time while watching the results in RViz!
