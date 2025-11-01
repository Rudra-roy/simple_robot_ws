# RViz Configuration for Costmap Layers

To visualize the new layered costmap architecture, add these displays to your RViz config:

## Display Configuration

### 1. Combined Costmap (Main)
```yaml
Display Name: Costmap (Combined)
Type: Map
Topic: /costmap
Color Scheme: costmap
```

### 2. Obstacle Layer (Ray Tracing)
```yaml
Display Name: Obstacle Layer
Type: Map
Topic: /costmap/obstacle_layer
Color Scheme: costmap
Alpha: 0.7
```
This shows the ray-traced free space and obstacles from sensors.

### 3. Inflation Layer
```yaml
Display Name: Inflation Layer  
Type: Map
Topic: /costmap/inflation_layer
Color Scheme: costmap
Alpha: 0.5
```
This shows the safety margins around obstacles.

### 4. Static Layer (if using map)
```yaml
Display Name: Static Layer
Type: Map
Topic: /costmap/static_layer
Color Scheme: map
Alpha: 0.3
```
This shows permanent obstacles from the map.

## Quick RViz Setup

1. **Open RViz**
   ```bash
   rviz2
   ```

2. **Add Map displays** (4 times):
   - Click "Add" button
   - Select "Map"
   - Set topic to each layer

3. **Arrange in order** (bottom to top):
   - Static Layer (bottom, alpha=0.3)
   - Obstacle Layer (alpha=0.7)
   - Inflation Layer (alpha=0.5)
   - Combined Costmap (top, alpha=1.0)

4. **Enable/disable layers** to see their individual contributions

## What to Look For

### Obstacle Layer (`/costmap/obstacle_layer`)
- **Black cells**: Obstacles detected by sensors
- **White cells**: Free space (confirmed by ray tracing)
- **Gray cells**: Unknown (sensor hasn't reached)

**Key observation**: Space between robot and obstacles should be WHITE (ray traced), not gray!

### Inflation Layer (`/costmap/inflation_layer`)
- **Black center**: Lethal obstacle
- **Dark gray ring**: High cost (close to obstacle)
- **Light gray ring**: Low cost (near inflation radius)
- **White outside**: No inflation cost

**Key observation**: Should see smooth gradient from obstacles, like your second image!

### Combined Costmap (`/costmap`)
- Result of all layers combined
- Should look like Nav2 costmaps
- Should match your second image

## Comparison Test

Side-by-side comparison:
```
Your Image #2              Our Implementation
┌──────────────┐          ┌──────────────┐
│ ████  Obs    │          │ ████  Obs    │
│ ████  Core   │          │ ████  Core   │
│ ░░░░  Infl   │    ≈     │ ░░░░  Infl   │
│ ····  Free   │          │ ····  Free   │
└──────────────┘          └──────────────┘
Should match!
```

## Debugging Tips

### Issue: Obstacle interiors still gray
- **Check**: Is point cloud being received?
  ```bash
  ros2 topic hz /rosbot/camera_depth/point_cloud
  ```
- **Check**: Are transforms available?
  ```bash
  ros2 run tf2_tools view_frames
  ```
- **Check**: Is robot position valid?
  ```bash
  ros2 topic echo /tf --field transforms[0].transform
  ```

### Issue: No inflation visible
- **Check**: `inflation_radius` parameter
  ```bash
  ros2 param get /costmap inflation_radius
  ```
- **Increase** if too small: `inflation_radius: 0.5`

### Issue: Layers not published
- **Check**: Topics exist
  ```bash
  ros2 topic list | grep costmap
  ```
- **Should see**:
  - `/costmap`
  - `/costmap/obstacle_layer`
  - `/costmap/inflation_layer`
  - `/costmap/static_layer`

## Performance Monitoring

Check node performance:
```bash
ros2 topic hz /costmap
# Should be ~10 Hz (publish_frequency)

ros2 topic bw /costmap
# Should be reasonable (<1 MB/s)
```

## Tuning Parameters

### For More Conservative Navigation
```yaml
inflation_radius: 0.5          # Larger safety margin
cost_scaling_factor: 15.0      # Steeper gradient
```

### For More Aggressive Navigation
```yaml
inflation_radius: 0.2          # Smaller safety margin
cost_scaling_factor: 5.0       # Gentler gradient
```

### For Better Ray Tracing
```yaml
obstacle_max_range: 7.0        # See farther
obstacle_min_range: 0.05       # Catch close obstacles
```

## Expected Results

After implementing these changes:

1. ✅ **Obstacle interiors** no longer shown as unknown (white/gray)
   - Ray tracing marks space as free or inaccessible
   
2. ✅ **Layered visualization** matches your second image
   - Clear obstacle cores (black)
   - Inflation zones (gray gradients)
   - Free space (white)
   
3. ✅ **Efficient planning**
   - Planner doesn't explore inaccessible interiors
   - Smoother paths around obstacles
   - Faster planning times

4. ✅ **Debugging capability**
   - Individual layer inspection
   - Clear understanding of cost sources
   - Easy parameter tuning
