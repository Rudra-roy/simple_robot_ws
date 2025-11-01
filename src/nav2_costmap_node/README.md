# Nav2 Costmap Node - Layered Costmap Architecture

## Overview

This node implements a layered costmap architecture similar to Nav2's costmap_2d, with proper ray tracing to handle obstacle interiors and multiple cost layers for robust navigation.

## Problem Solved

### Issue 1: Unknown Space Inside Obstacles
**Problem**: SLAM systems often mark the interior of obstacles as "unknown" (white/gray) because sensors only see the outer surface. This creates planning inefficiencies as the planner may attempt to explore inaccessible areas.

**Solution**: Implemented **ray tracing** using Bresenham's line algorithm:
- For each obstacle point detected by the sensor, trace a ray from the robot's position to the obstacle
- Mark all cells along the ray as **FREE_SPACE** (sensor beam passed through them)
- Mark only the endpoint as **LETHAL_OBSTACLE** (sensor hit something there)
- This ensures that space between the robot and obstacles is correctly marked as free
- Space beyond obstacles remains unknown (sensor didn't reach there)

### Issue 2: Missing Costmap Layers
**Problem**: Original implementation had a single flat costmap without proper layering for different cost sources.

**Solution**: Implemented **multi-layer architecture**:
1. **Static Layer**: From map server (permanent obstacles)
2. **Obstacle Layer**: From point cloud sensors (dynamic obstacles)
3. **Inflation Layer**: Safety margin around obstacles
4. **Combined Layer**: Master costmap combining all layers

## Architecture

### Costmap Layers

```
┌─────────────────────────────────────────┐
│         COMBINED COSTMAP                │
│  (Published to /costmap)                │
└─────────────────────────────────────────┘
              ▲
              │
    ┌─────────┴──────────┐
    │   Layer Combiner   │
    └─────────┬──────────┘
              │
   ┌──────────┼──────────┐
   │          │          │
┌──▼───┐  ┌──▼───┐  ┌──▼───┐
│Static│  │Obsta-│  │Infla-│
│Layer │  │cle   │  │tion  │
│      │  │Layer │  │Layer │
└──────┘  └──────┘  └──────┘
```

### Cost Values

- `FREE_SPACE = 0`: Confirmed free
- `INSCRIBED_INFLATED_OBSTACLE = 253`: Robot's inscribed radius touches obstacle
- `LETHAL_OBSTACLE = 254`: Certain obstacle
- `NO_INFORMATION = 255`: Unknown space

### Layer Priority

When combining layers:
1. **Static obstacles** have highest priority (permanent map features)
2. **Dynamic obstacles** override unknown space
3. **Inflation costs** are added around all obstacles
4. **Free space** from ray tracing clears unknown areas

## Ray Tracing Implementation

```cpp
void raytrace_line(int x0, int y0, int x1, int y1) {
  // Bresenham's algorithm
  // Trace from robot (x0,y0) to obstacle (x1,y1)
  // Mark all intermediate cells as FREE_SPACE
  // Don't clear the endpoint (that's the obstacle)
}
```

**Benefits**:
- Correctly marks space between robot and obstacles as free
- Prevents "unknown space" accumulation inside accessible areas
- Enables efficient path planning by reducing unknown regions
- Handles sensor occlusion naturally

## Configuration

### Parameters (`config/costmap.yaml`)

```yaml
costmap:
  ros__parameters:
    # Frames
    global_frame_id: odom
    base_frame_id: base_link
    
    # Grid size
    width: 500           # cells
    height: 500          # cells  
    resolution: 0.05     # meters/cell
    
    # Layers
    use_static_map: false         # Use map from map_server
    track_unknown_space: true     # Maintain unknown regions
    
    # Obstacle layer
    obstacle_max_range: 5.0       # meters
    obstacle_min_range: 0.1       # meters
    
    # Inflation layer
    inflation_radius: 0.3         # meters
    cost_scaling_factor: 10.0     # Exponential decay factor
```

### Key Parameters Explained

- **inflation_radius**: Safety buffer around obstacles
  - Robot inscribed radius should be less than this
  - Larger values = more conservative navigation
  
- **cost_scaling_factor**: Controls inflation cost decay
  - Higher values = steeper cost gradient
  - Formula: `cost = 254 * exp(-factor * (dist - res) / radius)`
  
- **track_unknown_space**: Keep unknown regions vs assume free
  - `true`: Conservative, marks unseen areas as unknown
  - `false`: Optimistic, assumes unseen areas are free

## Published Topics

- `/costmap` - Combined costmap (OccupancyGrid)
- `/costmap/static_layer` - Static map layer
- `/costmap/obstacle_layer` - Dynamic obstacles with ray tracing
- `/costmap/inflation_layer` - Inflation costs around obstacles

## Subscribed Topics

- Point cloud: Configurable (default: `/rosbot/camera_depth/point_cloud`)
- Static map: `/map` (if `use_static_map: true`)

## Visualization in RViz

Add these topics to RViz:
1. **Combined Costmap**: `/costmap`
   - Shows final navigation costmap
   - Color: Obstacles=black, Free=white, Inflated=gray
   
2. **Obstacle Layer**: `/costmap/obstacle_layer`
   - Shows ray-traced free space (white)
   - Obstacles from sensors (black)
   
3. **Inflation Layer**: `/costmap/inflation_layer`
   - Shows safety margins
   - Gradient from obstacles

## Integration with RTAB-Map

Updated RTAB-Map configuration to also use ray tracing:

```yaml
Grid/RayTracing: 'true'
Grid/Scan2dUnknownSpaceFilled: 'true'
```

This ensures both SLAM and costmap use consistent obstacle representation.

## Performance Considerations

1. **Ray Tracing Overhead**: 
   - Bresenham is very efficient: O(max(dx, dy))
   - Minimal CPU impact even with dense clouds

2. **Inflation Computation**:
   - Cached distance costs for efficiency
   - Runs only when obstacles change
   
3. **Memory**: 
   - 500x500 @ 0.05m = 250KB per layer
   - Total: ~1MB for all layers

## Comparison with Original

| Feature | Original | New Implementation |
|---------|----------|-------------------|
| Ray tracing | ❌ | ✅ |
| Layered architecture | ❌ | ✅ |
| Static map support | ❌ | ✅ |
| Inflation decay | Linear | Exponential |
| Unknown space handling | Poor | Proper |
| Layer visualization | ❌ | ✅ |

## Future Enhancements

1. **Voxel Layer**: 3D obstacle tracking with decay
2. **Spatio-Temporal Voxel Layer**: Track moving obstacles
3. **Range Sensor Layer**: Dedicated 2D laser scan layer
4. **Semantic Layer**: Classify obstacles by type
5. **Elevation Layer**: Height-aware costs

## References

- [Nav2 Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [Costmap Layering](https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d)
- [Bresenham's Line Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
