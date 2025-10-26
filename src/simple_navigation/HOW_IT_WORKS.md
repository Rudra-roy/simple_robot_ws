# Adaptive Planner - How It Works

## Navigation Strategy

The Adaptive Planner uses a **two-tier navigation approach**:

### 1. Primary Navigation: Straight Line Path
- When you send a goal, the robot **always starts with a straight line path**
- This is the most efficient route when there are no obstacles
- The straight line is divided into waypoints for smooth following

### 2. Adaptive Navigation: A* Replanning
- The robot **continuously monitors** the path ahead (up to `replan_distance`)
- When an **obstacle is detected** blocking the straight line:
  - **STOP** the robot immediately 🛑
  - **REPLAN** using A* algorithm to find path around obstacle
  - **VISUALIZE** the new path in RViz (wait 1 second)
  - **RESUME** navigation following the new path ▶️

## Key Differences from Before

### ✅ NOW (Correct Behavior)
- **Primary**: Straight line path (like waypoint_navigator)
- **Secondary**: A* only when obstacles detected
- **Costmap bounds**: Points outside costmap are **valid** (not obstacles)
- Robot uses efficient straight paths when possible

### ❌ BEFORE (Wrong Behavior)
- Always used A* planning
- Points outside costmap were flagged as obstacles
- Over-complicated navigation even in open space

## Behavior Flow

```
┌─────────────────────────────────────────────────────┐
│ 1. Receive Goal                                     │
│    → Plan STRAIGHT LINE path                        │
│    → Visualize in RViz                              │
└─────────────┬───────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────┐
│ 2. Follow Straight Path                             │
│    → Move towards waypoints                         │
│    → Check for obstacles ahead                      │
└─────────────┬───────────────────────────────────────┘
              │
              ▼
         ┌────┴────┐
         │ Obstacle│
         │ Ahead?  │
         └────┬────┘
              │
      ┌───────┴────────┐
      │ NO              │ YES
      ▼                 ▼
┌───────────┐   ┌─────────────────────────────────┐
│ Continue  │   │ 3. Adaptive Replanning           │
│ Straight  │   │    → STOP robot                  │
│ Path      │   │    → Run A* to avoid obstacle    │
│           │   │    → Visualize new path (1 sec)  │
└─────┬─────┘   │    → RESUME with new path        │
      │         └──────────┬──────────────────────┘
      │                    │
      └────────┬───────────┘
               │
               ▼
      ┌────────────────┐
      │ Reached Goal?  │
      └────┬───────────┘
           │
    ┌──────┴───────┐
    │ NO           │ YES
    │              │
    │              ▼
    │      ┌──────────────┐
    │      │ 🎉 Success!  │
    │      └──────────────┘
    │
    └──────► (Loop back to step 2)
```

## Costmap Behavior

### Inside Costmap Bounds
- Check actual cost value
- `cost < obstacle_threshold` → **FREE**
- `cost >= obstacle_threshold` → **OBSTACLE**

### Outside Costmap Bounds
- **Always treated as FREE space**
- Robot can plan paths beyond costmap coverage
- Prevents false obstacle detection in unmapped areas

## Example Scenario

### Scenario: Navigate from (0, 0) to (5, 5)

**Without Obstacles:**
```
Start → (straight line) → Goal
        ✅ Simple, efficient
```

**With Obstacle at (2.5, 2.5):**
```
Start → (straight line) → [OBSTACLE DETECTED!] 
                              ↓
                          [STOP & REPLAN]
                              ↓
Start → (curve around) → Goal
        ✅ A* finds best path
```

## Configuration

### Key Parameters

```python
'replan_distance': 1.0    # Check 1m ahead for obstacles
'obstacle_threshold': 50  # Costmap values ≥50 are obstacles
'waypoint_tolerance': 0.3 # How close to waypoint before advancing
```

### Tuning Tips

- **Large `replan_distance`**: Earlier obstacle detection, more replanning
- **Small `replan_distance`**: Later detection, less replanning
- **High `obstacle_threshold`**: More aggressive navigation (closer to obstacles)
- **Low `obstacle_threshold`**: More conservative (wider berth around obstacles)

## Topics Summary

### Inputs
- `/goal_pose` - Where to navigate
- `/odometry/filtered` - Robot position
- `/costmap` - Obstacle map

### Outputs
- `/cmd_vel` - Robot movement commands
- `/planned_path` - Path visualization (changes when replanning!)
- `/navigation_markers` - Current waypoint (green) and goal (red)

## RViz Visualization

Watch these displays to understand robot behavior:

1. **Path** (`/planned_path`) 
   - **Straight** = Using primary navigation
   - **Curved** = Using adaptive navigation (avoiding obstacle)

2. **MarkerArray** (`/navigation_markers`)
   - **Green sphere** = Current waypoint robot is tracking
   - **Red arrow** = Final goal

3. **Map** (`/costmap`)
   - **White** = Free space
   - **Black** = Obstacles
   - **Gray** = Unknown

When you see the path suddenly change from straight to curved, that's the adaptive planner in action! 🎯
