# Path Visualization - Quick Reference

## What Changed

The `/planned_path` topic now shows BOTH types of paths based on the robot's navigation mode:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PATH VISUALIZATION                            │
└─────────────────────────────────────────────────────────────────┘

╔═══════════════════════════════════════════════════════════════╗
║ GLOBAL_PLANNER Mode                                            ║
╠═══════════════════════════════════════════════════════════════╣
║                                                                 ║
║    Robot ●────────────────────────────────► ⭐ Goal           ║
║          └─ Green straight line (z=0.1m)                       ║
║                                                                 ║
║    Published: Global path (straight-line interpolation)        ║
║    Update Rate: 10 Hz                                          ║
║    Purpose: Show direct route when clear                       ║
╚═══════════════════════════════════════════════════════════════╝

╔═══════════════════════════════════════════════════════════════╗
║ LOCAL_AVOIDANCE Mode                                           ║
╠═══════════════════════════════════════════════════════════════╣
║                                                                 ║
║    Robot ●───┐                    ┌───► ⭐ Goal               ║
║              └──┐            ┌────┘                            ║
║                 └──┐  ███  ┌─┘  ← Red curved path (z=0.0m)    ║
║                    └───█───┘                                   ║
║                      Obstacle                                  ║
║                                                                 ║
║    Published: Local A* path (obstacle avoidance)               ║
║    Update Rate: 10 Hz                                          ║
║    Purpose: Show planned route around obstacles                ║
╚═══════════════════════════════════════════════════════════════╝
```

## Visual Differences in RViz

### Height Distinction
```
Side View:

 0.1m  ─────────────────  ← Global path (elevated)
       
 0.0m  ═══════════════════ ← Local path (ground level)
       ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
       Ground
```

### In RViz Display

```
Top View in RViz:

Clear Path Scenario (GLOBAL_PLANNER):
┌──────────────────────────────────┐
│                                  │
│   Robot 🤖 ━━━━━━━━━━━► ⭐      │
│           (Green line)            │
│                                  │
└──────────────────────────────────┘

Obstacle Avoidance (LOCAL_AVOIDANCE):
┌──────────────────────────────────┐
│                                  │
│   Robot 🤖 ━┓                    │
│            ┃  ┏━━━━━━━► ⭐       │
│            ┗━━┛ (Red curved)     │
│              ██                   │
│            Obstacle               │
└──────────────────────────────────┘
```

## State Transitions

```
State Machine Flow:

    ┌─────────────┐
    │    IDLE     │
    └──────┬──────┘
           │ Goal received
           ▼
    ┌─────────────────────┐
    │  GLOBAL_PLANNER     │  ← Publishes: Global straight-line path
    │  (Straight to goal) │
    └──────┬──────────────┘
           │ Obstacle detected
           ▼
    ┌─────────────────────┐
    │    SURVEYING        │  ← Publishes: Nothing (scanning)
    │   (360° rotation)   │
    └──────┬──────────────┘
           │ A* path planned
           ▼
    ┌─────────────────────┐
    │  LOCAL_AVOIDANCE    │  ← Publishes: Local A* path
    │  (Follow A* path)   │
    └──────┬──────────────┘
           │ Clearance reached
           ▼
    ┌─────────────────────┐
    │  GLOBAL_PLANNER     │  ← Publishes: Global straight-line path
    │  (Back to direct)   │
    └─────────────────────┘
```

## Code Changes Summary

### Before (Original)
```python
# Only published A* paths, no global visualization
def publish_path(self, path_waypoints):
    # Simple path publishing
    # No path type distinction
    # No global path during direct navigation
```

### After (Updated)
```python
# 1. Enhanced path publishing
def publish_path(self, path_waypoints, path_type="local"):
    # Distinguishes between global and local paths
    # Different z-heights for visual distinction
    # Adds proper orientations to waypoints

# 2. New global path generator
def publish_global_path_to_goal(self):
    # Creates straight-line path
    # Interpolates waypoints every 0.5m
    # Publishes as "global" type

# 3. Updated state execution
def execute_global_planner(self):
    # NOW: Publishes global path every iteration
    self.publish_global_path_to_goal()

def execute_local_avoidance(self):
    # NOW: Republishes local path every iteration
    self.publish_path(self.current_path, path_type="local")
```

## RViz Setup Guide

### Step 1: Add Path Display
```
RViz → Add → By topic → /planned_path → Path
```

### Step 2: Configure Display
```
Display Properties:
├─ Topic: /planned_path
├─ Alpha: 1.0
├─ Color Scheme: Flat Color
├─ Color: Your choice (e.g., cyan)
├─ Line Width: 0.05
├─ Pose Style: Arrows
└─ Shaft Length: 0.1
```

### Step 3: Observe
- **Green/Blue arrows**: Robot going straight to goal
- **Red/Yellow arrows**: Robot navigating around obstacles
- Arrows show path direction
- Real-time updates as robot moves

## Testing the Changes

### Test 1: Clear Path
```bash
# Set a goal with clear path
# Expected: Straight line path from robot to goal
# Path Type: global (z=0.1m)
```

### Test 2: Obstacle in Path
```bash
# Set a goal behind an obstacle
# Expected: 
#   1. Initial straight line (global)
#   2. Disappears during survey
#   3. Curved path appears (local A*)
# Path Type: local (z=0.0m)
```

### Test 3: Dynamic Replanning
```bash
# Follow A* path, introduce new obstacle
# Expected: Path regenerates around new obstacle
# Path Type: local (continuously updated)
```

## Topic Monitoring

```bash
# Monitor path publishing
ros2 topic hz /planned_path
# Expected: ~10 Hz

# View path messages
ros2 topic echo /planned_path
# Check z values: 0.1 (global) or 0.0 (local)

# Check planner state
ros2 topic echo /planner_state
# Correlate state with path type
```

## Benefits at a Glance

✅ **Always see the plan**: Path visible in both modes
✅ **Know the mode**: Visual distinction between global/local
✅ **Better debugging**: See if paths make sense
✅ **Real-time updates**: Paths update as robot moves
✅ **Orientation info**: Arrows show travel direction
✅ **Height separation**: Easy to distinguish in 3D view

## Quick Troubleshooting

| Issue | Check | Solution |
|-------|-------|----------|
| No path visible | Topic hz | Verify planner is running |
| Path not updating | Robot state | Check odometry input |
| Wrong path type | `/planner_state` | Verify state matches path |
| Flickering | Mode transitions | Normal during replanning |
| Flat appearance | RViz pose style | Set to "Arrows" |

---

**Result**: The `/planned_path` topic now comprehensively visualizes your robot's navigation intentions, whether going straight or navigating around obstacles! 🎯
