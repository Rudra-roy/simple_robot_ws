## Ray Tracing Visualization

### Before Ray Tracing (Your Issue #1)

```
Legend: R = Robot, O = Obstacle, ? = Unknown, · = Free

┌────────────────────────────────┐
│  · · · · · · · · · · · · · ·  │
│  · · · · · O O O O · · · · ·  │
│  · · · · O ? ? ? ? O · · · ·  │  ❌ Problem: Interior
│  · · · · O ? ? ? ? O · · · ·  │     is marked as UNKNOWN
│  · · · · O ? ? ? ? O · · · ·  │     (white/gray in your image)
│  · · · · O O O O O O · · · ·  │
│  · · · R · · · · · · · · · ·  │
│  · · · · · · · · · · · · · ·  │
└────────────────────────────────┘

Sensor sees:  ████  (only outer surface)
Interior:     ????  (never seen = unknown)
Planning:     SLOW  (tries to explore interior)
```

### After Ray Tracing (Fixed)

```
Legend: R = Robot, O = Obstacle, · = Free, ─ = Ray

┌────────────────────────────────┐
│  · · · · · · · · · · · · · ·  │
│  · · · · · O O O O · · · · ·  │
│  · · · · O · · · · O · · · ·  │  ✅ Solution: Ray tracing
│  · · ─ ─ O · · · · O · · · ·  │     marks interior as FREE
│  · · · · O · · · · O · · · ·  │     (sensor beam passed through)
│  · · · · O O O O O O · · · ·  │
│  · · · R · · · · · · · · · ·  │
│  · · · · · · · · · · · · · ·  │
└────────────────────────────────┘

Sensor ray:   ──────►O  (hits obstacle)
Free space:   ······    (ray passed through)
Endpoint:     O         (obstacle detected)
Planning:     FAST      (knows interior is inaccessible)
```

### How Ray Tracing Works

```
Step 1: Sensor detects obstacle point
        Robot at (0,0), Obstacle at (5,3)

Step 2: Trace ray using Bresenham's algorithm
        (0,0) → (1,0) → (2,1) → (3,2) → (4,2) → (5,3)
         FREE    FREE    FREE    FREE    FREE    OBS

Step 3: Mark cells
        Path cells = FREE_SPACE (0)
        Endpoint   = LETHAL_OBSTACLE (254)

Result: Space between robot and obstacle is confirmed FREE
```

### Layered Costmap (Your Issue #2)

```
Layer 1: STATIC LAYER (from map)
┌─────────────────┐
│ · · · · · · · · │  Permanent obstacles
│ · O O O · · · · │  from pre-built map
│ · O O O · · · · │
│ · · · · · · · · │
└─────────────────┘

Layer 2: OBSTACLE LAYER (from sensors + ray tracing)
┌─────────────────┐
│ · · · · · · · · │  Dynamic obstacles
│ · · · · · O · · │  with ray-traced
│ · · R ─ ─ O · · │  free space
│ · · · · · · · · │
└─────────────────┘

Layer 3: INFLATION LAYER (safety margin)
┌─────────────────┐
│ 10 20 30 30 20  │  Cost gradient
│ 20 50 80 80 50  │  around obstacles
│ 30 80 ██ 80 50  │  (exponential decay)
│ 20 50 80 80 50  │
└─────────────────┘

Combined: FINAL COSTMAP
┌─────────────────┐
│ 10 20 30 30 20  │  Max of all layers
│ 20 ██ ██ 80 50  │  Static + Dynamic
│ 30 ██ ██ ██ 50  │  + Inflation
│ 20 50 80 80 50  │
└─────────────────┘
```

### Cost Value Meanings

```
Value  | Color in RViz | Meaning              | Planner Action
-------|---------------|----------------------|------------------
  0    | White         | FREE_SPACE           | Safe to navigate
 1-252 | Gray          | Inflated cost        | Prefer to avoid
 253   | Dark gray     | INSCRIBED_INFLATED   | Robot edge here
 254   | Black         | LETHAL_OBSTACLE      | Cannot pass
 255   | Light gray    | NO_INFORMATION       | Unknown (explore?)
```

### Inflation Cost Formula

```
distance = euclidean_distance(cell, nearest_obstacle)

if distance <= inflation_radius:
    cost = 254 * exp(-cost_scaling_factor * (distance / inflation_radius))
else:
    cost = 0

Example with inflation_radius=0.3m, cost_scaling_factor=10.0:
    0.00m → 254 (lethal)
    0.05m → 223 (very high)
    0.10m → 153 (high)
    0.15m → 84  (medium)
    0.20m → 46  (low)
    0.25m → 25  (very low)
    0.30m → 0   (free)
```

### Real-World Example

Your obstacle (chair with legs):

```
Before (Issue):
    Top view of chair:
    ┌─────────────┐
    │ O O     O O │  Legs visible
    │ O         O │
    │ O    ?    O │  Interior = unknown
    │ O    ?    O │  Planner confused!
    │ O O     O O │
    └─────────────┘

After (Fixed):
    Top view of chair:
    ┌─────────────┐
    │ O O ─ ─ O O │  Legs visible
    │ O · · · · O │
    │ O · · · · O │  Interior = free (ray traced)
    │ O · · · · O │  Planner understands!
    │ O O ─ ─ O O │
    └─────────────┘
    
    Rays from robot see through gaps between legs
    → Mark interior as accessible
    → Planner knows this is navigable space
```

## Why This Matters

1. **Efficiency**: Planner doesn't waste time exploring known-inaccessible spaces
2. **Accuracy**: Costmap reflects reality (space IS accessible between legs)
3. **Consistency**: SLAM and navigation use same obstacle model
4. **Standard**: Matches Nav2 architecture (your second image)

## Testing Checklist

- [ ] Build updated nav2_costmap_node
- [ ] Launch navigation stack
- [ ] Check `/costmap/obstacle_layer` shows ray-traced free space
- [ ] Check `/costmap/inflation_layer` shows smooth gradients
- [ ] Verify obstacle interiors are no longer white/gray
- [ ] Compare with your second image - should match!
