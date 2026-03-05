# Autonomous Flight Control System - Technical Reference

Everything you need and more to ensure you understand the code, and the code works better than the drone itself. (The code definitely works better than the drone... DEFINITELY... *Im sorry DJI*)

This is also the manual that everyone throws out when they get the product. If you dont feel like reading, dont. If you have nothing better to do and have more time than atoms in the observable universe, have fun reading!
- Things you should actually read:
  - `README.md`
  - `PREFLIGHT-CHECKLIST.md`

## Overview

The `FlightController` class manages autonomous exploration of the environment. It implements a state machine that:
1. Identifies frontier cells (boundaries between mapped and unmapped areas)
2. Plans collision-free paths using A*
3. Executes waypoint-following navigation
4. Handles obstacles and error conditions

## Architecture

### State Machine

```
IDLE
  ↓
exploring ←──────────────────────────────┐
  ├─→ (frontier found) planning          │
  │       ├─→ (path found) moving        │
  │       │     ├─→ (waypoint reached) → back to exploring
  │       │     ├─→ (obstacle ahead) hovering
  │       │     └─→ (stuck) → back to exploring
  │       └─→ (no path) → back to exploring
  │
  └─→ (no frontier) landing
```

### State Descriptions

| State | Purpose | Conditions |
|-------|---------|-----------|
| **exploring** | Scan for frontier cells | All exploration targets exhausted? → landing |
| **planning** | Compute A* path to frontier | Success? → moving \| Fail? → exploring |
| **moving** | Follow waypoints | Complete? → exploring \| Obstacle? → hovering \| Stuck? → exploring |
| **hovering** | Pause on obstacle | Obstacle cleared? → moving \| Timeout? → exploring |
| **landing** | Safe descent | Final state |

## Implementation Details

### 1. Frontier Detection

**Function**: `find_frontier(grid, origin_cell, min_distance, max_distance)`

A frontier cell must satisfy:
- Value < 0.3 (classified as free)
- Adjacent to at least one cell with value 0.4-0.6 (unknown)
- Distance from origin between `min_distance` and `max_distance` cells

**Algorithm** (O(n²) per frame):
```python
for each cell (x, y):
    if grid[y,x] < 0.3:
        for each neighbor:
            if 0.4 <= grid[neighbor] <= 0.6:
                cell is frontier
                distance = manhattan(origin, cell)
                if min_dist <= distance <= max_dist:
                    add to candidates
return nearest candidate
```

**Optimization**: Precompute `find_all_frontiers()` to get multiple candidates

### 2. Path Planning

**Function**: `plan_path_to_target(grid, start, target)`

Uses A* pathfinding on a binary grid:
- Thresholded: < 0.5 = free (0), ≥ 0.5 = obstacle (255)
- Manhattan heuristic
- Diagonal movement allowed

Returns waypoint sequence from start to target (or empty list if unreachable).

**Cost Model**:
- Orthogonal moves: cost = 1.0
- Diagonal moves: cost = 1.4

### 3. Waypoint Following

**Method**: `_move_to_waypoint(target_cell)`

**Algorithm**:
```
current_cell ← get_drone_cell()
offset ← target_cell - current_cell
if |offset_x| > |offset_y|:
    move left/right
else:
    move forward/back
speed ← 30 (Tello 1-100 scale)
send_rc_control(lr, fb, ud=0, yv=0)
```

**Waypoint Completion**: When distance to waypoint < 2 cells

**Stuck Detection**: If not reaching waypoint for 30 consecutive frames (~1 second), restart exploration

### 4. Obstacle Avoidance

**Method**: `_check_obstacle_ahead(occupancy_grid, drone_cell)`

Checks 5 cells ahead:
```python
for d in 1..5:
    check_cell ← (cx, cy + d)
    if grid[check_cell] > 0.7:  # Occupied
        return True
return False
```

On detection:
- Hover (send zero velocity)
- Replan path
- Resume moving

## State Update Loop

Called once per frame with current occupancy grid:

```python
controller.update(occupancy_grid)  # ~30 FPS
```

### Logic Per State

#### exploring
```python
frontier = find_frontier(grid, current_cell, min_dist=5, max_dist=50)
if frontier:
    state = planning
else:
    state = landing  # Mission complete
```

#### planning
```python
path = plan_path_to_target(grid, current_cell, frontier)
if len(path) > 1:
    current_path = path
    path_index = 0
    state = moving
else:
    state = exploring  # Can't reach frontier, find another
```

#### moving
```python
waypoint = current_path[path_index]
move_to_waypoint(waypoint)

dist = manhattan(current_cell, waypoint)
if dist < 2:
    path_index += 1
    stuck_counter = 0
else:
    stuck_counter += 1
    if stuck_counter > 30:
        state = exploring  # Stuck, restart
        
if path_index >= len(path):
    state = exploring  # Path complete
```

#### hovering
```python
drone.send_rc_control(0, 0, 0, 0)  # Maintain position
if not check_obstacle_ahead(grid, current_cell):
    state = moving
```

#### landing
```python
drone.land()  # Blocks until complete
# Mission over
```

## Configuration

### Global Parameters

In `flight_runner.py`:
```python
USE_SIM = True          # False for real Tello
TEST_MODE = False       # True = no motors
DEPTH_EVERY_N = 3       # Inference rate
DEPTH_DOWNSAMPLE = 4    # Grid resolution
DEPTH_SCALE = 5.0       # Depth calibration
```

### FlightController Parameters

In `FlightController.__init__()`:
```python
# Frontier search
min_distance = 5        # cells = 0.5m
max_distance = 50       # cells = 5.0m

# Movement
speed = 30              # Tello RC scale (1-100)

# Safety
stuck_threshold = 30    # frames before "stuck"
check_distance = 5      # cells to lookahead
```

## Integration in Main Loop

```python
# After depth mapping:
if auto_flight and flight_controller:
    try:
        flight_controller.update(mapper.grid)
    except Exception as e:
        print(f"Flight error: {e}")
        auto_flight = False
        drone.land()
```

## Safety Considerations

### Emergency Landing
- Any unhandled exception triggers automatic land
- Graceful shutdown on user input (`q` key)

### Obstacle Safety
- Conservative thresholds (>0.7 = definitely obstacle)
- Lookahead distance prevents surprise collisions
- Stuck detection prevents fruitless movement

### Conservative Depth Mapping
- Top 15% and bottom 30% of image ignored (ceiling/floor)
- Heuristic obstacle detection (may be conservative)
- Always assume unknown is potentially dangerous

## Performance Characteristics

### Timing
- Frontier detection: O(n²) per call, ~50ms for 100x100 grid
- Path planning: O(n² log n) typical, <100ms for open areas
- Movement command: ~1ms
- Frame rate: 30 FPS (one depth inference every 3 frames)

### Memory
- Occupancy grid: ~1KB per cell value (8000 cells = 8MB for 80m² @ 0.1m resolution)
- Path cache: few KB (typically <100 waypoints)
- State overhead: <1KB

### Bottleneck
- Depth inference: ~100ms per frame on CPU (3 FPS effective)
- Frontier detection: O(n²) scales linearly with map area

## Customization Examples

### Custom Movement Strategy
Override `_move_to_waypoint()`:
```python
def _move_to_waypoint(self, target_cell):
    # Implement PID control, curved paths, etc.
    pass
```

### Frontier Scoring
Modify frontier selection in `update()`:
```python
# Find multiple frontiers, score by unexploredness
frontiers = self.planner.find_all_frontiers(
    occupancy_grid, drone_cell, max_distance=50, limit=10
)
best = max(frontiers, key=lambda f: frontier_score(f, grid))
```

### Altitude Control
Add altitude adjustment:
```python
# Climb towards obstacles
if center_dist_m < 1.0:
    ud = 30  # Climb
else:
    ud = 0   # Maintain
drone.send_rc_control(lr, fb, ud, yv)
```

### Return-to-Origin
Track visited cells and return path:
```python
if mission_complete:
    origin = self.mapper.world_to_cell(0, 0)
    path_home = astar(grid, current_cell, origin)
    # Follow path_home back to start
```

## Debugging

### Console Output
Add debug prints in state transitions:
```python
print(f"State: {self.state}, Frontier: {self.target_frontier}, Path length: {len(self.current_path)}")
```

### Visualization
The map window shows:
- White = free space
- Black = obstacles  
- Grey = unknown (frontiers marked in plan phase)

### Manual Control
Even in autonomous mode, can still use:
- `[` / `]` keys: adjust depth scale (affects mapping)
- `q` key: emergency land

## Limitations & Known Issues

1. **No Orientation Tracking**: Drone doesn't track heading; assumes forward-facing camera
2. **Simple Movement**: Proportional control only; no momentum compensation
3. **Frontier Bias**: Always picks nearest frontier; may miss better exploration targets
4. **Grid Resolution**: 0.1m fixed; could be adaptive
5. **Depth Uncertainty**: Monocular estimates can be unreliable in low texture areas

## Future Enhancements

- [ ] Yaw alignment (rotate to face waypoint)
- [ ] PID controller for smooth movement
- [ ] Information gain metric for frontier scoring
- [ ] Multi-frontier coordination
- [ ] Return-to-start after exploration
- [ ] Adaptive depth scale based on lighting
- [ ] Integration with Tello IMU for better odometry
