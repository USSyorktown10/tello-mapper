# Preflight Checklist - Autonomous Tello Drone

**Welcome, pilot!** Well, you not the pilot. But thats okay! This is how to get your drone set up so it can fly better than you :)

---

## Pre-Flight Checks (Before Any Mode)

### 1. Dependencies Installed
```bash
pip install -r requirements.txt
```
Verify all packages installed (torch, opencv, djitellopy, numpy, matplotlib, Pillow)

### 2. Model Weights Present
Verify these files exist: (They should come with the repository)
```
weights/lite-mono-small-640x192/encoder.pth
weights/lite-mono-small-640x192/depth.pth
```

### 3. Configuration Files
Check that these exist:
```
coco.names
yolov3.cfg
yolov3.weights
depth_config.txt (created after first run)
```

Note: This repo dosent come with yolov3.weights, as its too big of a file to include. You must download it separately.
[Download yolov3.weights](https://pjreddie.com/media/files/yolov3.weights)

After downloading, move `yolov3.weights` to the base tello-mapper directory. (Dont store under any folder in this directory. Example path: `/Users/USSyorktown10/Code/tello-mapper/yolov3.weights`)

---

## Mode 1: Simulation Testing (Webcam - No Drone Needed)

1. **Verify simulation mode**: In `flight_runner.py`:
   ```python
   USE_SIM = True    # ← Must be TRUE
   TEST_MODE = False # ← Can be anything
   ```

2. **Run the program**:
   ```bash
   python flight_runner.py
   ```

3. **Three windows should appear**:
   - Camera feed with crosshair and depth estimate
   - Depth map (colorized)
   - Occupancy grid map (white=free, black=obstacles)

4. **Test autonomous flight planning**:
   - Press `[` and `]` to adjust depth scale (testing calibration)
   - Press `a` to toggle autonomous flight planning mode
   - Watch the map as it shows path planning (NO actual movement, this is just simulation)
   - Press `q` to quit

5. **Expected behavior**:
   - Map updates in real-time with depth from webcam
   - When autonomous mode on, you see planning state in HUD
   - No errors in console

---

## Mode 2: Safe Testing with Real Tello (No Motors)

**Test with real drone but motors OFF - Best used for tuning model to your drone before actual flight**

### Pre-Flight Setup

1. **Tello battery**: Charge to >75% (recommended 100%)
2. **Tello power**: Turn drone ON
3. **Wi-Fi connection**: Connect computer to `TELLO-xxxxx` network
4. **Workspace**: Open area, preferably 4m x 4m minimum

### Tello Connection Test

```bash
python -c "from djitellopy import Tello; t = Tello(); t.connect(); print(f'Battery: {t.get_battery()}%'); t.streamon(); t.streamoff()"
```
Expected output: Battery percentage should print. If error, Tello not found.

### Configuration

Edit `flight_runner.py`:
```python
USE_SIM = False       # ← Use real Tello
TEST_MODE = True      # ← MOTORS OFF (critical!)
```

### Run Test Mode

```bash
python flight_runner.py
```

**Expected behavior**:
- Tello connects and streams video
- Three windows open (Camera, Depth, Map)
- HUD shows "TEST MODE (No Fly)"
- **Drone does NOT take off** (motors disabled)
- Depth scale displayed with `[` / `]` control

### Depth Calibration (TEST_MODE)

**Goal**: Fine-tune depth scale for accurate obstacle detection

1. **Place reference target**:
   - Put drone on level surface
   - Place wall or object **exactly 1 meter away**

2. **Adjust depth scale**:
   - Press `[` to decrease depth scale (makes distances appear farther)
   - Press `]` to increase depth scale (makes distances appear closer)
   - Adjust until HUD shows "≈ 1.0 m" (approximately 1 meter)

3. **Note**: Depth estimation is not perfectly accurate. Don't obsess over exact calibration. "Close enough" is fine.

4. **Save configuration**:
   - Adjusted value auto-saves to `depth_config.txt` on exit
   - On next run, this scale will be loaded automatically

### Test Autonomous Planning (TEST_MODE)

**Try autonomous mode WITHOUT actual flight**:

1. Press `a` to enable autonomous mode
2. Watch the HUD: should show "State: exploring" or similar
3. On the map window, watch path planning
4. **NO MOVEMENT COMMANDS SENT** (motors off)
5. This proves autonomous logic works before real flight

### Exit TEST_MODE

Press `q` to quit and land safely (no takeoff occurred)

---

## Mode 3: Real Autonomous Flight

**WARNING: DRONE WILL TAKE OFF IN 3 SECONDS**

### Pre-Flight Checklist

Before enabling real flight:

- [ ] Tello battery >75% (ideally 100%)
- [ ] Tello connected to Wi-Fi
- [ ] Workspace is **clear and large** (6m x 6m minimum)
- [ ] No people, pets, or obstacles in takeoff area
- [ ] Depth calibration done (from TEST_MODE)
- [ ] You are near the drone to catch it if needed
- [ ] You understand you can press `q` anytime to land

### Configuration

Edit `flight_runner.py`:
```python
USE_SIM = False       # ← Use real Tello
TEST_MODE = False     # ← MOTORS ON (will fly!)
```

### Pre-Launch Warnings

The drone will:
1. **Take off automatically in 3 seconds**
2. **Navigate autonomously** to explore the environment
3. **Send RC control commands** to move around
4. **Detect and avoid obstacles** (or try to)
5. **Continue until map is fully explored** then land

You can:
- Press `a` to toggle autonomous flight mode (default starts exploring)
- Press `q` **ANYTIME** to emergency land
- Press `[` / `]` during flight to adjust depth scale

### Launch

```bash
python flight_runner.py
```

**At launch** (3-second countdown):
```
Taking off in 3 seconds...
  (count down displayed)
```

**Then**:
- Tello lifts off
- Autonomous mode starts exploring
- Camera, depth, and map windows show live data
- HUD shows flight state (exploring, planning, moving, hovering, landing)

### What To Expect

**First 5-10 seconds**:
- Drone hovers at starting position
- Depth map fills with sensor data
- Occupancy grid updates
- Autonomous mode scans for frontiers

**Next 10-30 seconds**:
- Drone identifies frontier (unexplored area)
- Plans path using A* algorithm
- Starts moving towards frontier
- Sends movement commands (`send_rc_control`)
- Follows waypoints

**During flight**:
- Watch map window - shows planned path in white
- Watch camera window - HUD shows state transitions
- If obstacle detected ahead: drone hovers
- If stuck on waypoint: restarts exploration

**Mission complete**:
- When all frontiers explored
- Drone auto-lands
- Program exits

### Emergency Landing

Press `q` **anytime** to:
1. Stop autonomous mode
2. Immediately land drone
3. Exit program

---

## Troubleshooting Pre-Flight

### Tello Won't Connect
```
Error: Failed to connect to Tello
```
- Check: Is Tello powered ON?
- Check: Is computer connected to TELLO-xxxxx Wi-Fi?
- Check: Is djitellopy installed? `pip install djitellopy`
- Try: Restart Tello and reconnect to Wi-Fi

### Depth Map Empty
- Check: Is webcam accessible? (for SIM mode)
- Check: Is Tello stream on? (for real mode)
- Check: Are weights loaded? Check console for errors
- Try: Restart program

### Model Loading Error
```
Error: No such file or directory: weights/...
```
- Check: Weights directory exists?
- Check: Are encoder.pth and depth.pth present?
- Solution: Download model weights from Lite-Mono repo

### Battery Warning
```
Battery: 45%
```
- Charge Tello to >75% (critical for autonomous flight)
- Low battery = unreliable flight performance

### Autonomous Mode Not Starting
```
Press 'a' but nothing happens
```
- Check: Are you in real drone mode? (USE_SIM=False, TEST_MODE=False)
- Check: Did drone take off? Autonomous only works after takeoff
- Check: Is drone armed and ready?

---

## Post-Flight Checks

After each flight:

1. **Battery check**: Charge Tello for next flight
2. **Propeller check**: Inspect propellers for damage
3. **Depth calibration review**: Was depth accurate? Recalibrate if needed
4. **Log review**: Check console output for errors/warnings

---

## Configuration Reference

| Setting | Value | Effect |
|---------|-------|--------|
| `USE_SIM` | True | Use webcam (simulation) |
| `USE_SIM` | False | Use real Tello drone |
| `TEST_MODE` | True | Motors OFF (safe testing) |
| `TEST_MODE` | False | Motors ON (actual flight) |
| `DEPTH_SCALE` | 5.0 | Calibration factor for depth |
| `DEPTH_EVERY_N` | 3 | Process depth every N frames |
| `DEPTH_DOWNSAMPLE` | 4 | Grid resolution factor |

---

## Flight Control (During Flight)

| Key | Action |
|-----|--------|
| `a` | Toggle autonomous mode |
| `[` | Decrease depth scale |
| `]` | Increase depth scale |
| `q` | Emergency land and quit |

---

## Autonomous Flight Logic (What Happens When You Press `a`)

1. **Frontier Detection**: Scans map for unexplored areas
2. **Path Planning**: Uses A* to find collision-free route
3. **Movement**: Sends proportional RC commands
4. **Obstacle Avoidance**: Checks ahead for obstacles
5. **Stuck Recovery**: Restarts if blocked
6. **Mission Complete**: Auto-lands when fully explored

See `FLIGHT_CONTROL.md` for complete technical details.

---

## Success Criteria

✅ **Simulation mode works**: Webcam feed, depth map, grid all display
✅ **TEST_MODE works**: Real Tello connects, no takeoff, planning visible
✅ **Autonomous mode**: Drone takes off, explores, lands (if attempted)

**You're ready to fly!**