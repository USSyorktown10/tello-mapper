# Tello Drone Autonomous Mapping & Navigation

A real-time autonomous navigation system for DJI Tello drones that uses depth estimation from the camera, grid mapping, and A* path planning. Can operate in simulation mode (webcam) or with a real Tello drone.

## Features

- **Depth Estimation**: Uses Lite-Mono deep learning model to estimate 3D depth from single camera frames
- **Real-Time Grid Mapping**: Converts depth estimates into a probabilistic occupancy grid
- **A* Path Planning**: Makes collision-free paths through the mapped environment (frontiers I think they are called?)
- **Autonomous Exploration**: Automatically identifies and explores unmapped areas
- **Waypoint Following**: Navigates via computed waypoints with obstacle avoidance
- **Dual Mode Operation**:
  - **Simulation Mode**: Test with your webcam (no drone required)
  - **Flight Mode**: Control a real DJI Tello drone with autonomous navigation
- **Flight Control**: Takeoff, landing, movement commands via Tello SDK
- **Depth Scale Calibration**: Interactive tuning of depth scale factor (persisted to `depth_config.txt`)

## Important Limitations

1. **Depth is Not True Range**: The Lite-Mono model estimates relative depth, not absolute distance. The system uses a calibrated scale factor (`DEPTH_SCALE`) to convert relative depth to meters, but this is approximate and may require tuning.
2. **Heuristic-Based Obstacle Detection**: The depth-to-occupancy grid conversion uses conservative heuristics (e.g., ignoring top 15% and bottom 30% of image to avoid ceiling/floor artifacts).
3. **Limited Field of View**: Obstacles directly above or behind the drone cannot be detected.
4. **Simplified Movement Model**: Current waypoint following uses simple proportional control; doesn't account for momentum or orientation.

To get accurate range measurements, consider adding:
- **LiDAR or Ultrasonic Sensors**: Provide ground-truth distance
- **Stereo Cameras**: Enable true stereo depth (beter than one camera, you can triangulate with stereo I think)

## System Architecture

### Core Modules

| File | Purpose |
|------|---------|
| `flight_runner.py` | Main application: runs camera, depth inference, mapping, and flight control |
| `mapper.py` | `Mapper` class: maintains occupancy grid, processes depth maps, visualizes map |
| `planner.py` | `astar()` function for path planning on occupancy grid; exploration utilities |
| `camera.py` | Alternative demo using YOLOv3 object detection (not used by main system) |
| `test_simple.py` | Lite-Mono model testing utility (standalone) |
| `lilboi.py` | Simple wall mapping demo (alternative exploration demo) |
| `Lite-Mono/` | Submodule: deep learning model for monocular depth estimation |
| `networks/` | Custom PyTorch layers for Lite-Mono depth encoder/decoder |

### Dependencies

See `requirements.txt`. Key packages:
- **PyTorch 2.0.1** + **TorchVision 0.15.2**: Deep learning framework
- **OpenCV 4.8**: Computer vision and video I/O
- **djitellopy 2.4**: DJI Tello SDK (Python bindings)
- **NumPy 1.24**: Numerical computing

## Setup Instructions

### Prerequisites

- **Python 3.8+** (tested on Python 3.10+)
- **pip** package manager
- **yolov3.weights** must be downloaded from yolo, it was too big for me to upload to GitHub
- *(Optional)* NVIDIA GPU with CUDA 11.8+ for faster inference (CPU works but is ~5x slower)

### Installation

1. **Clone the repository** (including the Lite-Mono submodule):
   ```bash
   git clone --recursive https://github.com/ussyorktown10/tello-mapper.git
   cd tello-mapper
   ```

2. **Create a Python virtual environment**:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Verify setup**:
   ```bash
   python -c "import torch; print(f'PyTorch: {torch.__version__}'); import cv2; print('OpenCV OK')"
   ```

### GPU Setup (Optional but Recommended)

If you have an NVIDIA GPU:
1. Install CUDA 11.8 from https://developer.nvidia.com/cuda-11-8-0-download-archive
2. Replace `torch==2.0.1` and `torchvision==0.15.2` in `requirements.txt` with GPU variants:
   ```
   torch==2.0.1+cu118
   torchvision==0.15.2+cu118
   ```
3. Reinstall: `pip install -r requirements.txt --index-url https://download.pytorch.org/whl/cu118`

## Usage

### Simulation Mode (Webcam, Recommended for First Test)

Run with your webcam—no drone needed:
```bash
python flight_runner.py
```

You'll see 3 windows:
- **Camera**: Live webcam feed with HUD (distance estimate, depth scale)
- **Depth**: Colorized depth map from Lite-Mono (magma colormap)
- **Map**: Occupancy grid (white=free, black=obstacle)

**Controls**:
- `[` / `]`: Decrease/increase depth scale (for calibration)
- `a`: Toggle autonomous flight mode (TEST_MODE only - no takeoff)
- `q`: Quit

The depth scale is automatically saved to `depth_config.txt` when you exit.

### Real Drone Mode (Tello)

1. **Ensure Tello is ready**:
   - Tello battery charged (>75% recommended)
   - Tello powered on and discoverable
   - Your computer connected to Tello's Wi-Fi network (SSID: `TELLO-xxxxx`)

2. **Edit `flight_runner.py`**:
   - Set `USE_SIM = False` (line 11)
   - Set `TEST_MODE = True` for safe testing first (disables motors, just runs camera/AI)
   - Optional: Set `TEST_MODE = False` for actual flight (take-off in 3 seconds!)

3. **Run**:
   ```bash
   python flight_runner.py
   ```

4. **First Run Tips**:
   - Start in a large, open space
   - Keep a clear takeoff area
   - Have the kill switch (land command) ready
   - The drone will take off automatically if `TEST_MODE = False`

### Autonomous Flight Mode

**TEST_MODE (Safe)**: Motors disabled, AI planning only:
```python
USE_SIM = False
TEST_MODE = True  # No motors!
```
- Press `a` to toggle autonomous mode
- Drone calculates and displays paths on map
- Perfect for tuning before actual flight

**Flight Mode (Real Autonomous Flight)**:
```python
USE_SIM = False
TEST_MODE = False  # ⚠️ Drone will take off!
```
- Drone takes off automatically (3-second countdown)
- Press `a` to enable autonomous exploration
- Drone will:
  1. Identify unmapped areas (frontiers)
  2. Plan collision-free paths using A*
  3. Navigate to frontiers autonomously
  4. Map new regions
  5. Continue until no frontiers remain
- Press `q` anytime to land (or KeyboardInterrupt to end quickly)

**How It Works**:
- State Machine: `exploring` → `planning` → `moving` → repeat
- If stuck on waypoint: restarts exploration
- If obstacle ahead: hovers and replans
- Mission complete: lands automatically

See [Flight Control Guide](./FLIGHT_CONTROL.md) for detailed technical documentation.

### Configuration

Edit the top of `flight_runner.py`:

```python
USE_SIM = True              # False = Use real Tello drone
TEST_MODE = False           # True = Motors off (camera + AI only)
DEPTH_EVERY_N = 3           # Process depth every N frames (3 = ~10 FPS inference)
DEPTH_DOWNSAMPLE = 4        # Downsample depth grid by this factor (speedup)
CONFIG_FILE = "depth_config.txt"  # Save/load depth scale calibration
DEPTH_SCALE = 5.0           # Multiplier for relative depth → meters
```

- **DEPTH_SCALE**: This is the most critical tuning parameter. If obstacles look too close/far in the map, adjust this interactively (press `[` or `]` while running, check [Preflight Checklist](./PREFLIGHT-CHECKLIST.md) for turning).

## Code Quality & Validation

### Known Issues

1. **Alternative Scripts Not Integrated**
   - `camera.py`: Standalone YOLOv3 demo; not used by main system
   - `lilboi.py`: Simple mapping demo; can be run separately for reference
   - `test_simple.py`: Model testing utility; requires manual image paths

### Verified Working

- Lite-Mono model loading and inference  
- Depth → occupancy grid conversion  
- A* path planning algorithm  
- Tello drone API integration (if djitellopy installed)  
- Webcam simulation mode  
- Video I/O and visualization  
- Automatic flight and navigation

## Troubleshooting

### "ModuleNotFoundError: No module named 'torch'"
→ Reinstall: `pip install -r requirements.txt`

### "ModuleNotFoundError: No module named 'networks'" / "No module named 'layers'"
→ Ensure you cloned with `--recursive` to get submodules. If not:
```bash
git submodule update --init --recursive
```

### Depth inference is very slow (>1 second per frame)
→ GPU is likely not being used. Check PyTorch device:
```python
import torch
print(torch.cuda.is_available())  # Should be True if GPU setup correct
```
If False, install CUDA-enabled PyTorch (see GPU Setup section).

### "Failed to connect to Tello" / Drone won't respond
→ 
- Confirm you're connected to Tello's Wi-Fi (not your home network!)
- Restart the drone (power off/on)
- Check battery level (`djitellopy` will print this)
- Try the simple `camera.py` demo to verify SDK is working

### Depth map is all zeros or noisy
→ 
- Check lighting (model may underperform in very dark environments)
- Try tuning `DEPTH_SCALE` (adjust with `[` and `]` keys)
- Verify weights are loaded: check `weights/lite-mono-small-640x192/encoder.pth` exists

### Camera window shows frozen frames
→ 
- In simulation mode: check webcam is accessible (`cv2.VideoCapture(0)`)
- In drone mode: Tello may have disconnected; try restarting

## Autonomous Flight Architecture

### Flight States

The `FlightController` manages a state machine for autonomous exploration:

```
exploring → planning → moving → (repeat or hover on obstacle) → landing
```

- **Exploring**: Scans occupancy grid for frontier cells (boundary between known/unknown)
- **Planning**: Uses A* pathfinding to compute collision-free route to frontier
- **Moving**: Follows waypoints, sending proportional RC control commands
- **Hovering**: Pauses if obstacle detected ahead; replans when clear
- **Landing**: Graceful descent (manual or emergency)

### Key Components

1. **FlightController Class** (`flight_runner.py`): Main state machine
2. **Frontier Detection** (`planner.py`): Identifies unexplored regions
3. **Path Planning** (`planner.py`): A* algorithm on occupancy grid
4. **Waypoint Following** (`flight_runner.py`): RC control to drone

### Movement Model

Tello RC control ranges:
- Left/Right: -100 (right) to +100 (left)
- Forward/Back: -100 (back) to +100 (forward)  
- Up/Down: -100 (down) to +100 (up)
- Yaw: -100 (CCW) to +100 (CW)

Current implementation uses **speed=30** with proportional control based on waypoint offset.

### Customizing Autonomous Behavior

Edit `FlightController` class in `flight_runner.py`:

```python
# Frontier search range (in grid cells, 0.1m resolution)
min_distance = 5      # 0.5m minimum
max_distance = 50     # 5.0m maximum

# Movement
speed = 30            # 1-100 Tello scale

# Obstacle detection
check_distance = 5    # cells to look ahead

# Stuck detection
stuck_threshold = 30  # frames (~1 second at 30 FPS)
```

Modify `_move_to_waypoint()` for different movement strategies:
- Current: proportional to offset (simple)
- Could add: P/PID control, orientation alignment, curved paths

## Example: Advanced Autonomous Mission

```python
# In flight_runner.py's main loop, after flight controller update:
if auto_flight and flight_controller:
    # Log exploration progress
    drone_cell = mapper.get_drone_cell()
    explored_percent = (mapper.grid < 0.3).sum() / mapper.grid.size * 100
    print(f"Drone at {drone_cell}, Explored: {explored_percent:.1f}%")
    
    # Optional: stop after exploring 80% of map
    if explored_percent > 80:
        auto_flight = False
        drone.land()
        print("Mission complete!")
```

## References

- **Lite-Mono**: https://github.com/noahzn/Lite-Mono
- **DJI Tello SDK**: https://github.com/damiafuentes/DJITelloPy
- **OpenCV**: https://docs.opencv.org/
- **PyTorch**: https://pytorch.org/

## License

See LICENSE file in repository.

Note that by using this software, you agree to the terms and conditions outlined in the LICENSE file. License is BSD-3. When you use this software, you are agreeing to abide by these terms, including liability for any damages caused by your use of the software.

YOLOOOOOOOOOO (yolov3 reference)
