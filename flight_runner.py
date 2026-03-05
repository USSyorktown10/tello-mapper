import sys
import cv2
import time
import threading
import numpy as np
from mapper import Mapper
from planner import find_frontier, plan_path_to_target, find_all_frontiers
import torch
import os

USE_SIM = True          # False = Real Tello
TEST_MODE = False         # True = Camera ON, Motors OFF (Good for tuning)

DEPTH_EVERY_N = 3       
DEPTH_DOWNSAMPLE = 4
CONFIG_FILE = "depth_config.txt"

# Load saved scale or default to 5.0
if os.path.exists(CONFIG_FILE):
    with open(CONFIG_FILE, "r") as f:
        try:
            DEPTH_SCALE = float(f.read().strip())
            print(f"Loaded saved scale: {DEPTH_SCALE}")
        except:
            DEPTH_SCALE = 5.0
else:
    DEPTH_SCALE = 5.0

torch.set_num_threads(2)
ROOT = os.path.dirname(os.path.abspath(__file__))
LITEMONO_DIR = os.path.join(ROOT, "Lite-Mono")
if LITEMONO_DIR not in sys.path:
    sys.path.insert(0, LITEMONO_DIR)

import networks
from layers import disp_to_depth

# vid reader
class BackgroundFrameRead:
    """
    Reads frames from Tello UDP stream using OpenCV directly.
    This bypasses the 'av' library crash issues.
    """
    def __init__(self, source="udp://@0.0.0.0:11111"):
        self.cap = cv2.VideoCapture(source)
        self.frame = None
        self.stopped = False
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        
    def start(self):
        self.thread.start()
        return self

    def update(self):
        while not self.stopped:
            if not self.cap.isOpened():
                time.sleep(0.1)
                continue
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
            else:
                time.sleep(0.01) # preventing busy wait on failure

    def stop(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()

class DummyDrone:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
    def takeoff(self): pass
    def land(self): pass
    def send_rc_control(self, lr, fb, ud, yv): pass
    def get_height(self): return 0
    def get_battery(self): return 100


class FlightController:
    """Manages autonomous flight: planning, obstacle avoidance, waypoint following."""
    
    def __init__(self, drone, mapper, planner_module):
        self.drone = drone
        self.mapper = mapper
        self.planner = planner_module
        self.state = "exploring"  # exploring, planning, moving, hovering, landing
        self.target_frontier = None
        self.current_path = []
        self.path_index = 0
        self.stuck_counter = 0
        self.last_pos = (0, 0)
        
    def update(self, occupancy_grid):
        """Main update loop for autonomous flight."""
        drone_cell = self.mapper.get_drone_cell()
        
        if self.state == "exploring":
            # Find nearest frontier
            self.target_frontier = self.planner.find_frontier(
                occupancy_grid, drone_cell, min_distance=5, max_distance=50
            )
            if self.target_frontier:
                self.state = "planning"
            else:
                # No frontier found - exploration complete or trapped
                print("No frontiers found. Exploration complete or area fully mapped.")
                self.state = "landing"
        
        elif self.state == "planning":
            # Plan path to frontier
            if self.target_frontier:
                self.current_path = self.planner.plan_path_to_target(
                    occupancy_grid, drone_cell, self.target_frontier
                )
                if len(self.current_path) > 1:
                    self.path_index = 0
                    self.state = "moving"
                    print(f"Planned path with {len(self.current_path)} waypoints")
                else:
                    # Can't reach frontier, try another
                    print("No path to frontier, exploring again")
                    self.state = "exploring"
        
        elif self.state == "moving":
            # Follow waypoint path
            if self.current_path and self.path_index < len(self.current_path):
                target_cell = self.current_path[self.path_index]
                self._move_to_waypoint(target_cell)
                
                # Check if reached waypoint (within 2 cells)
                dist = abs(drone_cell[0] - target_cell[0]) + abs(drone_cell[1] - target_cell[1])
                if dist < 2:
                    self.path_index += 1
                    self.stuck_counter = 0
                else:
                    self.stuck_counter += 1
                    if self.stuck_counter > 30:  # Stuck for ~1 second
                        print("Stuck on path, replanning...")
                        self.state = "exploring"
            else:
                # Path complete, return to exploring
                self.state = "exploring"
        
        elif self.state == "hovering":
            # Check if obstacle cleared
            self.drone.send_rc_control(0, 0, 0, 0)  # Hover
            if not self._check_obstacle_ahead(occupancy_grid, drone_cell):
                self.state = "moving"
        
        elif self.state == "landing":
            self.drone.land()
    
    def _move_to_waypoint(self, target_cell):
        """Send movement commands to reach target cell."""
        drone_cell = self.mapper.get_drone_cell()
        dx = target_cell[0] - drone_cell[0]
        dy = target_cell[1] - drone_cell[1]
        
        # Convert cell difference to movement commands
        # Tello RC control: lr (-100:right, 100:left), fb (-100:back, 100:forward)
        #                   ud (-100:down, 100:up), yv (-100:ccw, 100:cw)
        
        # Simple proportional control
        speed = 30  # 1-100 range
        
        if abs(dx) > abs(dy):
            # Move left/right
            fb = 0
            lr = speed if dx > 0 else -speed
        else:
            # Move forward/backward
            lr = 0
            fb = speed if dy > 0 else -speed
        
        # Maintain altitude
        ud = 0
        
        # Simple yaw (rotation to face direction)
        yv = 0
        
        if not TEST_MODE:
            self.drone.send_rc_control(lr, fb, ud, yv)
    
    def _check_obstacle_ahead(self, occupancy_grid, drone_cell, check_distance=5):
        """Check if obstacle is ahead of drone."""
        cx, cy = drone_cell
        
        # Check in front of drone (simple forward check)
        for d in range(1, check_distance):
            check_cell = (cx, cy + d)
            if (0 <= check_cell[0] < occupancy_grid.shape[1] and 
                0 <= check_cell[1] < occupancy_grid.shape[0]):
                if occupancy_grid[check_cell[1], check_cell[0]] > 0.7:  # Occupied
                    return True
        return False

def draw_hud(img, center_dist, scale, mode_msg, flight_state=""):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    # Crosshair
    cv2.line(img, (cx - 10, cy), (cx + 10, cy), (0, 255, 0), 1)
    cv2.line(img, (cx, cy - 10), (cx, cy + 10), (0, 255, 0), 1)
    # Text
    dist_txt = f"Dist: {center_dist:.2f} m" if center_dist else "Dist: --"
    cv2.putText(img, dist_txt, (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    # Info Bar
    status = "TEST MODE (No Fly)" if TEST_MODE else "FLYING"
    flight_txt = f" | State: {flight_state}" if flight_state else ""
    cv2.rectangle(img, (0, 0), (w, 40), (0, 0, 0), -1)
    cv2.putText(img, f"Scale: {scale:.1f} ([/]) | {status}{flight_txt}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


def main():
    global DEPTH_SCALE
    mapper = Mapper(size_m=8.0, resolution=0.1)
    
    cap = None          # For Webcam
    frame_reader = None # For Tello

    # Setup
    if USE_SIM:
        drone = DummyDrone()
        cap = cv2.VideoCapture(0)
    else:
        from djitellopy import Tello
        drone = Tello()
        try:
            drone.connect()
            drone.streamon()
            print(f"Battery: {drone.get_battery()}%")
            
            # Use Custom Reader instead of drone.get_frame_read()
            print("Starting Custom OpenCV Video Thread...")
            frame_reader = BackgroundFrameRead().start()
            time.sleep(2) # Warmup
        except Exception as e:
            print(f"Connection Error: {e}")
            return

    # Saftey
    if not TEST_MODE and not USE_SIM:
        print("Taking off in 3 seconds...")
        time.sleep(3)
        drone.takeoff()
    else:
        print("TEST MODE: Motors disabled. Running camera & AI only.")

    # Check for GPU
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    weights_folder = os.path.join(ROOT, "weights", "lite-mono-small-640x192")
    encoder_path = os.path.join(weights_folder, "encoder.pth")
    decoder_path = os.path.join(weights_folder, "depth.pth")

    encoder = networks.LiteMono(model="lite-mono-small", height=96, width=320)
    encoder.load_state_dict({k: v for k, v in torch.load(encoder_path, map_location=device).items() if k in encoder.state_dict()})
    encoder.to(device).eval()

    depth_decoder = networks.DepthDecoder(encoder.num_ch_enc, scales=range(3))
    depth_decoder.load_state_dict({k: v for k, v in torch.load(decoder_path, map_location=device).items() if k in depth_decoder.state_dict()})
    depth_decoder.to(device).eval()

    # Main
    running = True
    frame_count = 0
    prev_depth = None
    
    # Initialize flight controller (will be None if USE_SIM)
    flight_controller = None
    auto_flight = False  # Toggle with 'a' key

    try:
        while running:
            # Get Frame
            frame = None
            if USE_SIM:
                ret, frame = cap.read()
                if not ret: continue
            else:
                if frame_reader is None or frame_reader.frame is None:
                    time.sleep(0.01)
                    continue
                frame = frame_reader.frame.copy()

            # Crop & Resize (Correcting Aspect Ratio)
            orig_h, orig_w = frame.shape[:2]
            target_aspect = 320 / 96
            crop_h = int(orig_w / target_aspect)
            cy = orig_h // 2
            crop_frame = frame[max(0, cy - crop_h // 2):min(orig_h, cy + crop_h // 2), 0:orig_w]
            
            input_img = cv2.resize(crop_frame, (320, 96))
            input_tensor = torch.from_numpy(cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)).float() / 255.0
            input_tensor = input_tensor.permute(2, 0, 1).unsqueeze(0).to(device)

            frame_count += 1
            center_dist_m = None

            # Inference
            if frame_count % DEPTH_EVERY_N == 0:
                with torch.no_grad():
                    features = encoder(input_tensor)
                    outputs = depth_decoder(features)
                    disp = outputs[("disp", 0)]
                    _, depth_out = disp_to_depth(disp, 0.1, 100.0)
                    depth_npy = depth_out.cpu().squeeze().numpy()
                    depth_npy = cv2.medianBlur(depth_npy, 5)
                    
                    prev_depth = depth_npy * DEPTH_SCALE # Apply Scale
            
            depth = prev_depth

            if depth is not None:
                mapper.process_depth_map(depth, downsample=DEPTH_DOWNSAMPLE)
                
                # Get Center Distance for HUD
                h_d, w_d = depth.shape
                mid_patch = depth[h_d//2-2:h_d//2+2, w_d//2-2:w_d//2+2]
                if mid_patch.size > 0:
                    center_dist_m = np.median(mid_patch)

            # Visualization
            if depth is not None:
                d_vis = np.clip(depth, 0.0, 5.0)
                d_norm = (d_vis / 5.0 * 255).astype(np.uint8)
                d_color = cv2.applyColorMap(d_norm, cv2.COLORMAP_MAGMA)
                cv2.imshow('Depth', d_color)

            map_vis = mapper.get_grid_for_viz()
            cv2.imshow('Map', cv2.resize(map_vis, (400, 400), interpolation=cv2.INTER_NEAREST))

            flight_state_str = flight_controller.state if flight_controller and auto_flight else ""
            draw_hud(frame, center_dist_m, DEPTH_SCALE, "TEST" if TEST_MODE else "FLY", flight_state_str)
            cv2.imshow('Camera', frame)

            # nputs (Scale Tuning + Flight Control)
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                running = False
            elif key == ord(']'):
                DEPTH_SCALE += 0.5
            elif key == ord('['):
                DEPTH_SCALE = max(0.5, DEPTH_SCALE - 0.5)
            elif key == ord('a'):
                # Toggle autonomous flight
                if not USE_SIM and not TEST_MODE:
                    auto_flight = not auto_flight
                    if auto_flight and flight_controller is None:
                        # Initialize flight controller on first activation
                        flight_controller = FlightController(drone, mapper, sys.modules[__name__])
                        print("Autonomous flight ENABLED")
                    print(f"Auto flight: {'ON' if auto_flight else 'OFF'}")

            # Autonomous Flight Control
            if auto_flight and flight_controller and not USE_SIM and not TEST_MODE:
                try:
                    flight_controller.update(mapper.grid)
                except Exception as e:
                    print(f"Flight control error: {e}")
                    auto_flight = False
                    drone.land()

    finally:
        # Save the settings
        with open(CONFIG_FILE, "w") as f:
            f.write(str(DEPTH_SCALE))
        print(f"Scale {DEPTH_SCALE} saved to {CONFIG_FILE}.")

        if frame_reader:
            frame_reader.stop()
        
        if not USE_SIM and not TEST_MODE:
            try:
                drone.land()
            except: pass
            try:
                drone.streamoff()
            except: pass
            
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()