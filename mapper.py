import numpy as np
import cv2

class Mapper:
    def __init__(self, size_m=10.0, resolution=0.1):
        self.resolution = resolution
        self.size_m = size_m
        self.cells = int(size_m / resolution)
        # Grid: 0.5 = unknown, 0.0 = free, 1.0 = occupied
        self.grid = np.full((self.cells, self.cells), 0.5, dtype=np.float32)
        self.origin = (self.cells // 2, self.cells // 2)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cell_to_world(self, cx, cy):
        """Convert cell coordinates to world coordinates (meters)."""
        wx = (cx - self.origin[0]) * self.resolution
        wy = (self.origin[1] - cy) * self.resolution
        return wx, wy
    
    def get_drone_cell(self):
        """Get drone's current position in grid cell coordinates."""
        return self.world_to_cell(self.x, self.y)
    
    def update_pose(self, x, y, theta):
        """Update drone's pose (world coordinates, meters; theta in radians)."""
        self.x = x
        self.y = y
        self.theta = theta

    def mark_occupied(self, wx, wy):
        cx, cy = self.world_to_cell(wx, wy)
        if 0 <= cx < self.cells and 0 <= cy < self.cells:
            # Increase probability of occupancy
            self.grid[cy, cx] = min(0.95, self.grid[cy, cx] + 0.15)

    def mark_free_line(self, wx1, wy1, wx2, wy2):
        c1 = self.world_to_cell(wx1, wy1)
        c2 = self.world_to_cell(wx2, wy2)
        x1, y1 = c1
        x2, y2 = c2

        # Bresenham's Line Algo to clear path
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        for (px, py) in points:
            if 0 <= px < self.cells and 0 <= py < self.cells:
                # Decrease probability (mark as free)
                self.grid[py, px] = max(0.05, self.grid[py, px] - 0.1)

    def process_depth_map(self, depth_map, camera_fov_deg=80.0, max_range_m=4.0, downsample=4):
        """
        depth_map: HxW array of depths in meters.
        """
        h, w = depth_map.shape
        cx_img = w / 2.0
        
        # HEURISTIC: Ignore Top 15% (Ceiling/Lights) and Bottom 30% (Floor)
        # This prevents mapping the ground as a wall.
        y_start = int(h * 0.15)
        y_end = int(h * 0.70) 

        for yy in range(y_start, y_end, downsample):
            for xx in range(0, w, downsample):
                d = depth_map[yy, xx]
                
                # Filter invalid or too-far depths
                if d <= 0.1 or d > max_range_m:
                    continue

                # Calculate Bearing
                # We map x-pixel to angle. 
                # (xx - cx_img) gives pixels from center.
                bearing_deg = (xx - cx_img) / cx_img * (camera_fov_deg / 2.0)
                bearing_rad = np.deg2rad(bearing_deg) + self.theta

                # Project point
                ox = self.x + d * np.cos(bearing_rad)
                oy = self.y + d * np.sin(bearing_rad)

                # Raycast: Clear space between drone and object, mark object
                self.mark_free_line(self.x, self.y, ox, oy)
                self.mark_occupied(ox, oy)

    def get_grid_for_viz(self):
        # Threshold: >0.6 is occupied (Black), <0.6 is free (White)
        # Unknown (0.5) will be greyish or treated as free here for clarity
        vis = np.zeros((self.cells, self.cells), dtype=np.uint8)
        
        # 0 = Obstacle (Black), 255 = Free (White)
        # Scale 0.0-1.0 to 0-255 inverted
        vis = (255 * (1.0 - self.grid)).astype(np.uint8)
        return vis