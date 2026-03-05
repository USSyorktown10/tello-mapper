from djitellopy import Tello
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

LOWER_COLOR_BOUND = (0, 0, 100)
UPPER_COLOR_BOUND = (100, 100, 255) 
MIN_AREA_THRESHOLD = 5000 
GRID_SIZE = 100
CELL_SIZE_CM = 20
grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)  # occupancy grid
drone_pos = [GRID_SIZE//2, GRID_SIZE//2]
drone_yaw = 0

tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%")
tello.streamon()

def draw_map():
    plt.clf()
    plt.imshow(grid, cmap="gray", vmin=-1, vmax=1)
    plt.scatter(drone_pos[0], drone_pos[1], c="blue", label="Drone")
    plt.title("Occupancy Map")
    plt.legend()
    plt.pause(0.01)

def update_drone_pose(delta_x, delta_y, delta_yaw=0):
    drone_pos[0] += delta_x
    drone_pos[1] += delta_y
    global drone_yaw
    drone_yaw = (drone_yaw + delta_yaw) % 360

def map_walls(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_COLOR_BOUND, UPPER_COLOR_BOUND)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > MIN_AREA_THRESHOLD:
            x, y, w, h = cv2.boundingRect(c)
            wall_x = drone_pos[0] + int(w / CELL_SIZE_CM)
            wall_y = drone_pos[1] + int(h / CELL_SIZE_CM)
            if 0 <= wall_x < GRID_SIZE and 0 <= wall_y < GRID_SIZE:
                grid[wall_x, wall_y] = -1
    grid[drone_pos[0], drone_pos[1]] = 1  # mark current as free space

plt.ion()
tello.takeoff()
try:
    while True:
        frame = tello.get_frame_read().frame
        map_walls(frame)
        draw_map()

        # Example: try moving forward if grid ahead is not a wall, else rotate and try left
        next_cell = [drone_pos[0], drone_pos[1] + 2]  # forward step
        if 0 <= next_cell[0] < GRID_SIZE and 0 <= next_cell[1] < GRID_SIZE and grid[next_cell[0], next_cell[1]] != -1:
            tello.forward(40)  # move forward
            update_drone_pose(0, 2)
        else:
            tello.rotate_counter_clockwise(90)
            update_drone_pose(0, 0, 90)

        cv2.imshow("Drone Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(2)  # give time for each command

finally:
    tello.land()
    tello.streamoff()
    cv2.destroyAllWindows()
