import heapq
import numpy as np


def astar(grid, start, goal, allow_diagonal=False):
    """
    A* on a binary occupancy grid where 0=free, 255=occupied.
    start and goal are (x,y) cell tuples.
    Returns list of cells from start to goal or empty list if no path.
    """
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])

    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
    if allow_diagonal:
        neighbors += [(-1,-1),(-1,1),(1,-1),(1,1)]

    rows, cols = grid.shape
    closed = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: h(start, goal)}
    open_heap = [(fscore[start], start)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        closed.add(current)

        for dx, dy in neighbors:
            neigh = (current[0] + dx, current[1] + dy)
            if not (0 <= neigh[0] < cols and 0 <= neigh[1] < rows):
                continue
            if grid[neigh[1], neigh[0]] == 255:
                continue
            if neigh in closed:
                continue

            tentative_g = gscore[current] + (1.4 if dx and dy else 1.0)
            if tentative_g < gscore.get(neigh, float('inf')):
                came_from[neigh] = current
                gscore[neigh] = tentative_g
                fscore[neigh] = tentative_g + h(neigh, goal)
                heapq.heappush(open_heap, (fscore[neigh], neigh))

    return []


def find_frontier(grid, origin_cell, min_distance=5, max_distance=None):
    """
    Find a frontier cell (boundary between known free and unknown areas) to explore.
    Grid convention: 0.0 = free, 1.0 = occupied, 0.5 = unknown.
    
    Args:
        grid: Occupancy grid (0.0=free, 1.0=occupied, 0.5=unknown)
        origin_cell: (x, y) current position in grid
        min_distance: Minimum distance from origin to consider
        max_distance: Maximum distance from origin (None=unlimited)
    
    Returns:
        (x, y) frontier cell coordinates or None if none found
    """
    rows, cols = grid.shape
    sx, sy = origin_cell
    best = None
    best_dist = float('inf')
    
    for y in range(rows):
        for x in range(cols):
            # Look for free cells
            if grid[y, x] < 0.3:  # Free cell
                # Check if any neighbor is unknown (0.4-0.6 range)
                neigh_unknown = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < cols and 0 <= ny < rows:
                        if 0.4 <= grid[ny, nx] <= 0.6:
                            neigh_unknown = True
                            break
                
                if neigh_unknown:
                    # This is a frontier cell (free next to unknown)
                    dist = abs(x - sx) + abs(y - sy)
                    if dist >= min_distance and (max_distance is None or dist <= max_distance):
                        # Prefer nearest frontier for faster exploration
                        if dist < best_dist:
                            best = (x, y)
                            best_dist = dist
    
    return best


def find_all_frontiers(grid, origin_cell, max_distance=None, limit=5):
    """
    Find multiple frontier cells for exploration options.
    Returns list of (x, y, distance) tuples sorted by distance.
    """
    rows, cols = grid.shape
    sx, sy = origin_cell
    frontiers = []
    
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] < 0.3:
                neigh_unknown = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < cols and 0 <= ny < rows:
                        if 0.4 <= grid[ny, nx] <= 0.6:
                            neigh_unknown = True
                            break
                
                if neigh_unknown:
                    dist = abs(x - sx) + abs(y - sy)
                    if max_distance is None or dist <= max_distance:
                        frontiers.append((x, y, dist))
    
    # Sort by distance (nearest first) and return top N
    frontiers.sort(key=lambda f: f[2])
    return frontiers[:limit]


def plan_path_to_target(grid, start, target, allow_diagonal=False):
    """
    Plan a path from start to target, handling obstacles.
    
    Args:
        grid: Occupancy grid (0.0-1.0)
        start: (x, y) start position
        target: (x, y) target position
        allow_diagonal: Whether to allow diagonal movement
    
    Returns:
        List of (x, y) waypoints from start to target, or empty list if no path
    """
    # Convert to binary grid for A*: <0.5 is free, >=0.5 is obstacle
    binary_grid = np.where(grid < 0.5, 0, 255).astype(np.uint8)
    return astar(binary_grid, start, target, allow_diagonal=allow_diagonal)
