from controller import Robot
import math
from gridNode import GridNode
from checkpoint import Checkpoint
import heapq

def run_agent():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')

    left.setPosition(float('inf'))
    right.setPosition(float('inf'))

    left.setVelocity(0.0)
    right.setVelocity(0.0)

    gps = robot.getDevice('gps');       gps.enable(timestep)
    compass = robot.getDevice('compass'); compass.enable(timestep)

    camera = robot.getDevice('camera'); camera.enable(timestep)  # left enabled as before

    dist_sensors = [robot.getDevice(f'ps{k}') for k in range(8)]
    for s in dist_sensors:
        s.enable(timestep)

    #Initial Positions
    x0 = None
    y0 = None

    # Map relative directions to absolute directions
    REL_TO_ABS = {
    "N": {"front": "N", "left": "W", "right": "E", "back": "S"},
    "E": {"front": "E", "left": "N", "right": "S", "back": "W"},
    "S": {"front": "S", "left": "E", "right": "W", "back": "N"},
    "W": {"front": "W", "left": "S", "right": "N", "back": "E"},
    }

    DIRECTION_OFFSETS = {
    "N": (1, 0),
    "E": (0, -1),
    "S": (-1, 0),
    "W": (0, 1),
    }

    IDEAL_YAWS = {
        "N": 0.0,
        "E": -math.pi / 2,
        "S": math.pi,
        "W": math.pi / 2,
    }

    #Initializing Dictionary to hold grid nodes
    grid_nodes = {}

    #Initializing set to hold cell to comeback mapping
    cell_to_comeback = dict()

    #Initializing dictionary for mapping
    direction_mapping = {}

    #constants
    CENTER_TOL  = 0.015  # Tolerance to consider the robot at the center of a cell (per assignment spec)
    SENSOR_THRESHOLD = 100.0  # Threshold to consider an obstacle detected by distance sensors
    MAX_SPEED   = 6.28
    CELL = 0.15
    ALIGN_TOL_DEG = 0.5      # stop rotating when within this tolerance
    SETTLE_STEPS  = 6        # require stability N steps in a row
    Kp_turn       = 2.0      # rotation proportional gain (increased for visible rotation)
    MANAHATTAN_THRESHOLD = 20  # Threshold distance for Manhattan distance to consider cells "close"

    # Intializing next cell
    next_cell = None

    #Initializing direction
    direction = None

    #Modes
    mode = "exploration"  # default mode

    #A* variables
    open_set = []       # Will store f_score, g_score, node)
    planned_path = []   # list of grid cells [(i,j), (i2,j2), ...] (excluding current cell)
    came_from = {}      #parent path reconstruction
    g_score = {}      # cost from start to node
    f_score = {}        # estimated cost from start to

    #Helper functions
    #A*
    def movement_cost(current, neighbor):
        return 1  # Each grid step costs 1
    
    def heuristic(node, goal):
        # Manhattan distance for 4-connected grid
        print("Heuristic calculation between", node, "and", goal, "is", abs(node[0] - goal[0]) + abs(node[1] - goal[1]))
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    
    def reconstruct_path(came_from_map, current):
        path = [current]
        while current in came_from_map:
            current = came_from_map[current]
            path.append(current)
        path.reverse()
        return path

    def astar(start, goal, get_neighbors_func):
        if start == goal:
            return [start]

        open_set.clear()
        came_from.clear()
        g_score.clear()
        f_score.clear()

        g_score[start] = 0
        f_score[start] = heuristic(start, goal)
        heapq.heappush(open_set, (f_score[start], 0, start))

        closed = set()

        while open_set:

            current_f, current_g, current = heapq.heappop(open_set)

            # Skip outdated entries (stale g that is worse than the best we know)
            if current_g != g_score.get(current, float('inf')):
                continue

            # Now it's safe to "close" the node
            closed.add(current)

            if current == goal:
                return reconstruct_path(came_from, current)

            for neighbor in get_neighbors_func(current):
                if neighbor in closed:
                    continue
                tentative_g = current_g + movement_cost(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    estimated_f = tentative_g + heuristic(neighbor, goal)
                    f_score[neighbor] = estimated_f
                    heapq.heappush(open_set, (estimated_f, tentative_g, neighbor))

        return None
    
    def get_free_neighbors(cell):
        return grid_nodes[cell].neighbors if cell in grid_nodes else []
    
    #Determine next cell helpers
    def find_center_cell(cells):
        if not cells:
            return None
        avg_i = sum(i for i, j in cells) / len(cells)
        avg_j = sum(j for i, j in cells) / len(cells)
        print("Center cell calculated as:", (round(avg_i), round(avg_j)))
        return (round(avg_i), round(avg_j))

    def get_directions_info(sensor_vals, cardinal_direction):
        """
        Helper function to determine obstacle/free status in all directions
        based on sensor values and current cardinal heading.
        
        Args:
            sensor_vals: List of 8 distance sensor values
            cardinal_direction: Current cardinal direction ("N", "E", "S", "W")
            
        Returns:
            Dictionary mapping absolute directions to Checkpoint.OBSTACLE or Checkpoint.FREE
        """
        front_h = REL_TO_ABS[cardinal_direction]["front"]
        right_h = REL_TO_ABS[cardinal_direction]["right"]
        left_h  = REL_TO_ABS[cardinal_direction]["left"]
        back_h  = REL_TO_ABS[cardinal_direction]["back"]

        directions_info = {}
        #print("Front heading:", sensor_vals[0] + sensor_vals[7])
        directions_info[front_h] = Checkpoint.OBSTACLE if (sensor_vals[0] > SENSOR_THRESHOLD or sensor_vals[7] > SENSOR_THRESHOLD or (sensor_vals[0] + sensor_vals[7] > 145)) else Checkpoint.FREE
        directions_info[right_h] = Checkpoint.OBSTACLE if sensor_vals[2] > SENSOR_THRESHOLD else Checkpoint.FREE
        directions_info[left_h] = Checkpoint.OBSTACLE if sensor_vals[5] > SENSOR_THRESHOLD else Checkpoint.FREE
        directions_info[back_h] = Checkpoint.OBSTACLE if (sensor_vals[4] > SENSOR_THRESHOLD or sensor_vals[3] > SENSOR_THRESHOLD) else Checkpoint.FREE
        
        return directions_info

    #movement
    def cruising_speed(left_speed, right_speed):
        left.setVelocity(left_speed)
        right.setVelocity(right_speed)

    def angle_diff_rad(a, b):
        # smallest signed difference a - b in [-pi, pi]
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def rotate_towards(target_dir: str):
        """Stop and rotate in place at the center until heading matches target_dir."""
        target_yaw = IDEAL_YAWS[target_dir]
        max_v = min(left.getMaxVelocity(), right.getMaxVelocity(), MAX_SPEED)
        aligned_count = 0

        # hard stop before rotation
        left.setVelocity(0.0)
        right.setVelocity(0.0)

        while robot.step(timestep) != -1:
            nx, ny, _ = compass.getValues()
            yaw = math.atan2(nx, ny)
            err = angle_diff_rad(target_yaw, yaw)
            err_deg = abs(err * 180.0 / math.pi)

            if err_deg <= ALIGN_TOL_DEG:
                aligned_count += 1
                if aligned_count >= SETTLE_STEPS:
                    left.setVelocity(0.0)
                    right.setVelocity(0.0)
                    return
            else:
                aligned_count = 0

            # in-place rotation: +err -> CCW (left backward, right forward)
            omega = Kp_turn * err
            vL, vR = -omega, omega

            # clamp
            m = max(abs(vL), abs(vR))
            if m > max_v:
                s = max_v / m
                vL *= s; vR *= s

            left.setVelocity(vL)
            right.setVelocity(vR)


    while robot.step(timestep) != -1:

        #Initialize Direction-Cell Mapping
        directions_info = {}
        
        vals = [s.getValue() for s in dist_sensors]

        #Get Values
        gps_values = gps.getValues()
        north = compass.getValues()

        # Compute heading
        heading = math.atan2(north[0], north[1])
        heading_deg = (heading * 180.0 / math.pi + 360) % 360
        cardinal = heading_to_cardinal(heading_deg)
        

        # Compute initial position
        x0 = gps_values[0] if x0 is None else x0
        y0 = gps_values[1] if y0 is None else y0

        # Compute current position
        x_curr = gps_values[0]
        y_curr = gps_values[1]

        i = round((x_curr - x0) / 0.15)
        j = round((y_curr - y0) / 0.15)

        # Get existing node or create a new one
        if (i, j) in grid_nodes:
            current_node = grid_nodes[(i, j)]
            current_node.world_pos = (x_curr, y_curr)  # Update position
        else:
            current_node = GridNode((i, j), (x_curr, y_curr))
        current_node.mark_visited()

        #Check if robot is  at cell center
        at_center = math.hypot(x_curr - (i * 0.15 + x0), y_curr - (j * 0.15 + y0)) <= CENTER_TOL

        #Initialize unvisited cells count 
        unvisited_count = 0

        # Update directions info if at center
        if at_center and current_node.neighbors == []:
            directions_info = get_directions_info(vals, cardinal)

            # Update the direction mapping
            direction_mapping[(i, j)] = directions_info


            #print("Distance sensor values:", vals)
            #print(f"At cell ({i}, {j}), direction mapping: {direction_mapping[(i, j)]}")

            #Determine unvisited neighbors
            for direction, status in direction_mapping.get((i, j), {}).items():
                if status == Checkpoint.FREE:
                    (dx, dy) = DIRECTION_OFFSETS[direction]
                    neighbor_coord = (i + dx, j + dy)
                    if (neighbor_coord not in grid_nodes) or (current_node.grid_coord in grid_nodes[neighbor_coord].neighbors):
                        current_node.add_neighbor(neighbor_coord)
                    if neighbor_coord not in grid_nodes:
                        unvisited_count += 1
                        

        if (i,j) not in grid_nodes:
            grid_nodes[(i, j)] = current_node

        #update cells to comeback to
        if unvisited_count >1:
            cell_to_comeback[(i, j)] = unvisited_count -1

        #Find the  next cell to visit
        # Check all neighbors to find an unvisited one, not just the first
        unvisited_neighbor = []
        for neighbor in current_node.neighbors:
            if neighbor not in grid_nodes and neighbor not in unvisited_neighbor:
                unvisited_neighbor.append(neighbor)
                break

        if at_center:   
            if not planned_path:
                if unvisited_neighbor:
                    #print("Moving to unvisited neighbor:", unvisited_neighbor)
                    if cell_to_comeback:
                        center_cell = find_center_cell(list(cell_to_comeback.keys()))
                        distance = 100
                        for cell in unvisited_neighbor:
                            dist = heuristic(center_cell, cell)
                            if dist < distance:
                                distance = dist
                                next_cell = cell
                    
                        next_cell = unvisited_neighbor[0]

                if next_cell in grid_nodes and cell_to_comeback:

                    #print("Planning path to next cell to comeback...")
                    next_cell = min(
                        cell_to_comeback.keys(),
                        key=lambda cell: heuristic(current_node.grid_coord, cell),
                    )
                    path = astar(current_node.grid_coord, next_cell, get_free_neighbors)
                    #print("The next cell to comeback is:", next_cell)
                    #n = get_free_neighbors(current_node.grid_coord)
                    #for neigh in n:
                        #print("Neighbor of current cell:", neigh)
                    
                    if path and len(path) > 1:
                        #print("Path found:", path)
                        planned_path = path[1:]  # Exclude current cell
                        cell_to_comeback[next_cell] -= 1
                        if cell_to_comeback[next_cell] <= 0:
                            del cell_to_comeback[next_cell]

                print("Next cell : ", next_cell)

            else :
                next_cell = planned_path[0]
                if (i,j) == next_cell:
                    planned_path.pop(0)

        #find the heading to next cell
        if next_cell:
            di = next_cell[0] - i
            dj = next_cell[1] - j
            for direction, (dx, dy) in DIRECTION_OFFSETS.items():
                if (di, dj) == (dx, dy):
                    direction = direction
                    break
            if (0,0) == (di,dj):
                direction = cardinal

        # Check yaws 
        ideal_yaw = IDEAL_YAWS[direction] if direction else 0.0
        current_yaw = heading
        yaw_diff = (ideal_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # --- decide action at center: rotate or move ---
        if at_center and direction and (direction != cardinal):
            # Stop & rotate in place until aligned; this function blocks until done.
            rotate_towards(direction)
        # after it returns, we are aligned; let the next loop tick set forward motion
        elif direction:
            # Not rotating: go forward (or keep your cruising logic)
            cruising_speed(0.65 * MAX_SPEED, 0.65 * MAX_SPEED)
        else:
            # No next_cell yet: just stop (or keep a gentle crawl if you prefer)
            cruising_speed(0.0, 0.0)
            
        print("Cells to comeback to:")
        for cell, count in cell_to_comeback.items():
            print(f"Cell {cell}: {count} unvisited neighbors")

        #print ("-----")  # Separator for readability
        #print("Next cell:", next_cell)

        #print the reachable neighbors from every cell in the grid
        #print("Neighbors from each cell:")
        #for cell, neighbors in grid_nodes.items():
            #print(f"Cell {cell}: {neighbors.neighbors}")

def heading_to_cardinal(heading_deg: float) -> str:
    """
    Converts a compass heading in degrees (0° = North, 90° = West)
    to a cardinal direction: N, W, S, or E.
    """
    directions = ["N", "W", "S", "E"]
    # Divide 360° into 4 equal sectors of 90° each
    index = int(((heading_deg + 45) % 360) / 90)
    return directions[index]



if __name__ == "__main__":
    run_agent()