from controller import Robot
import math
import os
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

    # Target spot color definitions (RGB values)
    TARGET_COLORS = {
        0: {"name": "Red", "rgb": (255, 0, 0)},
        1: {"name": "Blue", "rgb": (0, 0, 255)},
        2: {"name": "Yellow", "rgb": (255, 255, 0)},
        3: {"name": "Magenta", "rgb": (255, 0, 255)},
        4: {"name": "Cyan", "rgb": (0, 255, 255)},
        5: {"name": "Orange", "rgb": (255, 165, 0)},
        6: {"name": "Black", "rgb": (0, 0, 0)},
    }

    # Dictionary to store detected targets at grid positions
    detected_targets = {}
    
    # Track the actual starting cell
    starting_cell = None
    
    # Track maze coordinates (read from simulator output if available)
    maze_start_x = None  # Will be set from simulator output
    maze_start_y = None

    #Initializing Dictionary to hold grid nodes
    grid_nodes = {}

    #Initializing set to hold cell to comeback mapping
    cell_to_comeback = dict()

    #Initializing dictionary for mapping
    direction_mapping = {}

    #constants
    CENTER_TOL  = 0.015  # Tolerance to consider the robot at the center of a cell (per assignment spec)
    SENSOR_THRESHOLD = 95.0  # Threshold to consider an obstacle detected by distance sensors
    MAX_SPEED   = 6.28
    CELL = 0.15
    ALIGN_TOL_DEG = 0.5      # stop rotating when within this tolerance
    SETTLE_STEPS  = 6        # require stability N steps in a row
    Kp_turn       = 3.0      # rotation proportional gain (increased for visible rotation)
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
    #Color detection
    def rgb_distance(color1, color2):
        """Calculate Euclidean distance between two RGB colors."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(color1, color2)))
    
    def detect_target_color(camera):
        """
        Analyze camera image to detect target spot colors.
        Returns tuple: (target_id, color_name) or (None, None) if no target detected.
        """
        width = camera.getWidth()
        height = camera.getHeight()
        image = camera.getImage()
        
        if not image:
            return None, None
        
        # Sample the center region of the image (where floor would be visible)
        center_x = width // 2
        center_y = int(height * 0.75)  # Look at lower part of image (floor)
        
        # Sample a small region around the center
        sample_size = 5
        rgb_samples = []
        
        for dy in range(-sample_size, sample_size + 1):
            for dx in range(-sample_size, sample_size + 1):
                x = center_x + dx
                y = center_y + dy
                
                if 0 <= x < width and 0 <= y < height:
                    # Get pixel color (Webots camera.getImageArray() returns BGRA format)
                    pixel_index = (y * width + x) * 4
                    if pixel_index + 2 < len(image):
                        r = camera.imageGetRed(image, width, x, y)
                        g = camera.imageGetGreen(image, width, x, y)
                        b = camera.imageGetBlue(image, width, x, y)
                        rgb_samples.append((r, g, b))
        
        if not rgb_samples:
            return None, None
        
        # Calculate average color
        avg_r = sum(rgb[0] for rgb in rgb_samples) / len(rgb_samples)
        avg_g = sum(rgb[1] for rgb in rgb_samples) / len(rgb_samples)
        avg_b = sum(rgb[2] for rgb in rgb_samples) / len(rgb_samples)
        avg_color = (avg_r, avg_g, avg_b)
        
        # Check if the color is gray/neutral (normal floor)
        # Gray colors have similar R, G, B values
        max_rgb = max(avg_r, avg_g, avg_b)
        min_rgb = min(avg_r, avg_g, avg_b)
        rgb_range = max_rgb - min_rgb
        
        # If color is too gray (low range) and not very dark, it's probably normal floor
        GRAY_THRESHOLD = 40  # If RGB values are within this range, it's considered gray
        BRIGHTNESS_THRESHOLD = 50  # Minimum brightness to exclude very dark grays
        
        if rgb_range < GRAY_THRESHOLD and max_rgb > BRIGHTNESS_THRESHOLD:
            # This is likely the normal gray floor
            return None, None
        
        # Find closest matching target color (excluding black for now)
        min_distance = float('inf')
        best_match = None
        COLOR_THRESHOLD = 100  # Threshold to consider a color match valid
        
        for target_id, color_info in TARGET_COLORS.items():
            # Skip black (ID 6) since gray floor is too similar
            if target_id == 6:
                continue
                
            distance = rgb_distance(avg_color, color_info["rgb"])
            if distance < min_distance:
                min_distance = distance
                best_match = target_id
        
        # Check for black separately with stricter criteria
        # Black should be very dark with low RGB values
        if max_rgb < 30:  # Very dark
            black_distance = rgb_distance(avg_color, TARGET_COLORS[6]["rgb"])
            if black_distance < 50:  # Close to pure black
                best_match = 6
                min_distance = black_distance
        
        # Only return match if it's close enough
        if best_match is not None and min_distance < COLOR_THRESHOLD:
            print(f"[COLOR DETECTION] Detected color: {TARGET_COLORS[best_match]['name']} (ID: {best_match})")
            print(f"[COLOR DETECTION] Average RGB: ({avg_r:.1f}, {avg_g:.1f}, {avg_b:.1f}), Distance: {min_distance:.1f}")
            return best_match, TARGET_COLORS[best_match]["name"]
        
        return None, None
    
    #A*
    def movement_cost(current, neighbor):
        return 1  # Each grid step costs 1
    
    def heuristic(node, goal):
        # Manhattan distance for 4-connected grid
        #print("Heuristic calculation between", node, "and", goal, "is", abs(node[0] - goal[0]) + abs(node[1] - goal[1]))
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
    
    def solve_tsp_greedy(targets, start_pos):
        """
        Solve TSP using greedy nearest neighbor heuristic.
        Returns: (ordered list of targets, total distance, full path with intermediate cells)
        """
        if not targets:
            return [], 0, []
        
        unvisited = list(targets)
        current = start_pos
        tour = [current]
        total_distance = 0
        full_path = [current]
        
        while unvisited:
            # Find nearest unvisited target
            nearest = min(unvisited, key=lambda t: heuristic(current, t))
            
            # Find path from current to nearest
            path_segment = astar(current, nearest, get_free_neighbors)
            if path_segment:
                # Add path (excluding current position since it's already in full_path)
                full_path.extend(path_segment[1:])
                total_distance += len(path_segment) - 1
            
            tour.append(nearest)
            unvisited.remove(nearest)
            current = nearest
        
        # Add return path to start
        if current != start_pos:
            return_path = astar(current, start_pos, get_free_neighbors)
            if return_path:
                full_path.extend(return_path[1:])  # Exclude current, include start
                total_distance += len(return_path) - 1
        
        return tour, total_distance, full_path
    
    def generate_path_for_current_targets():
        """
        Generate and write path file for currently found targets.
        Called whenever a new target is detected.
        """
        if not detected_targets:
            return  # No targets yet, nothing to write
        
        # Get target positions
        target_positions = list(detected_targets.keys())
        start_position = starting_cell if starting_cell else (0, 0)
        
        # Solve TSP with current targets
        tour, total_dist, full_path = solve_tsp_greedy(target_positions, start_position)
        
        print(f"\n[DYNAMIC PATH] Generating path for {len(detected_targets)} targets found so far...")
        print(f"[DYNAMIC PATH] Path length: {total_dist} cells")
        
        # Write the path file
        write_path_file(full_path)
    
    def write_path_file(full_path, filename="agent_109061_102594.path"):
        """
        Write the path to a .path file in the required format.
        Each line contains: x y (in cell coordinates relative to start)
        First and last lines should be 0 0
        """
        print(f"\n[PATH FILE] Starting to write path file...")
        print(f"[PATH FILE] Path length: {len(full_path)}")
        print(f"[PATH FILE] Starting cell: {starting_cell}")
        
        # Try to find C3_supervisor directory
        controller_dir = os.path.dirname(__file__)
        project_root = os.path.abspath(os.path.join(controller_dir, "..", ".."))
        
        # Look for C3_supervisor directory
        candidate_dirs = [
            os.path.join(project_root, "controllers", "C3_supervisor"),
            os.path.join(project_root, "controllers", "c3_supervisor"),
            os.path.join(project_root, "supervisors", "C3_supervisor"),
            os.path.join(project_root, "supervisors", "c3_supervisor"),
        ]
        
        # Find the first existing directory, or fall back to controller directory
        output_dir = next((d for d in candidate_dirs if os.path.isdir(d)), controller_dir)
        filepath = os.path.join(output_dir, filename)
        
        # Get the starting position for relative coordinates
        start_i, start_j = starting_cell if starting_cell else (0, 0)
        
        print(f"[PATH FILE] Will convert from internal coords to relative coords")
        print(f"[PATH FILE] First 3 path cells: {full_path[:3]}")
        print(f"[PATH FILE] Last 3 path cells: {full_path[-3:]}")
        
        try:
            print(f"[PATH FILE] Opening file: {filepath}")
            with open(filepath, 'w') as f:
                for idx, cell in enumerate(full_path):
                    # Make coordinates relative to starting position
                    relative_i = cell[0] - start_i
                    relative_j = cell[1] - start_j
                    
                    # Internal grid: (i, j) where N:(+1,0), E:(0,-1), S:(-1,0), W:(0,+1)
                    # Output format: x y where x is East-West, y is North-South
                    # 
                    # The robot's i axis represents North-South (N is +i, S is -i)
                    # The robot's j axis represents West-East (E is -j, W is +j)
                    # 
                    # For output file, try completely different mapping:
                    # Maybe j maps to y and i maps to x?
                    x = relative_i  # Try i -> x
                    y = relative_j  # Try j -> y
                    f.write(f"{x} {y}\n")
                    
                    # Debug first few and last lines
                    if idx < 5 or idx >= len(full_path) - 2:
                        print(f"[PATH DEBUG] Cell {cell} -> relative({relative_i},{relative_j}) -> output({x},{y})")
            
            print(f"[PATH FILE] Successfully written to: {filepath}")
            print(f"[PATH FILE] Total cells in path: {len(full_path)}")
            print(f"[PATH FILE] Coordinates relative to start: {starting_cell}")
            if output_dir == controller_dir:
                print(f"[PATH FILE] Warning: C3_supervisor directory not found, file written to controller directory")
            return filepath
        except Exception as e:
            print(f"[ERROR] Failed to write path file: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def print_final_solution():
        """Print the final solution with shortest path through all targets."""
        if not detected_targets:
            print("\n" + "="*60)
            print("[FINAL SOLUTION] No targets found!")
            print("="*60)
            return
        
        target_positions = list(detected_targets.keys())
        # Use the actual starting cell, not (0,0)
        start_position = starting_cell if starting_cell else (0, 0)
        
        print("\n" + "="*60)
        print("[FINAL SOLUTION] All Targets Found!")
        print("="*60)
        print(f"Starting position: {start_position}")
        print(f"Total targets found: {len(detected_targets)}")
        print("\nTarget Details:")
        for pos, info in sorted(detected_targets.items()):
            print(f"  Target {info['id']} ({info['color']}): Cell {pos}")
        
        print("\n[DEBUG] All explored cells:")
        for cell in sorted(grid_nodes.keys()):
            neighbors = grid_nodes[cell].neighbors
            print(f"  Cell {cell}: neighbors = {neighbors}")
        
        # Solve TSP (including return to start)
        tour, total_dist, full_path = solve_tsp_greedy(target_positions, start_position)
        
        print(f"\n[DEBUG] Raw path (internal coordinates):")
        print(f"  {full_path[:20]}...")  # First 20 cells
        
        print(f"\n[SHORTEST PATH] Visiting order from start {start_position}:")
        for i, cell in enumerate(tour):
            if cell == start_position:
                print(f"  {i}. START at {cell}")
            elif cell in detected_targets:
                info = detected_targets[cell]
                print(f"  {i}. Target {info['id']} ({info['color']}) at {cell}")
        print(f"  {len(tour)}. RETURN to START at {start_position}")
        
        print(f"\n[PATH SUMMARY]")
        print(f"  Total distance (grid cells): {total_dist}")
        print(f"  Number of targets: {len(detected_targets)}")
        print(f"  Total cells in path: {len(full_path)}")
        
        print(f"\n[DETAILED PATH] Cell-by-cell route:")
        path_str = " -> ".join(str(cell) for cell in full_path)
        # Print in chunks to avoid too long lines
        chunk_size = 80
        for i in range(0, len(path_str), chunk_size):
            print(f"  {path_str[i:i+chunk_size]}")
        
        # Write the path file
        print()
        write_path_file(full_path)
        
        print("="*60 + "\n")
    
    #Determine next cell helpers
    def find_center_cell(cells):
        if not cells:
            return None
        avg_i = sum(i for i, j in cells) / len(cells)
        avg_j = sum(j for i, j in cells) / len(cells)
        #print("Center cell calculated as:", (round(avg_i), round(avg_j)))
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
        directions_info[front_h] = Checkpoint.OBSTACLE if (sensor_vals[0] > SENSOR_THRESHOLD or sensor_vals[7] > SENSOR_THRESHOLD or (sensor_vals[0] + sensor_vals[7] > 146)) else Checkpoint.FREE
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

            # clamp to prevent exceeding motor limits
            vL = max(-max_v, min(max_v, vL))
            vR = max(-max_v, min(max_v, vR))

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
        
        if x0 == gps_values[0] and y0 == gps_values[1] and starting_cell is None:
            print(f"[INIT] Robot starting world position: x={x0:.4f}, y={y0:.4f}")

        # Compute current position
        x_curr = gps_values[0]
        y_curr = gps_values[1]

        i = round((x_curr - x0) / CELL)
        j = round((y_curr - y0) / CELL)
        
        # Track the starting cell (first cell we're in)
        if starting_cell is None:
            starting_cell = (i, j)
            print(f"[INIT] Starting cell set to: {starting_cell}")
            print(f"[INIT] Heading: {cardinal}")
            print(f"[INIT] Sensor values: {vals}")

        # Get existing node or create a new one
        if (i, j) in grid_nodes:
            current_node = grid_nodes[(i, j)]
            current_node.world_pos = (x_curr, y_curr)  # Update position
        else:
            current_node = GridNode((i, j), (x_curr, y_curr))
        current_node.mark_visited()

        #Check if robot is  at cell center
        at_center = math.hypot(x_curr - (i * CELL + x0), y_curr - (j * CELL + y0)) <= CENTER_TOL

        # Check if we're visiting an already-detected target cell
        if at_center and (i, j) in detected_targets:
            info = detected_targets[(i, j)]
            print(f"[TARGET VISIT] Visiting previously found target {info['id']} ({info['color']}) at cell ({i}, {j})")
            # Generate updated path file when visiting a known target
            generate_path_for_current_targets()

        # Detect target colors when at cell center
        if at_center and (i, j) not in detected_targets:
            target_id, color_name = detect_target_color(camera)
            if target_id is not None:
                # Check if this target ID was already found at another location
                existing_targets = {info['id']: pos for pos, info in detected_targets.items()}
                
                if target_id in existing_targets:
                    # Target ID already found at a different position - possible duplicate
                    prev_pos = existing_targets[target_id]
                    print(f"[WARNING] Target {target_id} ({color_name}) detected again at cell ({i}, {j})")
                    print(f"[WARNING] Previously found at cell {prev_pos}. Ignoring duplicate.")
                else:
                    # New unique target
                    detected_targets[(i, j)] = {"id": target_id, "color": color_name}
                    print(f"[TARGET FOUND] Target {target_id} ({color_name}) detected at cell ({i}, {j})")
                    print(f"[TARGET FOUND] World position: ({x_curr:.3f}, {y_curr:.3f})")
                    print(f"[PROGRESS] Total targets found so far: {len(detected_targets)}")
                    
                    # Generate path file with current targets
                    generate_path_for_current_targets()
        
        # Check if exploration should end (found 3-7 targets and no more unvisited cells nearby)
        MIN_TARGETS = 3
        MAX_TARGETS = 7
        if len(detected_targets) >= MIN_TARGETS:
            # Check if there are still unvisited neighbors to explore
            has_unvisited = any(neighbor not in grid_nodes for neighbor in current_node.neighbors)
            has_comeback_cells = bool(cell_to_comeback)
            
            if (len(detected_targets) >= MAX_TARGETS) or (not has_unvisited and not has_comeback_cells):
                print(f"\n[EXPLORATION COMPLETE] Found {len(detected_targets)} targets")
                print("[EXPLORATION COMPLETE] Computing shortest path through all targets...")
                print_final_solution()
                # Stop the robot
                cruising_speed(0.0, 0.0)
                break  # Exit main loop

        #Initialize unvisited cells count 
        unvisited_count = 0

        # Update directions info if at center
        if at_center and current_node.neighbors == []:
            directions_info = get_directions_info(vals, cardinal)

            # Update the direction mapping
            direction_mapping[(i, j)] = directions_info

            if (i, j) == starting_cell:
                print(f"[START CELL] Direction mapping at starting position {(i, j)}:")
                for direction, status in directions_info.items():
                    print(f"  {direction}: {status}")

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

                #print("Next cell : ", next_cell)

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
            cruising_speed(1 * MAX_SPEED, 1 * MAX_SPEED)
        else:
            # No next_cell yet: just stop (or keep a gentle crawl if you prefer)
            cruising_speed(0.0, 0.0)
            
        #print("Cells to comeback to:")
        #for cell, count in cell_to_comeback.items():
            #print(f"Cell {cell}: {count} unvisited neighbors")

        #print ("-----")  # Separator for readability
        #print("Next cell:", next_cell)

        #print the reachable neighbors from every cell in the grid
        #print("Neighbors from each cell:")
        #for cell, neighbors in grid_nodes.items():
            #print(f"Cell {cell}: {neighbors.neighbors}")
    
    # If loop ended without break (simulation ended), print solution anyway
    print("\n[SIMULATION ENDED] Loop exited, checking for targets...")
    if detected_targets:
        print(f"[SIMULATION ENDED] Found {len(detected_targets)} targets. Computing final solution...")
        print_final_solution()
    else:
        print("[SIMULATION ENDED] No targets were detected during exploration.")

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