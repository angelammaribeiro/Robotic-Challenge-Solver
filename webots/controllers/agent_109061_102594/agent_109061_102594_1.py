"""""sample2 controller."""

from controller import Robot
import math
from typing import Dict, List, Tuple
from gridNode import GridNode
from checkpoint import Checkpoint


# ==========
# Constants
# ==========
CELL_SIZE   = 0.15
CENTER_TOL  = 0.02
FRONT_T     = 100
SIDE_T      = 100
BACK_T      = 100
MAX_SPEED   = 6.28


# =========================
# Heading / Map Conventions
# =========================
HEADINGS = ["N", "E", "S", "W"]

# Relative → absolute heading map
REL_TO_ABS = {
    "N": {"front": "N", "left": "W", "right": "E", "back": "S"},
    "E": {"front": "E", "left": "N", "right": "S", "back": "W"},
    "S": {"front": "S", "left": "E", "right": "W", "back": "N"},
    "W": {"front": "W", "left": "S", "right": "N", "back": "E"},
}

# Grid offsets (i ← X, j ← Y). Keep these.
DIR_OFFSETS: Dict[str, Tuple[int, int]] = {
    "N": (1,  0),   # +Y
    "E": (0,  -1),   # +X
    "S": (-1, 0),   # -Y
    "W": (0, 1),   # -X
}

# Ideal yaw for each heading (true Webots compass frame)
IDEAL_YAWS: Dict[str, float] = {
    "N": 0.0,
    "E": math.pi / 2,
    "S": math.pi,
    "W": 3 * math.pi / 2,
}


# ===============
# Math utilities
# ===============
def shortest_angle_error(a: float, b: float) -> float:
    """Shortest signed difference a - b in (-pi, pi]."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def normalize_angle(theta: float) -> float:
    """Normalize to [0, 2π)."""
    return theta % (2 * math.pi)


def round_to_cell(coord: float, origin: float) -> int:
    """Stable rounding from world coord to grid index."""
    return round((coord - origin) / CELL_SIZE)


def at_center(x: float, y: float, cx: float, cy: float) -> bool:
    """Within tolerance of cell center."""
    return math.hypot(x - cx, y - cy) <= CENTER_TOL


# ==================
# Motion primitives
# ==================
def cruising(left, right, max_speed: float = MAX_SPEED) -> None:
    left.setVelocity(0.4 * max_speed)
    right.setVelocity(0.4 * max_speed)


def rotate_robot(yaw: float, target_yaw: float, left, right,
                 max_speed: float = MAX_SPEED) -> None:
    """Proportional in-place turn toward target_yaw."""
    err = shortest_angle_error(target_yaw, yaw)
    Kp = 1.2
    turn_cmd = -Kp * err                     # +err → CW
    cap = 0.35 * max_speed
    turn_cmd = max(-cap, min(cap, turn_cmd)) # clamp

    # in-place turn: left = -ω, right = +ω
    left.setVelocity(-turn_cmd)
    right.setVelocity(+turn_cmd)


# ==================
# Sensing utilities
# ==================
def read_pose(gps) -> Tuple[float, float, float]:
    """(x, y, z) world position."""
    return gps.getValues()


def read_yaw_cw_from_north(compass) -> float:
    """
    TRUE WEBOTS COMPASS:
    compass.getValues() returns the vector to magnetic North in the robot frame.
    Use XY components: atan2(x, y) gives CCW from North. Negate for CW-from-North.
    0 = North, π/2 = East, π = South, 3π/2 = West.
    """
    nx, ny, _ = compass.getValues()
    yaw_ccw_from_north = math.atan2(nx, ny)      # CCW from North
    yaw_cw_from_north  = (-yaw_ccw_from_north) % (2 * math.pi)
    return yaw_cw_from_north


def discrete_heading(yaw: float) -> str:
    """Quantize yaw to one of HEADINGS."""
    idx = int((yaw + math.pi / 4) // (math.pi / 2)) % 4
    return HEADINGS[idx]


def classify_neighbors_at_center(
    current_heading: str,
    dist_values: List[float],
    neighbors: Dict[str, Checkpoint],
) -> None:
    """Update neighbors dict in-place based on distance thresholds."""
    front_h = REL_TO_ABS[current_heading]["front"]
    right_h = REL_TO_ABS[current_heading]["right"]
    left_h  = REL_TO_ABS[current_heading]["left"]
    back_h  = REL_TO_ABS[current_heading]["back"]

    # Front
    if dist_values[0] > FRONT_T or dist_values[7] > FRONT_T:
        neighbors[front_h] = Checkpoint.OBSTACLE
    elif neighbors[front_h] == Checkpoint.UNKNOWN:
        neighbors[front_h] = Checkpoint.FREE

    # Right
    if dist_values[2] > SIDE_T:
        neighbors[right_h] = Checkpoint.OBSTACLE
    elif neighbors[right_h] == Checkpoint.UNKNOWN:
        neighbors[right_h] = Checkpoint.FREE

    # Left
    if dist_values[5] > SIDE_T:
        neighbors[left_h] = Checkpoint.OBSTACLE
    elif neighbors[left_h] == Checkpoint.UNKNOWN:
        neighbors[left_h] = Checkpoint.FREE

    # Back
    if dist_values[4] > BACK_T or dist_values[3] > BACK_T:
        neighbors[back_h] = Checkpoint.OBSTACLE
    elif neighbors[back_h] == Checkpoint.UNKNOWN:
        neighbors[back_h] = Checkpoint.FREE


def free_neighbors(i: int, j: int,
                   neighbors: Dict[str, Checkpoint]) -> List[Tuple[int, int]]:
    """List reachable neighbor cells from FREE directions."""
    cells: List[Tuple[int, int]] = []
    for d, state in neighbors.items():
        if state == Checkpoint.FREE and d in DIR_OFFSETS:
            di, dj = DIR_OFFSETS[d]
            cells.append((i + di, j + dj))
    return cells


def delta_to_heading(di: int, dj: int) -> str:
    """Translate (di, dj) to absolute heading."""
    if   di == 1 and dj == 0:  return "N"
    if   di == -1 and dj == 0: return "S"
    if   di == 0 and dj == 1:  return "E"
    if   di == 0 and dj == -1: return "W"
    return "N"  # safe default


# ==============
# Main controller
# ==============
def run_robot(robot: Robot) -> None:
    # --- devices ---
    timestep = int(robot.getBasicTimeStep())

    left  = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    for m in (left, right):
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

    gps = robot.getDevice('gps');       gps.enable(timestep)
    compass = robot.getDevice('compass'); compass.enable(timestep)

    camera = robot.getDevice('camera'); camera.enable(timestep)  # left enabled as before

    dist_sensors = [robot.getDevice(f'ps{k}') for k in range(8)]
    for s in dist_sensors:
        s.enable(timestep)

    # --- state ---
    x0 = y0 = None
    grid: Dict[Tuple[int, int], Dict] = {}
    next_cell: Tuple[int, int] | None = None

    prev_cell = prev_world = None
    prev_heading = None
    CURRENT_STATE = "EXPLORING"
    comeback = set()  # kept

    # --- main loop ---
    while robot.step(timestep) != -1:
        # Pose & discretization
        x, y, z = read_pose(gps)
        if x0 is None and y0 is None:
            x0, y0 = x, y

        i = round_to_cell(x, x0)   # X -> i
        j = round_to_cell(y, y0)   # Y -> j

        cx, cy = x0 + i * CELL_SIZE, y0 + j * CELL_SIZE
        centered = at_center(x, y, cx, cy)

        # Heading from TRUE Webots compass
        yaw = read_yaw_cw_from_north(compass)
        current_heading = discrete_heading(yaw)

        # Neighbor holder
        if (i, j) not in grid:
            neighbors: Dict[str, Checkpoint] = {
                "N": Checkpoint.UNKNOWN,
                "E": Checkpoint.UNKNOWN,
                "S": Checkpoint.UNKNOWN,
                "W": Checkpoint.UNKNOWN,
            }
        else:
            neighbors = grid[(i, j)]["neighbors"]

        # Motion delta logging (kept)
        current_cell = (i, j)
        current_world = (x, y)
        if prev_cell is None:
            prev_cell, prev_world, prev_heading = current_cell, current_world, current_heading
        else:
            di_log = current_cell[0] - prev_cell[0]
            dj_log = current_cell[1] - prev_cell[1]
            dx = current_world[0] - prev_world[0]
            dy = current_world[1] - prev_world[1]
            print(
                "Motion delta:",
                f"prev_cell={prev_cell}",
                f"current_cell={current_cell}",
                f"heading={prev_heading}",
                f"Δi={di_log}", f"Δj={dj_log}",
                f"Δx={dx:.4f}", f"Δy={dy:.4f}",
                f"distance={math.hypot(dx, dy):.4f}",
            )
            if current_cell != prev_cell:
                print(
                    "Cell transition:",
                    f"{prev_cell} -> {current_cell}",
                    f"heading={prev_heading}",
                    f"Δi={di_log}", f"Δj={dj_log}",
                    f"Δx={dx:.3f}", f"Δy={dy:.3f}",
                )
            prev_cell, prev_world, prev_heading = current_cell, current_world, current_heading

        # Classify neighbors only when centered & exploring
        if centered and CURRENT_STATE == "EXPLORING":
            vals = [s.getValue() for s in dist_sensors]
            classify_neighbors_at_center(current_heading, vals, neighbors)

            # FREE → reachable
            reachable = free_neighbors(i, j, neighbors)
            unvisited_here = [nb for nb in reachable if nb not in grid]

            # comeback bookkeeping
            if len(unvisited_here) > 1:
                comeback.add(current_cell)
            else:
                comeback.discard(current_cell)
        else:
            reachable = []

        # Record / update grid
        entry = grid.get(current_cell)
        if entry is None:
            grid[current_cell] = {
                "node": GridNode(current_cell, current_world, list(reachable), True),
                "neighbors": dict(neighbors),
            }
        else:
            node = entry["node"]
            if node.neighbors != reachable and reachable:
                node.neighbors = list(reachable)
            entry["neighbors"] = dict(neighbors)

        # Choose next cell (same policy as before)
        if reachable and (next_cell is None or next_cell not in reachable):
            unvisited_here = [nb for nb in reachable if nb not in grid]
            if unvisited_here:
                next_cell = unvisited_here[0]

        # No target? keep moving (or stop if you prefer)
        if next_cell is None:
            print(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}, Yaw={math.degrees(yaw):.2f}°")
            if CURRENT_STATE == "EXPLORING":
                cruising(left, right)
            continue

        # Delta → target heading (consistent with DIR_OFFSETS)
        di = next_cell[0] - i
        dj = next_cell[1] - j
        target_heading = delta_to_heading(di, dj)
        target_yaw = IDEAL_YAWS[target_heading]
        err = shortest_angle_error(target_yaw, yaw)

        # State transitions
        if centered and target_heading != current_heading and CURRENT_STATE == "EXPLORING":
            CURRENT_STATE = "ROTATING"
        if CURRENT_STATE == "ROTATING" and abs(err) < math.radians(3):
            CURRENT_STATE = "EXPLORING"

        # Actuation
        if CURRENT_STATE == "EXPLORING":
            cruising(left, right)
        elif CURRENT_STATE == "ROTATING":
            rotate_robot(yaw, target_yaw, left, right)

        # Debug
        print(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}, Yaw={math.degrees(yaw):.2f}°")
        print(f"Current cell: {current_cell}, Next cell: {next_cell}, Heading: {current_heading}, Target heading: {target_heading}, State: {CURRENT_STATE}")


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
