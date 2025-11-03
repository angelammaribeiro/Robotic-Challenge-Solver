"""Webots controller for the C2 mapping challenge.

The controller explores the maze cell-by-cell, building an internal grid aligned
with the assignment convention: GPS X increases toward geographic north while
the compass Y component points to north. We compensate for this 90° rotation
whenever we convert between world coordinates, grid indices, and headings.

The robot only rotates when sitting at the centre of a grid cell and travels to
adjacent cells using a frontier-driven strategy backed by A* path planning when
necessary. Once every frontier has been explored, a 27x55 textual map is
written to disk in the supervisor's expected format.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from controller import Robot

from grid import (
    CARDINALS,
    GridMap,
    CELL_SIZE,
    CENTER_TOL,
    SENSOR_THRESHOLD,
    astar,
)
from mapping_writer import write_map

# ============
# Configuration
# ============
MAX_SPEED = 6.28
ROTATION_TOL = math.radians(3.0)
HEADING_CARDINAL_TOL = 15.0  # degrees
MAP_OUTPUT_PATH = "agent_nmec1_nmec2_c2.map"

REL_TO_ABS: Dict[str, Dict[str, str]] = {
    "N": {"front": "N", "right": "E", "back": "S", "left": "W"},
    "E": {"front": "E", "right": "S", "back": "W", "left": "N"},
    "S": {"front": "S", "right": "W", "back": "N", "left": "E"},
    "W": {"front": "W", "right": "N", "back": "E", "left": "S"},
}

SENSOR_GROUPS = {
    "front": (0, 7),
    "right": (1, 2),
    "left": (5, 6),
    "back": (3, 4),
}

CARDINAL_TO_DEG = {
    "N": 0.0,
    "W": 90.0,
    "S": 180.0,
    "E": 270.0,
}

DRIVE_KP_DIST = 4.0
DRIVE_KP_HEADING = 3.0
ROTATE_KP = 2.2
MAX_DRIVE_SPEED = 0.6 * MAX_SPEED
MAX_TURN_SPEED = 0.5 * MAX_SPEED


# ==========
# Math utils
# ==========
def normalize_angle_rad(theta: float) -> float:
    return (theta + math.pi) % (2 * math.pi) - math.pi


def shortest_angle_error_rad(target: float, current: float) -> float:
    return normalize_angle_rad(target - current)


def heading_to_cardinal(heading_deg: float) -> str:
    """Quantise heading (degrees) to the closest cardinal direction."""
    index = int((heading_deg + 45.0) // 90.0) % 4
    return CARDINALS[index]


def delta_to_heading(delta: Tuple[int, int]) -> str:
    di, dj = delta
    if di > 0:
        return "N"
    if di < 0:
        return "S"
    if dj > 0:
        return "E"
    if dj < 0:
        return "W"
    return "N"


def desired_heading_deg(dx: float, dy: float) -> float:
    """Compute the desired heading in degrees for vector (dx, dy).

    We invert the dy component because increasing GPS Y points east whereas our
    heading convention increases clockwise (0° = North, 90° = West).
    """
    angle = math.degrees(math.atan2(-dy, dx))
    return (angle + 360.0) % 360.0


def cell_center_world(x0: float, y0: float, cell: Tuple[int, int]) -> Tuple[float, float]:
    i, j = cell
    cx = x0 + i * CELL_SIZE
    cy = y0 + j * CELL_SIZE
    return cx, cy


# ==================
# Motion primitives
# ==================
def rotate_in_place(left, right, error_rad: float) -> None:
    turn = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, ROTATE_KP * error_rad))
    left.setVelocity(-turn)
    right.setVelocity(turn)


def drive_towards(left, right, dist: float, heading_error: float) -> None:
    forward = max(0.0, min(MAX_DRIVE_SPEED, DRIVE_KP_DIST * dist))
    turn = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, DRIVE_KP_HEADING * heading_error))
    left.setVelocity(forward - turn)
    right.setVelocity(forward + turn)


def stop_robot(left, right) -> None:
    left.setVelocity(0.0)
    right.setVelocity(0.0)


# ==============
# Sensing helpers
# ==============
def read_heading_deg(compass) -> float:
    nx, ny, _ = compass.getValues()
    heading = math.degrees(math.atan2(nx, ny))
    return (heading + 360.0) % 360.0


def classify_walls(
    current_heading: str,
    sensor_values: List[float],
) -> Dict[str, bool]:
    walls: Dict[str, bool] = {}
    for rel_dir, indices in SENSOR_GROUPS.items():
        blocked = any(sensor_values[idx] > SENSOR_THRESHOLD for idx in indices)
        abs_dir = REL_TO_ABS[current_heading][rel_dir]
        walls[abs_dir] = blocked
    return walls


# ============
# Main control
# ============
def run_controller() -> None:
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    left = robot.getDevice("left wheel motor")
    right = robot.getDevice("right wheel motor")
    for motor in (left, right):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)

    distance_sensors = [robot.getDevice(f"ps{k}") for k in range(8)]
    for sensor in distance_sensors:
        sensor.enable(timestep)

    grid = GridMap()
    origin_cell = (0, 0)
    path_queue: List[Tuple[int, int]] = []
    map_written = False

    x0 = y0 = None
    last_classified_cell: Optional[Tuple[int, int]] = None
    mode = "IDLE"

    while robot.step(timestep) != -1:
        position = gps.getValues()
        heading_deg = read_heading_deg(compass)
        heading_rad = math.radians(heading_deg)
        cardinal_heading = heading_to_cardinal(heading_deg)

        if x0 is None and y0 is None:
            x0, y0 = position[0], position[1]
            origin_cell = (0, 0)

        i = round((position[0] - x0) / CELL_SIZE)
        j = round((position[1] - y0) / CELL_SIZE)
        current_cell = (i, j)

        cell_cx, cell_cy = cell_center_world(x0, y0, current_cell)
        center_dist = math.hypot(position[0] - cell_cx, position[1] - cell_cy)
        at_center = center_dist <= CENTER_TOL

        if at_center:
            grid.mark_visited(current_cell)
            if path_queue and current_cell == path_queue[0]:
                path_queue.pop(0)

            if current_cell != last_classified_cell:
                sensor_values = [sensor.getValue() for sensor in distance_sensors]
                walls = classify_walls(cardinal_heading, sensor_values)
                for direction, blocked in walls.items():
                    grid.set_wall_between(current_cell, direction, blocked)
                last_classified_cell = current_cell

            if not path_queue:
                adjacent_targets = [
                    neighbor
                    for neighbor in grid.neighbors(current_cell)
                    if not grid.get(neighbor).visited
                ]

                if adjacent_targets:
                    path_queue = [adjacent_targets[0]]
                else:
                    frontier_targets = [
                        cell
                        for cell in grid.frontiers()
                        if not grid.get(cell).visited
                    ]
                    frontier_targets.sort(key=lambda cell: abs(cell[0] - i) + abs(cell[1] - j))

                    for frontier in frontier_targets:
                        candidate_path = astar(grid, current_cell, frontier)
                        if candidate_path and len(candidate_path) > 1:
                            path_queue = candidate_path[1:]
                            break

            if not path_queue and not grid.frontiers() and not map_written:
                stop_robot(left, right)
                write_map(grid, origin_cell, MAP_OUTPUT_PATH)
                map_written = True

            if path_queue:
                next_cell = path_queue[0]
                delta = (next_cell[0] - current_cell[0], next_cell[1] - current_cell[1])
                target_heading = delta_to_heading(delta)
                target_heading_deg = CARDINAL_TO_DEG[target_heading]
                target_heading_rad = math.radians(target_heading_deg)
                angle_error = shortest_angle_error_rad(target_heading_rad, heading_rad)

                if abs(angle_error) > math.radians(HEADING_CARDINAL_TOL):
                    mode = "ROTATING"
                else:
                    mode = "DRIVING"
            else:
                mode = "IDLE"

        if mode == "ROTATING" and path_queue:
            next_cell = path_queue[0]
            delta = (next_cell[0] - current_cell[0], next_cell[1] - current_cell[1])
            target_heading = delta_to_heading(delta)
            target_heading_rad = math.radians(CARDINAL_TO_DEG[target_heading])
            angle_error = shortest_angle_error_rad(target_heading_rad, heading_rad)

            if abs(angle_error) <= ROTATION_TOL:
                mode = "DRIVING"
                stop_robot(left, right)
            else:
                rotate_in_place(left, right, angle_error)
                continue

        if mode == "DRIVING" and path_queue:
            goal_cell = path_queue[0]
            goal_x, goal_y = cell_center_world(x0, y0, goal_cell)
            dx = goal_x - position[0]
            dy = goal_y - position[1]
            distance = math.hypot(dx, dy)
            desired_deg = desired_heading_deg(dx, dy)
            desired_rad = math.radians(desired_deg)
            heading_error = shortest_angle_error_rad(desired_rad, heading_rad)

            if distance <= CENTER_TOL * 0.6:
                stop_robot(left, right)
                mode = "IDLE"
            else:
                drive_towards(left, right, distance, heading_error)
            continue

        if mode == "IDLE":
            stop_robot(left, right)

        if map_written:
            break


if __name__ == "__main__":
    run_controller()
