"""Grid representation and path planning helpers for the mapping controller.

This module is adapted for the C2 mapping challenge and aligned with the
coordinate conventions used by the current Webots controller:

- The origin cell ``(0, 0)`` corresponds to the robot starting pose.
- Index *i* grows when the robot moves north (positive GPS X).
- Index *j* grows toward the west (negative GPS Y).  This matches the
  ``DIRECTION_OFFSETS`` table already in use by the controller where moving
  east decreases *j*.

Each cell stores wall information for the four cardinal directions in addition
to bookkeeping flags (``visited`` and ``frontier``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from heapq import heappush, heappop
from typing import Dict, Iterable, List, Optional, Tuple

# Canonical direction labels
NORTH = "N"
EAST = "E"
SOUTH = "S"
WEST = "W"
CARDINALS = (NORTH, EAST, SOUTH, WEST)

# Mapping between headings and grid offsets (i, j).
# NOTE: moving east decreases j in our controller, hence the (0, -1) entry.
DIR_VECTORS: Dict[str, Tuple[int, int]] = {
    NORTH: (1, 0),
    EAST: (0, -1),
    SOUTH: (-1, 0),
    WEST: (0, 1),
}

# Opposite headings used when updating walls symmetrically between cells.
OPPOSITE: Dict[str, str] = {
    NORTH: SOUTH,
    EAST: WEST,
    SOUTH: NORTH,
    WEST: EAST,
}


@dataclass
class Cell:
    """State tracked for each explored grid cell."""

    coords: Tuple[int, int]
    visited: bool = False
    frontier: bool = False
    walls: Dict[str, Optional[bool]] = field(
        default_factory=lambda: {d: None for d in CARDINALS}
    )

    def set_wall(self, direction: str, is_wall: bool) -> None:
        """Record whether a wall exists in the given direction."""
        if direction not in self.walls:
            raise ValueError(f"Invalid direction {direction!r}")
        self.walls[direction] = is_wall

    def is_wall(self, direction: str) -> Optional[bool]:
        return self.walls.get(direction)

    def known_free_directions(self) -> List[str]:
        """Return directions known to be free (no wall)."""
        return [d for d, wall in self.walls.items() if wall is False]


class GridMap:
    """Sparse grid storing the explored maze."""

    def __init__(self) -> None:
        self._cells: Dict[Tuple[int, int], Cell] = {}

    def get(self, coords: Tuple[int, int]) -> Cell:
        """Return the cell at *coords*, creating it if necessary."""
        if coords not in self._cells:
            self._cells[coords] = Cell(coords)
        return self._cells[coords]

    def mark_visited(self, coords: Tuple[int, int]) -> None:
        cell = self.get(coords)
        cell.visited = True
        cell.frontier = False

    def mark_frontier(self, coords: Tuple[int, int]) -> None:
        cell = self.get(coords)
        if not cell.visited:
            cell.frontier = True

    def clear_frontier(self, coords: Tuple[int, int]) -> None:
        if coords in self._cells:
            self._cells[coords].frontier = False

    def set_wall_between(
        self,
        origin: Tuple[int, int],
        direction: str,
        is_wall: bool,
    ) -> None:
        """Record wall information for *origin* and the adjacent cell."""
        if direction not in DIR_VECTORS:
            raise ValueError(f"Invalid direction {direction!r}")

        origin_cell = self.get(origin)
        origin_cell.set_wall(direction, is_wall)

        di, dj = DIR_VECTORS[direction]
        neighbor_coords = (origin[0] + di, origin[1] + dj)
        neighbor_cell = self.get(neighbor_coords)
        neighbor_cell.set_wall(OPPOSITE[direction], is_wall)

        if is_wall:
            neighbor_cell.frontier = False
        else:
            if not neighbor_cell.visited:
                neighbor_cell.frontier = True

    def neighbors(self, coords: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Return neighboring cells that are known to be free."""
        cell = self.get(coords)
        result: List[Tuple[int, int]] = []
        for direction in cell.known_free_directions():
            di, dj = DIR_VECTORS[direction]
            result.append((coords[0] + di, coords[1] + dj))
        return result

    def frontiers(self) -> List[Tuple[int, int]]:
        return [coords for coords, cell in self._cells.items() if cell.frontier]

    def visited_cells(self) -> Iterable[Cell]:
        return (cell for cell in self._cells.values() if cell.visited)

    def cells(self) -> Iterable[Cell]:
        return self._cells.values()


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(
    grid: GridMap,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    """Run A* on the discovered grid using only confirmed free directions."""

    if start == goal:
        return [start]

    open_heap: List[Tuple[int, int, Tuple[int, int]]] = []
    heappush(open_heap, (manhattan(start, goal), 0, start))

    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], int] = {start: 0}

    while open_heap:
        _, cost, current = heappop(open_heap)

        if current == goal:
            return _reconstruct_path(came_from, current)

        for neighbor in grid.neighbors(current):
            tentative = cost + 1
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                priority = tentative + manhattan(neighbor, goal)
                heappush(open_heap, (priority, tentative, neighbor))

    return None


def _reconstruct_path(
    came_from: Dict[Tuple[int, int], Tuple[int, int]],
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
