"""ASCII map writer for the incremental mapping task.

The file format follows the C2 specification: 27 lines by 55 columns where the
centre of the canvas marks the robot's starting cell with the character ``I``.
The coordinate system matches the controller's grid indices (``i`` grows toward
north, ``j`` grows toward west).

Only non-space characters are scored by ``mapping_score.awk``.  We therefore
focus on serialising confirmed walls together with the origin marker while
keeping unexplored or free cells as blanks.  This mirrors the supervisor output
closely enough for scoring while avoiding premature assumptions about unseen
areas.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

from grid import CARDINALS, GridMap, NORTH, SOUTH, EAST, WEST

MAP_ROWS = 27
MAP_COLS = 55
CENTER_ROW = MAP_ROWS // 2
CENTER_COL = MAP_COLS // 2
STEP_I = 2  # vertical spacing (north/south)
STEP_J = 2  # horizontal spacing (east/west)

EMPTY_CHAR = " "
VISITED_CHAR = "X"
FRONTIER_CHAR = " "
NORTH_SOUTH_WALL = "|"
EAST_WEST_WALL = "-"
ORIGIN_CHAR = "I"
FREE_CHAR = "X"

# Offsets around a cell centre used to draw walls and junctions.
NORTH_OFFSETS = [(-1, 0)]
SOUTH_OFFSETS = [(1, 0)]
WEST_OFFSETS = [(0, -1)]
EAST_OFFSETS = [(0, 1)]

def _new_canvas() -> List[List[str]]:
    return [[EMPTY_CHAR for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]


def _cell_to_canvas(coords: Tuple[int, int]) -> Tuple[int, int]:
    i, j = coords
    # Rotate the canvas so that north extends to the right, south to the left,
    # west upward and east downward relative to the supervisor convention.
    row = CENTER_ROW - j * STEP_I  # j grows toward west -> move up
    col = CENTER_COL + i * STEP_J  # i grows toward north -> move right
    return row, col


def _within_canvas(row: int, col: int) -> bool:
    return 0 <= row < MAP_ROWS and 0 <= col < MAP_COLS


def _draw_offsets(
    canvas: List[List[str]], offsets: Iterable[Tuple[int, int]], row: int, col: int, char: str
) -> None:
    for dr, dc in offsets:
        r, c = row + dr, col + dc
        if _within_canvas(r, c):
            canvas[r][c] = char
OFFSETS = {
    NORTH: EAST_OFFSETS,  # draw north to the right
    SOUTH: WEST_OFFSETS,  # draw south to the left
    WEST: NORTH_OFFSETS,  # draw west upward
    EAST: SOUTH_OFFSETS,  # draw east downward
}


def _draw_direction(
    canvas: List[List[str]],
    row: int,
    col: int,
    direction: str,
    wall_state: bool | None,
    allow_free: bool,
) -> None:
    if wall_state is None:
        return

    offsets = OFFSETS[direction]

    if wall_state is True:
        if not allow_free:
            return
        char = NORTH_SOUTH_WALL if direction in (NORTH, SOUTH) else EAST_WEST_WALL
        _draw_offsets(canvas, offsets, row, col, char)
        return

    if not allow_free:
        return

    # Only place free markers when the slot is still empty; never overwrite walls.
    for dr, dc in offsets:
        r, c = row + dr, col + dc
        if _within_canvas(r, c) and canvas[r][c] == EMPTY_CHAR:
            canvas[r][c] = FREE_CHAR


def write_map(grid: GridMap, origin: Tuple[int, int], path: str) -> None:
    """Serialise the explored ``grid`` to ``path`` using the required format."""
    canvas = _new_canvas()

    for cell in grid.cells():
        row, col = _cell_to_canvas(cell.coords)
        if not _within_canvas(row, col):
            continue

        if cell.coords == origin:
            canvas[row][col] = ORIGIN_CHAR
        elif cell.visited:
            canvas[row][col] = VISITED_CHAR
        elif cell.frontier:
            canvas[row][col] = FRONTIER_CHAR

        allow_free = cell.visited or cell.coords == origin

        for direction in CARDINALS:
            wall_state = cell.is_wall(direction)
            _draw_direction(canvas, row, col, direction, wall_state, allow_free)

    lines = ["".join(row) for row in canvas]
    if len(lines) != MAP_ROWS or any(len(line) != MAP_COLS for line in lines):
        raise RuntimeError("Generated map does not meet 27x55 specification")

    with open(path, "w", encoding="utf-8") as map_file:
        map_file.write("\n".join(lines))
        map_file.write("\n")
