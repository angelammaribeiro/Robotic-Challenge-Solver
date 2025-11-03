"""ASCII map writer for the mapping challenge.

The output must contain exactly 27 lines and 55 columns. The central
character (row 13, column 27; 0-based indexing) is reserved for the starting
cell and is marked with ``I``. Each grid cell is projected to the canvas using a
2-character spacing that honours the assignment convention where north matches
positive compass Y while the GPS X grows toward north. This means that a grid
step along +i moves *up* (toward lower row indices) within the rendered map.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

from grid import CARDINALS, GridMap, NORTH, SOUTH, EAST, WEST

MAP_ROWS = 27
MAP_COLS = 55
CENTER_ROW = MAP_ROWS // 2
CENTER_COL = MAP_COLS // 2

# Cell spacing on the ASCII canvas. With a step of two characters we have
# enough room to draw walls while keeping the discovered arena well within the
# 27x55 limits.
STEP_I = 2  # affects rows (north/south)
STEP_J = 4  # affects columns (east/west)

# Individual characters used in the textual representation.
EMPTY_CHAR = " "
VISITED_CHAR = "."
FRONTIER_CHAR = "?"
NORTH_SOUTH_WALL = "-"
EAST_WEST_WALL = "|"
CORNER_CHAR = "+"
ORIGIN_CHAR = "I"

# Offsets around a cell centre used to draw walls and corner junctions.
NORTH_OFFSETS = [(-1, -1), (-1, 0), (-1, 1)]
SOUTH_OFFSETS = [(1, -1), (1, 0), (1, 1)]
WEST_OFFSETS = [(-1, -1), (0, -1), (1, -1)]
EAST_OFFSETS = [(-1, 1), (0, 1), (1, 1)]


def _new_canvas() -> List[List[str]]:
    return [[EMPTY_CHAR for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]


def _cell_to_canvas(coords: Tuple[int, int]) -> Tuple[int, int]:
    """Convert grid coordinates to canvas coordinates."""
    i, j = coords
    row = CENTER_ROW - i * STEP_I
    col = CENTER_COL + j * STEP_J
    return row, col


def _within_canvas(row: int, col: int) -> bool:
    return 0 <= row < MAP_ROWS and 0 <= col < MAP_COLS


def _draw_offsets(canvas: List[List[str]], offsets, row: int, col: int, char: str) -> None:
    for dr, dc in offsets:
        r, c = row + dr, col + dc
        if _within_canvas(r, c):
            canvas[r][c] = char


def _draw_wall(canvas: List[List[str]], row: int, col: int, direction: str) -> None:
    if direction == NORTH:
        _draw_offsets(canvas, NORTH_OFFSETS, row, col, NORTH_SOUTH_WALL)
        for dc in (-1, 1):
            r, c = row - 1, col + dc
            if _within_canvas(r, c):
                canvas[r][c] = CORNER_CHAR
    elif direction == SOUTH:
        _draw_offsets(canvas, SOUTH_OFFSETS, row, col, NORTH_SOUTH_WALL)
        for dc in (-1, 1):
            r, c = row + 1, col + dc
            if _within_canvas(r, c):
                canvas[r][c] = CORNER_CHAR
    elif direction == WEST:
        _draw_offsets(canvas, WEST_OFFSETS, row, col, EAST_WEST_WALL)
        for dr in (-1, 1):
            r, c = row + dr, col - 1
            if _within_canvas(r, c):
                canvas[r][c] = CORNER_CHAR
    elif direction == EAST:
        _draw_offsets(canvas, EAST_OFFSETS, row, col, EAST_WEST_WALL)
        for dr in (-1, 1):
            r, c = row + dr, col + 1
            if _within_canvas(r, c):
                canvas[r][c] = CORNER_CHAR


def write_map(grid: GridMap, origin: Tuple[int, int], path: str) -> None:
    """Serialise the explored ``grid`` to ``path`` using the required format."""
    canvas = _new_canvas()

    # Draw cells.
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

        for direction in CARDINALS:
            wall_state = cell.is_wall(direction)
            if wall_state is True:
                _draw_wall(canvas, row, col, direction)

    lines = ["".join(row) for row in canvas]
    if len(lines) != MAP_ROWS or any(len(line) != MAP_COLS for line in lines):
        raise RuntimeError("Generated map does not meet 27x55 specification")

    with open(path, "w", encoding="utf-8") as map_file:
        map_file.write("\n".join(lines))
        map_file.write("\n")
