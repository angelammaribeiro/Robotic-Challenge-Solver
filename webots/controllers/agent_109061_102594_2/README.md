# C2 Mapping Controller

Python controller for the RMI-2526 Assignment 2 (C2 – Mapping) targeting the
e-puck robot in Webots. The controller explores the 7×14 maze with noiseless GPS
and compass, builds an internal grid map, and exports the discovered layout to a
27×55 `.map` file compatible with `mapping_score.awk`.

## How it works

- The first GPS reading anchors the grid origin; grid index `i` increases
  northward (GPS X), while `j` increases eastward (GPS Y).
- Compass headings are computed with `atan2(nx, ny)` which yields
  0° = North, 90° = West, 180° = South, 270° = East. The controller keeps this
  convention throughout and corrects for the 90° rotation between GPS and
  compass frames.
- The robot only pivots while centred in a cell. Distance sensors detect walls
  on the four sides; frontier-based exploration and A* routing guarantee full
  coverage of reachable cells.
- When no frontier remains the controller writes `agent_nmec1_nmec2_c2.map` in
  the controller directory.

## Running

Launch Webots with the provided world and select `agent_nmec1_nmec2_c2/controller.py`
as the controller for the e-puck. No extra dependencies are required beyond the
Webots Python API.
