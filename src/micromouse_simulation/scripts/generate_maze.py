#!/usr/bin/env python3
"""
Micromouse maze + Gazebo SDF world generator.

Generates a maze on an N x N grid using a randomized depth-first spanning
tree (guarantees every cell is reachable), then optionally knocks down a
fraction of the remaining internal walls to introduce loops (real
micromouse competition mazes are not perfect trees), then forces the
center 2x2 block fully open as the goal area per standard micromouse
convention.

Outputs:
  <out-dir>/<name>.sdf   - a Gazebo world with the maze as static geometry
  <out-dir>/<name>.json  - the logical maze (per-cell N/E/S/W walls), for
                            tooling, tests, or debug overlays later on

Standard dimensions default to IFFA/NEMO classic-size rules: 16x16 cells,
180mm cell pitch, 12mm wall thickness, 50mm wall height. All overridable.
"""
import argparse
import json
import random
from pathlib import Path


def build_args():
    p = argparse.ArgumentParser(description="Micromouse maze + Gazebo world generator")
    p.add_argument("--size", type=int, default=16, help="grid size N (NxN cells)")
    p.add_argument("--cell-size", type=float, default=0.18, help="cell pitch in meters")
    p.add_argument("--wall-thickness", type=float, default=0.012, help="wall thickness in meters")
    p.add_argument("--wall-height", type=float, default=0.05, help="wall height in meters")
    p.add_argument(
        "--loop-fraction",
        type=float,
        default=0.15,
        help="fraction of remaining closed internal edges to additionally open, creating loops",
    )
    p.add_argument("--seed", type=int, default=None, help="random seed, for reproducible mazes")
    p.add_argument("--out-dir", type=str, default=".", help="output directory")
    p.add_argument("--name", type=str, default="maze_16x16", help="base filename for outputs")
    return p.parse_args()


class Maze:
    def __init__(self, n):
        self.n = n
        # vwalls[c][r]: vertical boundary line at x = c*cell_size, for c in 0..n,
        # spanning row r. True = wall present. c=0 and c=n are the outer
        # boundary and are never opened.
        self.vwalls = [[True] * n for _ in range(n + 1)]
        # hwalls[r][c]: horizontal boundary line at y = r*cell_size, for r in 0..n,
        # spanning col c. True = wall present. r=0 and r=n are the outer
        # boundary and are never opened.
        self.hwalls = [[True] * n for _ in range(n + 1)]

    def neighbors(self, c, r):
        out = []
        if c > 0:
            out.append((c - 1, r))
        if c < self.n - 1:
            out.append((c + 1, r))
        if r > 0:
            out.append((c, r - 1))
        if r < self.n - 1:
            out.append((c, r + 1))
        return out

    def open_between(self, c1, r1, c2, r2):
        if c2 == c1 + 1:
            self.vwalls[c1 + 1][r1] = False
        elif c2 == c1 - 1:
            self.vwalls[c1][r1] = False
        elif r2 == r1 + 1:
            self.hwalls[r1 + 1][c1] = False
        elif r2 == r1 - 1:
            self.hwalls[r1][c1] = False

    def is_open(self, c1, r1, c2, r2):
        if c2 == c1 + 1:
            return not self.vwalls[c1 + 1][r1]
        if c2 == c1 - 1:
            return not self.vwalls[c1][r1]
        if r2 == r1 + 1:
            return not self.hwalls[r1 + 1][c1]
        if r2 == r1 - 1:
            return not self.hwalls[r1][c1]
        return False


def carve_spanning_tree(maze, start, rng):
    n = maze.n
    visited = [[False] * n for _ in range(n)]
    stack = [start]
    visited[start[0]][start[1]] = True
    while stack:
        c, r = stack[-1]
        unvisited = [(nc, nr) for nc, nr in maze.neighbors(c, r) if not visited[nc][nr]]
        if not unvisited:
            stack.pop()
            continue
        nc, nr = rng.choice(unvisited)
        maze.open_between(c, r, nc, nr)
        visited[nc][nr] = True
        stack.append((nc, nr))


def add_loops(maze, fraction, rng):
    n = maze.n
    candidates = []
    for c in range(n):
        for r in range(n):
            for nc, nr in maze.neighbors(c, r):
                if (nc > c) or (nc == c and nr > r):  # consider each edge once
                    if not maze.is_open(c, r, nc, nr):
                        candidates.append((c, r, nc, nr))
    rng.shuffle(candidates)
    k = int(len(candidates) * fraction)
    for c, r, nc, nr in candidates[:k]:
        maze.open_between(c, r, nc, nr)


def open_center_goal(maze):
    n = maze.n
    cx0, cx1 = n // 2 - 1, n // 2
    cy0, cy1 = n // 2 - 1, n // 2
    cells = [(cx0, cy0), (cx1, cy0), (cx0, cy1), (cx1, cy1)]
    for i in range(len(cells)):
        for j in range(i + 1, len(cells)):
            c1, r1 = cells[i]
            c2, r2 = cells[j]
            if abs(c1 - c2) + abs(r1 - r2) == 1:
                maze.open_between(c1, r1, c2, r2)
    return cells


def wall_segments(maze, cell_size, thickness, height):
    n = maze.n
    segs = []
    for c in range(n + 1):
        for r in range(n):
            if maze.vwalls[c][r]:
                x = c * cell_size
                y = r * cell_size + cell_size / 2
                segs.append({"x": x, "y": y, "sx": thickness, "sy": cell_size + thickness, "sz": height})
    for r in range(n + 1):
        for c in range(n):
            if maze.hwalls[r][c]:
                x = c * cell_size + cell_size / 2
                y = r * cell_size
                segs.append({"x": x, "y": y, "sx": cell_size + thickness, "sy": thickness, "sz": height})
    return segs


def cell_walls(maze, c, r):
    return {
        "N": maze.hwalls[r + 1][c],
        "S": maze.hwalls[r][c],
        "E": maze.vwalls[c + 1][r],
        "W": maze.vwalls[c][r],
    }


def to_sdf(maze, segs, cell_size, wall_height, start, goal_cells, name):
    n = maze.n
    extent = n * cell_size
    collisions = []
    for i, s in enumerate(segs):
        collisions.append(f"""
      <collision name="wall_{i}_collision">
        <pose>{s['x']:.4f} {s['y']:.4f} {wall_height/2:.4f} 0 0 0</pose>
        <geometry><box><size>{s['sx']:.4f} {s['sy']:.4f} {s['sz']:.4f}</size></box></geometry>
      </collision>
      <visual name="wall_{i}_visual">
        <pose>{s['x']:.4f} {s['y']:.4f} {wall_height/2:.4f} 0 0 0</pose>
        <geometry><box><size>{s['sx']:.4f} {s['sy']:.4f} {s['sz']:.4f}</size></box></geometry>
        <material>
          <ambient>0.85 0.1 0.1 1</ambient>
          <diffuse>0.85 0.1 0.1 1</diffuse>
        </material>
      </visual>""")
    collisions_xml = "".join(collisions)

    start_x = start[0] * cell_size + cell_size / 2
    start_y = start[1] * cell_size + cell_size / 2

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="default_physics" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>

    <light type="directional" name="sun">
      <pose>0 0 5 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="maze_floor">
      <static>true</static>
      <pose>{extent/2:.4f} {extent/2:.4f} 0 0 0 0</pose>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry><box><size>{extent:.4f} {extent:.4f} 0.01</size></box></geometry>
        </collision>
        <visual name="floor_visual">
          <geometry><box><size>{extent:.4f} {extent:.4f} 0.01</size></box></geometry>
          <material>
            <ambient>0.95 0.95 0.92 1</ambient>
            <diffuse>0.95 0.95 0.92 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="micromouse_maze">
      <static>true</static>
      <link name="walls">{collisions_xml}
      </link>
    </model>

    <!-- Start cell center: ({start[0]}, {start[1]}) -> ({start_x:.4f}, {start_y:.4f}, 0) -->
    <!-- Goal cells (center 2x2): {goal_cells} -->
  </world>
</sdf>
"""


def to_json(maze, cell_size, wall_thickness, wall_height, start, goal_cells):
    n = maze.n
    cells = [[cell_walls(maze, c, r) for c in range(n)] for r in range(n)]
    return {
        "size": n,
        "cell_size": cell_size,
        "wall_thickness": wall_thickness,
        "wall_height": wall_height,
        "start": list(start),
        "goal_cells": [list(g) for g in goal_cells],
        "cells": cells,
    }


def main():
    args = build_args()
    rng = random.Random(args.seed)
    maze = Maze(args.size)
    start = (0, 0)
    carve_spanning_tree(maze, start, rng)
    if args.loop_fraction > 0:
        add_loops(maze, args.loop_fraction, rng)
    goal_cells = open_center_goal(maze)

    segs = wall_segments(maze, args.cell_size, args.wall_thickness, args.wall_height)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    sdf_text = to_sdf(maze, segs, args.cell_size, args.wall_height, start, goal_cells, args.name)
    sdf_path = out_dir / f"{args.name}.sdf"
    sdf_path.write_text(sdf_text)

    json_data = to_json(maze, args.cell_size, args.wall_thickness, args.wall_height, start, goal_cells)
    json_path = out_dir / f"{args.name}.json"
    json_path.write_text(json.dumps(json_data, indent=2))

    print(f"Wrote {sdf_path} ({len(segs)} wall segments)")
    print(f"Wrote {json_path}")


if __name__ == "__main__":
    main()
