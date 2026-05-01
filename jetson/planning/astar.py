"""A* on a 2D occupancy costmap. Octile neighbors, diagonal cost 1.414.

Returns a list of (x_m, y_m) waypoints in world frame, or None if no path.
"""
from __future__ import annotations

import heapq
from typing import Optional

import numpy as np

from jetson.planning.costmap import Costmap


def astar_cells(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
                lethal: int = 100, diag_cost: float = 1.4142) -> Optional[list[tuple[int, int]]]:
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w): return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w): return None
    if grid[start] >= lethal or grid[goal] >= lethal: return None

    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, diag_cost), (-1, 1, diag_cost),
            (1, -1, diag_cost), (1, 1, diag_cost)]

    g: dict[tuple[int, int], float] = {start: 0.0}
    parent: dict[tuple[int, int], Optional[tuple[int, int]]] = {start: None}
    pq: list[tuple[float, tuple[int, int]]] = [(0.0, start)]

    while pq:
        _, u = heapq.heappop(pq)
        if u == goal:
            path = []
            while u is not None:
                path.append(u)
                u = parent[u]
            return path[::-1]
        for dr, dc, base in nbrs:
            v = (u[0] + dr, u[1] + dc)
            if not (0 <= v[0] < h and 0 <= v[1] < w): continue
            cell_cost = grid[v]
            if cell_cost >= lethal: continue
            step = base * (1.0 + cell_cost / 50.0)
            ng = g[u] + step
            if ng < g.get(v, 1e18):
                g[v] = ng
                parent[v] = u
                hcost = float(np.hypot(v[0] - goal[0], v[1] - goal[1]))
                heapq.heappush(pq, (ng + hcost, v))
    return None


def astar_world(cm: Costmap, start_xy: tuple[float, float],
                goal_xy: tuple[float, float]) -> Optional[list[tuple[float, float]]]:
    s = cm.world_to_cell(*start_xy)
    g = cm.world_to_cell(*goal_xy)
    cells = astar_cells(cm.grid, s, g)
    if cells is None:
        return None
    return [cm.cell_to_world(r, c) for (r, c) in cells]
