"""Build a 2D inflated occupancy costmap from arena.yaml.

Cell values:
    0    = free
    1-99 = inflated / higher cost near obstacles
    100+ = lethal (robot center cannot enter)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Costmap:
    grid: np.ndarray           # HxW uint8, y-major (row = y, col = x)
    resolution_m: float        # meters per cell
    origin_xy: tuple[float, float]  # world (x, y) of cell (0, 0)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        r = int(round((y - self.origin_xy[1]) / self.resolution_m))
        c = int(round((x - self.origin_xy[0]) / self.resolution_m))
        return (r, c)

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        x = self.origin_xy[0] + c * self.resolution_m
        y = self.origin_xy[1] + r * self.resolution_m
        return (x, y)

    def in_bounds(self, r: int, c: int) -> bool:
        h, w = self.grid.shape
        return 0 <= r < h and 0 <= c < w


def build_costmap(arena_cfg: dict, planning_cfg: dict) -> Costmap:
    """Rasterize arena rectangles + inflate by inflation_radius_m."""
    # TODO:
    #   1. Determine grid size from arena.size + an arena-origin offset.
    #      Let arena span x in [-length_x/4, 3*length_x/4] centered on ramp A;
    #      y in [-width_y/2, +width_y/2]. (Revisit once we confirm origin.)
    #   2. Create HxW uint8 zeros.
    #   3. For each obstacle rect, set cells inside to 100.
    #   4. Inflate: distance transform, then map distances < inflation_radius
    #      to monotonically decreasing costs (e.g., 99 * exp(-d/sigma)).
    #   5. Return Costmap with origin_xy consistent with step 1.
    raise NotImplementedError
