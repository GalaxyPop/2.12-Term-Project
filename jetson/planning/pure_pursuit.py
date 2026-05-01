"""Pure Pursuit path follower for a differential-drive robot.

Inputs: current (x, y, theta), world-frame path [(x, y), ...], gains.
Outputs: (v, omega, done) where v is linear m/s and omega is rad/s.

The controller is a thin wrapper around the classical geometry: transform
the lookahead point into the robot frame, compute curvature = 2*y/L^2,
scale forward velocity down on tight turns.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np


def pure_pursuit(
    pose: np.ndarray,
    path: Sequence[tuple[float, float]],
    lookahead_m: float,
    v_max: float,
    v_min_on_turn: float,
    turn_slowdown: float,
    goal_tolerance_m: float,
) -> tuple[float, float, bool]:
    if not path:
        return 0.0, 0.0, True
    px, py, th = pose
    pts = np.asarray(path, dtype=float)

    dists = np.hypot(pts[:, 0] - px, pts[:, 1] - py)
    closest_idx = int(np.argmin(dists))

    # Check if we're at the goal.
    if dists[-1] < goal_tolerance_m:
        return 0.0, 0.0, True

    # Walk forward from the closest index until we're >= lookahead away.
    i = closest_idx
    while i < len(pts) - 1 and dists[i] < lookahead_m:
        i += 1
    tx, ty = pts[i]

    # Transform target into robot frame.
    dx, dy = tx - px, ty - py
    cth, sth = np.cos(-th), np.sin(-th)
    xr = cth * dx - sth * dy
    yr = sth * dx + cth * dy
    L2 = xr * xr + yr * yr
    if L2 < 1e-6:
        return 0.0, 0.0, True
    curvature = 2.0 * yr / L2

    v = v_max * max(v_min_on_turn / max(v_max, 1e-6),
                    1.0 - turn_slowdown * abs(curvature))
    omega = curvature * v
    return float(v), float(omega), False
