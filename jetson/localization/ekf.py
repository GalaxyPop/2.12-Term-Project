"""3-state EKF: state = (x, y, theta) in the world frame.

Predict: unicycle model driven by (v, omega) from ESP32 telemetry.
Update:  tag detection converted to a world-frame pose via tf_tree.

Measurement noise scales with tag range squared (distant tags are trusted less).
"""
from __future__ import annotations

import numpy as np


def wrap_angle(a: float) -> float:
    return np.arctan2(np.sin(a), np.cos(a))


class EKF2D:
    def __init__(self, ekf_cfg: dict):
        self.x = np.zeros(3)                                # (x, y, theta)
        self.P = np.diag(ekf_cfg["P0_diag"]).astype(float)
        self._Q_per_s = np.diag(ekf_cfg["Q_diag"]).astype(float)
        self._R_tag_1m = np.diag(ekf_cfg["R_tag_1m_diag"]).astype(float)
        self.last_tag_stamp: float = -np.inf

    def predict(self, v: float, w: float, dt: float) -> None:
        """Unicycle propagation with Jacobian-based covariance update."""
        th = self.x[2]
        self.x = self.x + np.array([v * np.cos(th) * dt,
                                    v * np.sin(th) * dt,
                                    w * dt])
        self.x[2] = wrap_angle(self.x[2])
        F = np.array([[1.0, 0.0, -v * np.sin(th) * dt],
                      [0.0, 1.0,  v * np.cos(th) * dt],
                      [0.0, 0.0,  1.0]])
        self.P = F @ self.P @ F.T + self._Q_per_s * dt

    def update_tag(self, z_world: np.ndarray, range_m: float, stamp: float) -> None:
        """z_world = (x, y, theta) in world frame derived from a tag detection."""
        R = self._R_tag_1m * max(range_m, 0.1) ** 2
        y = z_world - self.x
        y[2] = wrap_angle(y[2])
        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        self.P = (np.eye(3) - K) @ self.P
        self.last_tag_stamp = stamp

    @property
    def trace(self) -> float:
        return float(np.trace(self.P))
