"""Tag-relative parking controller.

Drives a differential-drive robot to a pose defined relative to an AprilTag,
using live detections from `jetson.perception.apriltag_detector.TagDetector`.
The target is expressed in the camera optical frame (see `TagKeyPose`); the
controller converts it to a base-frame goal using the camera-to-base extrinsic
and runs a polar-coordinate go-to-pose feedback law.

Expected usage is one `TagDocker` instance per mission leg, ticked at the
control-loop rate (e.g. 20 Hz). `step()` consumes the full detection list
from `TagDetector.latest()` each tick and returns the commanded (v, omega).

See `README_tag_docking.md` for the derivation and tuning guide.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Sequence

import numpy as np

from jetson.planning.tag_keypose import TagKeyPose


class DockerStatus(Enum):
    DOCKING = auto()
    PARKED = auto()
    LOST = auto()


@dataclass
class DockerOutput:
    v: float
    omega: float
    status: DockerStatus
    err: tuple[float, float, float]   # (ex, ey, e_psi) in base frame


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _compute_target_in_base(
    keypose: TagKeyPose, T_base_cam: np.ndarray
) -> tuple[float, float, float]:
    """Target tag pose (x, y, psi_normal) in base frame for a given keypose.

    - x, y     : tag center position in base XY (meters)
    - psi_normal: angle of the tag's out-of-face normal in base XY (radians)
    """
    t_cam = np.array(
        [keypose.target_lateral_m, 0.0, keypose.target_depth_m, 1.0]
    )
    t_base = T_base_cam @ t_cam

    c, s = np.cos(keypose.target_yaw_rad), np.sin(keypose.target_yaw_rad)
    n_cam = np.array([s, 0.0, -c])
    n_base = T_base_cam[:3, :3] @ n_cam
    psi_n = float(np.arctan2(n_base[1], n_base[0]))
    return float(t_base[0]), float(t_base[1]), psi_n


def _extract_tag_in_base(T_base_tag: np.ndarray) -> tuple[float, float, float]:
    """Tag center (x, y) and normal yaw psi_n in base XY."""
    x_t = float(T_base_tag[0, 3])
    y_t = float(T_base_tag[1, 3])
    n = T_base_tag[:3, :3] @ np.array([0.0, 0.0, 1.0])
    psi_n = float(np.arctan2(n[1], n[0]))
    return x_t, y_t, psi_n


class TagDocker:
    """Polar-coordinate parking controller for a single target tag."""

    def __init__(
        self,
        target_tag_id: int,
        T_base_camOpt: np.ndarray,
        keypose: TagKeyPose,
        gains: dict,
    ):
        if keypose.tag_id != target_tag_id:
            raise ValueError(
                f"target_tag_id={target_tag_id} does not match "
                f"keypose.tag_id={keypose.tag_id}"
            )
        self.tag_id = target_tag_id
        self.T_base_cam = np.asarray(T_base_camOpt, dtype=float)
        self.keypose = keypose
        self.gains = gains

        self._target_base = _compute_target_in_base(keypose, self.T_base_cam)
        # Signed standoff distance of the robot origin from the tag along the
        # tag normal direction when parked. Equals sqrt(x_t^2 + y_t^2) for a
        # placeholder that centers the tag laterally.
        self._standoff = float(
            np.hypot(self._target_base[0], self._target_base[1])
        )
        self._last_seen_ts: Optional[float] = None
        self._parked_streak = 0

    def step(self, detections: Sequence, now: float) -> DockerOutput:
        det = next((d for d in detections if d.tag_id == self.tag_id), None)

        if det is None:
            return self._handle_no_detection(now)

        self._last_seen_ts = now

        T_base_tag = self.T_base_cam @ det.T_cam_tag
        x_t, y_t, psi_n = _extract_tag_in_base(T_base_tag)

        # Goal pose for the robot in base frame: sit on the tag normal at the
        # target standoff, facing back at the tag.
        x_g = x_t + self._standoff * np.cos(psi_n)
        y_g = y_t + self._standoff * np.sin(psi_n)
        theta_g = _wrap(psi_n + np.pi)

        # Robot pose in the goal-aligned frame. Robot is at (0,0,0) in base,
        # so T_goal_robot = inv(T_base_goal).
        cos_g, sin_g = np.cos(theta_g), np.sin(theta_g)
        x_r = -(cos_g * x_g + sin_g * y_g)
        y_r = sin_g * x_g - cos_g * y_g
        theta_r = _wrap(-theta_g)

        rho = float(np.hypot(x_r, y_r))
        # Siciliano polar-coord controller with goal at origin, heading 0.
        if rho < 1e-6:
            alpha = 0.0
        else:
            alpha = _wrap(np.arctan2(-y_r, -x_r) - theta_r)
        beta = _wrap(-theta_r - alpha)

        reverse = abs(alpha) > np.pi / 2
        if reverse:
            alpha = _wrap(alpha + np.pi)
            beta = _wrap(beta + np.pi)

        k_rho = self.gains["k_rho"]
        k_alpha = self.gains["k_alpha"]
        k_beta = self.gains["k_beta"]
        v = k_rho * rho
        omega = k_alpha * alpha + k_beta * beta
        if reverse:
            v = -v

        slow_r = self.gains["slow_radius_m"]
        if rho < slow_r and slow_r > 0.0:
            v *= rho / slow_r

        v_max = self.gains["v_max_park"]
        w_max = self.gains["omega_max_park"]
        v = float(np.clip(v, -v_max, v_max))
        omega = float(np.clip(omega, -w_max, w_max))

        ex = x_t - self._target_base[0]
        ey = y_t - self._target_base[1]
        e_psi = _wrap(psi_n - self._target_base[2])
        err = (ex, ey, e_psi)

        tol_xy = self.gains["tol_xy_m"]
        tol_yaw = self.gains["tol_yaw_rad"]
        stop_v = self.gains["stop_v"]
        stop_w = self.gains["stop_omega"]
        within_tol = (
            abs(ex) < tol_xy
            and abs(ey) < tol_xy
            and abs(e_psi) < tol_yaw
        )
        low_cmd = abs(v) < stop_v and abs(omega) < stop_w

        if within_tol and low_cmd:
            self._parked_streak += 1
        else:
            self._parked_streak = 0

        if self._parked_streak >= self.gains["parked_debounce_frames"]:
            return DockerOutput(0.0, 0.0, DockerStatus.PARKED, err)
        return DockerOutput(v, omega, DockerStatus.DOCKING, err)

    def _handle_no_detection(self, now: float) -> DockerOutput:
        zero_err = (0.0, 0.0, 0.0)
        timeout = self.gains["tag_lost_timeout_s"]
        if self._last_seen_ts is None or (now - self._last_seen_ts) > timeout:
            return DockerOutput(0.0, 0.0, DockerStatus.LOST, zero_err)
        # Momentary dropout: hold still and hope the next frame recovers.
        return DockerOutput(0.0, 0.0, DockerStatus.DOCKING, zero_err)
