"""Minimal static transform tree, full SE(3). No ROS dependency.

Frames:
    world        : arena fixed frame (origin at ramp A base, on the ground)
    base_link    : robot center on the ground plane, +X fwd, +Y left, +Z up. 
                    (** facing the ramp**)
                   Only translates/yaws in world when on flat ground; pitches
                   and rolls non-trivially on ramps.
    camera_link  : camera body on the robot (forward-pointing, level when
                   base_link is level)
    camera_opt   : camera OPTICAL frame, +Z fwd, +X right, +Y down
                   (this is the frame pupil_apriltags reports pose in)

The chain for a tag measurement of the robot pose:

    T_world_base = T_world_tag  ·  inv(T_cam_tag)  ·  inv(T_base_cameraOpt)

We keep everything as 4×4 matrices end-to-end and only project down to
(x, y, theta) at the last step (matrix_to_pose_2d) for the EKF.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


# ---- rotation / transform utilities ----

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """XYZ intrinsic (i.e. R = Rz(yaw) · Ry(pitch) · Rx(roll))."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_transform(translation, rpy) -> np.ndarray:
    """4×4 homogeneous transform from (3,) translation and (3,) rpy."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T


def make_transform_xyz_rpy(x: float, y: float, z: float,
                            roll: float, pitch: float, yaw: float) -> np.ndarray:
    return make_transform([x, y, z], [roll, pitch, yaw])


def invert_transform(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    R = T[:3, :3]
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def pose_to_matrix_2d(x: float, y: float, theta: float) -> np.ndarray:
    """2D ground-plane pose -> 4×4 (z=0, roll=pitch=0)."""
    return make_transform([x, y, 0.0], [0.0, 0.0, theta])


def pose_to_matrix_3d(x: float, y: float, z: float,
                      roll: float, pitch: float, yaw: float) -> np.ndarray:
    return make_transform([x, y, z], [roll, pitch, yaw])


def matrix_to_pose_2d(T: np.ndarray) -> np.ndarray:
    """Project a 4×4 transform down to (x, y, theta). Drops z, roll, pitch.

    NOTE: this is only correct when the frame being projected is (approximately)
    level with the world ground plane. On a pitched ramp the base_link is
    tilted; use `matrix_to_pose_2d_with_tilt` to flatten correctly, or only
    call this flattening when you know tilt is small.
    """
    yaw = np.arctan2(T[1, 0], T[0, 0])
    return np.array([T[0, 3], T[1, 3], yaw])


def matrix_to_pose_2d_with_tilt(T_world_base: np.ndarray,
                                 base_roll: float,
                                 base_pitch: float) -> np.ndarray:
    """Extract (x, y, yaw) of the *ground projection* of base_link.

    When the robot is tilted (e.g., mid-ramp), the EKF still models 2D
    pose, but the tag-observed base_link has nonzero roll/pitch. This
    helper strips the known tilt before reading the yaw, which keeps
    yaw consistent with the robot's heading on the ground plane.

    base_roll and base_pitch should come from the ESP32 BNO08x telemetry.
    """
    # Un-tilt: T_world_base_flat = T_world_base · Rx(-roll) · Ry(-pitch)^-1
    # Easier: read yaw from the (3,3) rotation after projecting its +X
    # axis onto the world XY plane.
    R = T_world_base[:3, :3]
    x_axis = R[:, 0]
    yaw = np.arctan2(x_axis[1], x_axis[0])
    return np.array([T_world_base[0, 3], T_world_base[1, 3], yaw])


# ---- static TF tree ----

class TFTree:
    """Static transforms from robot.yaml + arena.yaml. Full SE(3)."""

    def __init__(self, robot_cfg: dict, arena_cfg: dict):
        cm = robot_cfg["camera_mount"]
        self.T_base_cameraOpt = make_transform(
            cm["translation_m"], cm["rotation_rpy"]
        )
        # Tag poses in world frame — FULL SE(3).
        # Each tag yaml entry has x, y, z, roll, pitch, yaw.
        self.T_world_tag: dict[int, np.ndarray] = {}
        self.tag_meta: dict[int, dict] = {}
        for t in arena_cfg.get("tags", []):
            tid = int(t["id"])
            self.T_world_tag[tid] = make_transform_xyz_rpy(
                t["x"], t["y"], t["z"],
                t.get("roll", 0.0), t.get("pitch", 0.0), t.get("yaw", 0.0),
            )
            self.tag_meta[tid] = dict(t)

    def tag_size(self, tag_id: int, default: float) -> float:
        m = self.tag_meta.get(tag_id)
        if m is None:
            return default
        return float(m.get("size", default))

    def has_tag(self, tag_id: int) -> bool:
        return tag_id in self.T_world_tag
