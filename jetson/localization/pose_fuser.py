"""Convert an AprilTag detection (pose in camera_optical frame) to a
world-frame 2D measurement of the robot's base_link.

Everything is done in SE(3); we only project down to (x, y, theta) at the
very end, using base tilt (from ESP32 IMU) to correctly flatten.

Chain:
    T_world_tag    : from arena.yaml, full SE(3)
    T_cam_tag      : from pupil_apriltags (uses camera_optical convention)
    T_base_camOpt  : from robot.yaml, full SE(3)

    T_world_base = T_world_tag  ·  inv(T_cam_tag)  ·  inv(T_base_camOpt)
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from jetson.localization.tf_tree import (
    TFTree,
    invert_transform,
    matrix_to_pose_2d,
    matrix_to_pose_2d_with_tilt,
)


def tag_detection_to_world_pose(
    det,
    tf: TFTree,
    base_roll: float = 0.0,
    base_pitch: float = 0.0,
    max_tilt_rad: float = 0.087,
) -> Optional[np.ndarray]:
    """Return (x, y, theta) of base_link in world, or None if:
       - tag id is not known in the world map
       - base tilt exceeds threshold (robot mid-ramp; tag yaw projection is
         unreliable and we don't want to corrupt the EKF)

    Pass `base_roll` and `base_pitch` from ESP32 BNO08x telemetry. On flat
    ground both are near zero and the gate is transparent.
    """
    T_world_tag = tf.T_world_tag.get(det.tag_id)
    if T_world_tag is None:
        return None

    if max(abs(base_roll), abs(base_pitch)) > max_tilt_rad:
        return None

    T_cam_tag = det.T_cam_tag                         # 4x4 from detector
    T_tag_cam = invert_transform(T_cam_tag)
    T_cam_base = invert_transform(tf.T_base_cameraOpt)
    T_world_base = T_world_tag @ T_tag_cam @ T_cam_base

    return matrix_to_pose_2d_with_tilt(T_world_base, base_roll, base_pitch)


def detection_to_T_cam_tag(pose_R: np.ndarray, pose_t: np.ndarray) -> np.ndarray:
    """Pack pupil_apriltags' (R, t) outputs into a 4x4 transform.

    pupil_apriltags returns pose_R as 3x3 and pose_t as 3x1; build the
    homogeneous form the detector + fuser assume.
    """
    T = np.eye(4)
    T[:3, :3] = pose_R
    T[:3, 3] = pose_t.ravel()
    return T
