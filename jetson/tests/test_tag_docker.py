"""Offline convergence tests for TagDocker.

Simulates a stationary AprilTag in the world, integrates unicycle kinematics
driven by the controller, and asserts that the robot parks within tolerance
across a sweep of starting poses. No hardware, no camera, no serial link.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from jetson.localization.tf_tree import (
    invert_transform,
    make_transform_xyz_rpy,
    pose_to_matrix_2d,
)
from jetson.planning.tag_docker import DockerStatus, TagDocker
from jetson.planning.tag_keypose import TAG_6_PICKUP_PLACEHOLDER


# Camera extrinsic matches jetson/config/robot.yaml defaults.
T_BASE_CAM = make_transform_xyz_rpy(
    0.15, 0.00, 0.20, -1.5708, 0.0, -1.5708
)

# Test arena: single vertical tag at world origin, face pointing +Y.
T_WORLD_TAG = make_transform_xyz_rpy(
    0.0, 0.0, 0.30, -1.5708, 0.0, 0.0
)

GAINS = {
    "k_rho": 0.30,
    "k_alpha": 0.80,
    "k_beta": -0.15,
    "v_max_park": 0.10,
    "omega_max_park": 0.8,
    "slow_radius_m": 0.10,
    "tol_xy_m": 0.02,
    "tol_yaw_rad": 0.052,
    "stop_v": 0.02,
    "stop_omega": 0.10,
    "parked_debounce_frames": 5,
    "tag_lost_timeout_s": 1.0,
}

SIM_DT = 0.02           # 50 Hz integration
SIM_MAX_T = 20.0        # seconds of simulated time
CONTROL_PERIOD = 0.05   # 20 Hz controller


@dataclass
class FakeDetection:
    tag_id: int
    T_cam_tag: np.ndarray


def _synthesize_detection(robot_pose_world, tag_id=6):
    """Build a TagDetection-like stub for the given robot pose."""
    xr, yr, thr = robot_pose_world
    T_world_robot = pose_to_matrix_2d(xr, yr, thr)
    T_world_cam = T_world_robot @ T_BASE_CAM
    T_cam_world = invert_transform(T_world_cam)
    T_cam_tag = T_cam_world @ T_WORLD_TAG
    return FakeDetection(tag_id=tag_id, T_cam_tag=T_cam_tag)


def _run_sim(start_pose, gains=None):
    """Integrate unicycle dynamics under TagDocker control. Return final pose,
    last output, and the time at which PARKED was first reached (or None).
    """
    g = gains if gains is not None else GAINS
    docker = TagDocker(
        target_tag_id=6,
        T_base_camOpt=T_BASE_CAM,
        keypose=TAG_6_PICKUP_PLACEHOLDER,
        gains=g,
    )

    pose = np.array(start_pose, dtype=float)
    v, omega = 0.0, 0.0
    t = 0.0
    next_ctrl = 0.0
    parked_at = None
    last_out = None

    while t < SIM_MAX_T:
        if t >= next_ctrl:
            det = _synthesize_detection(tuple(pose))
            last_out = docker.step([det], now=t)
            v, omega = last_out.v, last_out.omega
            if last_out.status == DockerStatus.PARKED and parked_at is None:
                parked_at = t
                # Continue integrating briefly to confirm we stay stopped.
                if t > 1.0:
                    break
            next_ctrl += CONTROL_PERIOD

        pose[0] += v * np.cos(pose[2]) * SIM_DT
        pose[1] += v * np.sin(pose[2]) * SIM_DT
        pose[2] = (pose[2] + omega * SIM_DT + np.pi) % (2 * np.pi) - np.pi
        t += SIM_DT

    return pose, last_out, parked_at


# Target parked pose in world: robot at (0, 0.45) facing -Y.
TARGET_POSE = np.array([0.0, 0.45, -np.pi / 2])


def _assert_parked(pose, out, parked_at, scenario):
    assert out is not None, f"[{scenario}] no controller output"
    assert parked_at is not None, (
        f"[{scenario}] never parked; final err={out.err}, status={out.status}"
    )
    dx = pose[0] - TARGET_POSE[0]
    dy = pose[1] - TARGET_POSE[1]
    dth = (pose[2] - TARGET_POSE[2] + np.pi) % (2 * np.pi) - np.pi
    assert abs(dx) < 0.03, f"[{scenario}] x err {dx:.3f} m exceeds 3 cm"
    assert abs(dy) < 0.03, f"[{scenario}] y err {dy:.3f} m exceeds 3 cm"
    assert abs(dth) < 0.08, f"[{scenario}] theta err {dth:.3f} rad exceeds ~4.5 deg"


def test_straight_on_approach():
    pose, out, parked_at = _run_sim((0.0, 1.0, -np.pi / 2))
    _assert_parked(pose, out, parked_at, "straight_on")


def test_lateral_offset_left():
    pose, out, parked_at = _run_sim((-0.20, 1.0, -np.pi / 2))
    _assert_parked(pose, out, parked_at, "offset_left")


def test_lateral_offset_right():
    pose, out, parked_at = _run_sim((0.20, 1.0, -np.pi / 2))
    _assert_parked(pose, out, parked_at, "offset_right")


def test_small_yaw_error():
    pose, out, parked_at = _run_sim((0.0, 1.0, -np.pi / 2 + 0.2))
    _assert_parked(pose, out, parked_at, "yaw_error")


def test_combined_offset_and_yaw():
    pose, out, parked_at = _run_sim((-0.15, 0.8, -np.pi / 2 + 0.15))
    _assert_parked(pose, out, parked_at, "combined")


def test_lost_tag_triggers_lost_status():
    docker = TagDocker(
        target_tag_id=6,
        T_base_camOpt=T_BASE_CAM,
        keypose=TAG_6_PICKUP_PLACEHOLDER,
        gains=GAINS,
    )
    # First tick: no detections at all, last_seen is None -> LOST immediately.
    out = docker.step([], now=0.0)
    assert out.status == DockerStatus.LOST
    assert out.v == 0.0 and out.omega == 0.0


def test_transient_dropout_keeps_docking():
    docker = TagDocker(
        target_tag_id=6,
        T_base_camOpt=T_BASE_CAM,
        keypose=TAG_6_PICKUP_PLACEHOLDER,
        gains=GAINS,
    )
    det = _synthesize_detection((0.0, 1.0, -np.pi / 2))
    docker.step([det], now=0.0)             # prime last_seen
    out = docker.step([], now=0.3)          # 0.3 s since last seen: still docking
    assert out.status == DockerStatus.DOCKING
    out = docker.step([], now=1.5)          # 1.5 s: exceeded timeout -> LOST
    assert out.status == DockerStatus.LOST


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
