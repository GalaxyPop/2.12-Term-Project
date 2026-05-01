"""Top-level Jetson main loop.

Wires together:
    perception: RealSenseSource, TagDetector, ObstacleDetector
    localization: EKF2D + TFTree + tag_detection_to_world_pose
    planning: Costmap + A* + Pure Pursuit
    behavior: FSM (flat enum, ~15 states)
    comms: SerialLink (JSON over USB-CDC to ESP32)

Run with:
    python -m jetson.main --config jetson/config
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import yaml

from jetson.behavior.fsm import FSM, State
from jetson.comms.serial_link import SerialLink
from jetson.localization.ekf import EKF2D
from jetson.localization.pose_fuser import tag_detection_to_world_pose
from jetson.localization.tf_tree import TFTree
from jetson.perception.apriltag_detector import TagDetector
from jetson.perception.camera import RealSenseSource
from jetson.perception.obstacle_detector import ObstacleDetector
from jetson.planning.astar import astar_world
from jetson.planning.costmap import build_costmap
from jetson.planning.pure_pursuit import pure_pursuit


def load_configs(config_dir: Path) -> dict:
    return {
        "arena":  yaml.safe_load((config_dir / "arena.yaml").read_text()),
        "robot":  yaml.safe_load((config_dir / "robot.yaml").read_text()),
        "gains":  yaml.safe_load((config_dir / "gains.yaml").read_text()),
        "camera": yaml.safe_load((config_dir / "camera.yaml").read_text()),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=Path("jetson/config"))
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--no-hw", action="store_true",
                    help="Skip hardware bring-up; use simulated robot.")
    args = ap.parse_args()

    cfg = load_configs(args.config)

    # --- perception ---
    cam = RealSenseSource(cfg["camera"]); cam.start()
    tag_det = TagDetector(cam, cfg["arena"]); tag_det.start()
    obs_det = ObstacleDetector(cam, cfg["gains"]); obs_det.start()

    # --- localization ---
    tf = TFTree(cfg["robot"], cfg["arena"])
    ekf = EKF2D(cfg["gains"]["ekf"])

    # --- planning ---
    costmap = build_costmap(cfg["arena"], cfg["gains"]["planning"])

    # --- behavior ---
    fsm = FSM()

    # --- comms ---
    link = SerialLink(args.serial); link.start()

    dt = 1.0 / cfg["gains"]["fsm"]["tick_rate_hz"]
    pp = cfg["gains"]["pure_pursuit"]
    waypoints = cfg["arena"]["waypoints"]
    path = None
    last_tick = time.time()

    try:
        while fsm.current_state_desired() != State.DONE:
            tic = time.time()
            elapsed = tic - last_tick
            last_tick = tic

            # 1. predict
            odom = link.latest_odom()
            if odom is not None:
                ekf.predict(odom.v_meas, odom.w_meas, elapsed)

            # 2. update from tags (SE(3) -> 2D with tilt gating)
            max_tilt = cfg["arena"]["apriltag"]["max_base_tilt_for_ekf_rad"]
            roll = odom.roll if odom is not None else 0.0
            pitch = 0.0  # ESP32 telem currently ships roll only; extend packet to add pitch
            for det in tag_det.latest():
                z = tag_detection_to_world_pose(
                    det, tf, base_roll=roll, base_pitch=pitch, max_tilt_rad=max_tilt,
                )
                if z is not None:
                    ekf.update_tag(z, det.range_m, det.stamp)

            pose = ekf.x.copy()

            # 3. FSM tick -> decides current goal and state
            fsm.tick(pose=pose, blocked=obs_det.detected.is_set(), imu=odom)

            # 4. plan (replan on demand) + local control
            goal_name = fsm.current_goal_waypoint()
            if goal_name is not None:
                g = waypoints[goal_name]
                goal_xy = (g["x"], g["y"])
                if path is None or fsm.needs_replan(pose, goal_xy):
                    path = astar_world(costmap, (pose[0], pose[1]), goal_xy)

            if obs_det.detected.is_set() or path is None:
                v, w, done = 0.0, 0.0, False
            else:
                v, w, done = pure_pursuit(
                    pose, path,
                    lookahead_m=pp["lookahead_m"],
                    v_max=pp["v_max"],
                    v_min_on_turn=pp["v_min_on_turn"],
                    turn_slowdown=pp["turn_slowdown"],
                    goal_tolerance_m=pp["goal_tolerance_m"],
                )
                if done:
                    fsm.mark_goal_reached()
                    path = None

            link.send_vel(v, w)

            # 5. pace the loop
            time.sleep(max(0.0, dt - (time.time() - tic)))
    finally:
        link.send_vel(0.0, 0.0)
        link.stop()
        tag_det.stop(); obs_det.stop(); cam.stop()


if __name__ == "__main__":
    main()
