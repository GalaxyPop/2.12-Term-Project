"""Standalone runner: park the robot at the AprilTag-6 keypose.

Wires together the existing perception stack (camera + tag detector) with
`TagDocker` and the serial link to the ESP32. Does not depend on `main.py`
or the FSM; safe to run while those are stubbed or broken.

Run with:
    python -m jetson.tools.park_to_tag_6 [--serial /dev/ttyACM0]
                                         [--config jetson/config]
                                         [--max-time-s 30]
                                         [--dry-run]

Exits 0 on PARKED, 1 on LOST / timeout / error. Always sends (0, 0) on exit.
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

import yaml

from jetson.comms.serial_link import SerialLink
from jetson.localization.tf_tree import make_transform
from jetson.perception.apriltag_detector import TagDetector
from jetson.perception.camera import RealSenseSource
from jetson.planning.tag_docker import DockerStatus, TagDocker
from jetson.planning.tag_keypose import TAG_6_PICKUP_PLACEHOLDER


LOOP_HZ = 20.0
TARGET_TAG_ID = 6


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=Path("jetson/config"))
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--max-time-s", type=float, default=30.0,
                    help="Safety cap on runtime (seconds).")
    ap.add_argument("--dry-run", action="store_true",
                    help="Skip serial link; log commands but don't drive.")
    ap.add_argument("--log-hz", type=float, default=5.0,
                    help="Rate of on-screen status prints.")
    return ap.parse_args()


def load_configs(config_dir: Path) -> dict:
    return {
        "arena":  yaml.safe_load((config_dir / "arena.yaml").read_text()),
        "robot":  yaml.safe_load((config_dir / "robot.yaml").read_text()),
        "gains":  yaml.safe_load((config_dir / "gains.yaml").read_text()),
        "camera": yaml.safe_load((config_dir / "camera.yaml").read_text()),
    }


def build_T_base_cam(robot_cfg: dict):
    cm = robot_cfg["camera_mount"]
    return make_transform(cm["translation_m"], cm["rotation_rpy"])


def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("park_to_tag_6")

    args = parse_args()
    cfg = load_configs(args.config)

    T_base_cam = build_T_base_cam(cfg["robot"])
    docker = TagDocker(
        target_tag_id=TARGET_TAG_ID,
        T_base_camOpt=T_base_cam,
        keypose=TAG_6_PICKUP_PLACEHOLDER,
        gains=cfg["gains"]["tag_docking"],
    )

    cam = RealSenseSource(cfg["camera"])
    cam.start()
    tag_det = TagDetector(cam, cfg["arena"])
    tag_det.start()

    link = None
    if not args.dry_run:
        link = SerialLink(args.serial, args.baud)
        link.start()

    # Make sure we always send a final stop on any exit path.
    def _sigint(_sig, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint)

    exit_code = 1
    dt = 1.0 / LOOP_HZ
    log_period = 1.0 / max(args.log_hz, 0.1)
    next_log = 0.0
    start_t = time.time()

    log.info("Docking to tag %d; dry_run=%s; max_time=%.1fs",
             TARGET_TAG_ID, args.dry_run, args.max_time_s)

    try:
        while True:
            tic = time.time()
            elapsed = tic - start_t
            if elapsed > args.max_time_s:
                log.error("Timeout after %.1fs without PARKED", elapsed)
                break

            out = docker.step(tag_det.latest(), now=tic)

            if link is not None:
                link.send_vel(out.v, out.omega)

            if tic >= next_log:
                next_log = tic + log_period
                ex, ey, epsi = out.err
                log.info("t=%5.2f  v=%+.3f  w=%+.3f  err=(%+.3f, %+.3f, %+5.1fdeg)  %s",
                         elapsed, out.v, out.omega,
                         ex, ey, epsi * 180.0 / 3.141592653589793,
                         out.status.name)

            if out.status == DockerStatus.PARKED:
                log.info("PARKED at t=%.2fs", elapsed)
                exit_code = 0
                break
            if out.status == DockerStatus.LOST:
                log.error("LOST tag %d after %.2fs", TARGET_TAG_ID, elapsed)
                break

            sleep_for = dt - (time.time() - tic)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        log.warning("Interrupted by user")
    finally:
        try:
            if link is not None:
                link.send_vel(0.0, 0.0)
                link.send_estop()
                link.stop()
        finally:
            tag_det.stop()
            cam.stop()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
