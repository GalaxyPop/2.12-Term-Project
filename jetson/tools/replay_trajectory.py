"""Replay a canned 2D path with Pure Pursuit, using ESP32 odometry as pose.

Run with:
    python -m jetson.tools.replay_trajectory --path {straight,circle,uturn}

No EKF, no AprilTags. Odometry from the ESP32 is treated as the fused pose,
with the frame zeroed at the moment the script receives its first telemetry
line (so the robot doesn't need to start at (0,0,0) in absolute coords).

This exercises the full pure_pursuit -> send_vel -> wheels loop and will
drift noticeably on anything curved (known odometry bug in the ESP32 side).
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

from jetson.comms.serial_link import SerialLink
from jetson.planning.pure_pursuit import pure_pursuit

LOOP_HZ = 20.0
Path2D = List[Tuple[float, float]]


def build_straight(length_m: float = 1.0, step_m: float = 0.02) -> Path2D:
    n = max(2, int(length_m / step_m) + 1)
    return [(step_m * i, 0.0) for i in range(n)]


def build_circle(radius_m: float = 0.5, step_rad: float = math.pi / 30) -> Path2D:
    # Start at origin moving +X, arc to the left (CCW), return near origin.
    pts: Path2D = []
    for k in range(int(2 * math.pi / step_rad) + 1):
        phi = step_rad * k
        # Center at (0, radius), starting tangent +X.
        x = radius_m * math.sin(phi)
        y = radius_m - radius_m * math.cos(phi)
        pts.append((x, y))
    return pts


def build_uturn(straight_m: float = 0.5, arc_radius: float = 0.3,
                step_m: float = 0.02, step_rad: float = math.pi / 20) -> Path2D:
    pts: Path2D = []
    # Leg 1: straight forward.
    n1 = int(straight_m / step_m) + 1
    for i in range(n1):
        pts.append((step_m * i, 0.0))
    # Arc: 180 deg CCW around (straight_m, arc_radius).
    cx, cy = straight_m, arc_radius
    n2 = int(math.pi / step_rad) + 1
    for k in range(1, n2 + 1):
        phi = -math.pi / 2 + step_rad * k  # start at angle -pi/2 (south of center)
        pts.append((cx + arc_radius * math.cos(phi), cy + arc_radius * math.sin(phi)))
    # Leg 2: straight back. End of arc is (straight_m, 2*arc_radius), heading -X.
    x0, y0 = straight_m, 2 * arc_radius
    for i in range(1, n1 + 1):
        pts.append((x0 - step_m * i, y0))
    return pts


PATH_BUILDERS = {
    "straight": build_straight,
    "circle":   build_circle,
    "uturn":    build_uturn,
}


def wait_for_first_telemetry(link: SerialLink, deadline_s: float = 5.0):
    t_end = time.time() + deadline_s
    while time.time() < t_end:
        tm = link.latest_odom()
        if tm is not None:
            return tm
        time.sleep(0.05)
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", choices=sorted(PATH_BUILDERS.keys()), required=True)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--config", type=Path, default=Path("jetson/config/gains.yaml"))
    args = ap.parse_args()

    path = PATH_BUILDERS[args.path]()
    gains = yaml.safe_load(args.config.read_text())["pure_pursuit"]

    link = SerialLink(port=args.port, baud=args.baud)
    link.start()

    def _sigint(_s, _f): raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint)

    try:
        first = wait_for_first_telemetry(link)
        if first is None:
            print("No telemetry within 5s; aborting.", file=sys.stderr)
            return 1
        x0, y0, th0 = first.x, first.y, first.theta
        print(f"Zero frame: x0={x0:+.2f}, y0={y0:+.2f}, th0={th0:+.2f}")
        print(f"Path '{args.path}' has {len(path)} points; running at {LOOP_HZ:.0f} Hz.")

        cos0, sin0 = math.cos(-th0), math.sin(-th0)
        dt = 1.0 / LOOP_HZ

        while True:
            tic = time.time()
            tm = link.latest_odom()
            if tm is None:
                link.send_vel(0.0, 0.0)
                time.sleep(dt)
                continue

            # Rotate + translate raw odom into the start frame.
            rx, ry = tm.x - x0, tm.y - y0
            x_s = cos0 * rx - sin0 * ry
            y_s = sin0 * rx + cos0 * ry
            th_s = tm.theta - th0
            pose = np.array([x_s, y_s, th_s])

            v, w, done = pure_pursuit(
                pose, path,
                lookahead_m=gains["lookahead_m"],
                v_max=gains["v_max"],
                v_min_on_turn=gains["v_min_on_turn"],
                turn_slowdown=gains["turn_slowdown"],
                goal_tolerance_m=gains["goal_tolerance_m"],
            )

            link.send_vel(v, w)
            sys.stdout.write(
                f"\rpose=({x_s:+.2f},{y_s:+.2f},{th_s:+.2f})  "
                f"cmd v={v:+.2f} w={w:+.2f}  {'DONE' if done else '    '}   "
            )
            sys.stdout.flush()

            if done:
                print()
                break

            sleep_for = dt - (time.time() - tic)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        try:
            link.send_estop()
        finally:
            link.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
