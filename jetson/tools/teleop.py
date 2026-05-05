"""Keyboard teleop for the Jetson -> ESP32 serial link.

Run with:
    python -m jetson.tools.teleop [--port /dev/ttyACM0]

Controls (case-insensitive):
    w / s   : +/- linear velocity  (0.05 m/s steps)
    a / d   : +/- angular velocity (0.3 rad/s steps)
    space   : zero both
    q / C-c : estop + exit

Telemetry from the ESP32 is printed on each loop tick.
"""
from __future__ import annotations

import argparse
import select
import signal
import sys
import termios
import time
import tty
from contextlib import contextmanager

from jetson.comms.serial_link import SerialLink

LOOP_HZ = 20.0
V_STEP = 0.05         # m/s per key press
W_STEP = 0.30         # rad/s per key press
V_MAX = 0.30          # sanity clamp (robot.yaml limit)
W_MAX = 2.0


@contextmanager
def raw_terminal(fd: int):
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)   # char-at-a-time, no echo-line buffering
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def getch_nonblocking(fd: int, timeout_s: float) -> str:
    r, _, _ = select.select([fd], [], [], timeout_s)
    if not r:
        return ""
    return sys.stdin.read(1)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    link = SerialLink(port=args.port, baud=args.baud)
    link.start()

    v, w = 0.0, 0.0
    dt = 1.0 / LOOP_HZ
    fd = sys.stdin.fileno()

    # SIGINT handler — let the finally block close the link cleanly.
    def _sigint(_sig, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint)

    print("Teleop ready. w/s: v  a/d: w  space: zero  q: quit")
    print(f"Port: {args.port} @ {args.baud}")

    try:
        with raw_terminal(fd):
            while True:
                tic = time.time()
                key = getch_nonblocking(fd, timeout_s=0.0)
                if key:
                    k = key.lower()
                    if k == "w":   v = clamp(v + V_STEP, -V_MAX, V_MAX)
                    elif k == "s": v = clamp(v - V_STEP, -V_MAX, V_MAX)
                    elif k == "a": w = clamp(w + W_STEP, -W_MAX, W_MAX)
                    elif k == "d": w = clamp(w - W_STEP, -W_MAX, W_MAX)
                    elif k == " ": v, w = 0.0, 0.0
                    elif k == "q": break

                link.send_vel(v, w)
                telem = link.latest_odom()
                if telem is not None:
                    sys.stdout.write(
                        f"\rv={v:+.2f}  w={w:+.2f}  "
                        f"odom x={telem.x:+.2f} y={telem.y:+.2f} th={telem.theta:+.2f}  "
                        f"vm={telem.v_meas:+.2f} wm={telem.w_meas:+.2f}   "
                    )
                else:
                    sys.stdout.write(f"\rv={v:+.2f}  w={w:+.2f}  (no telemetry)   ")
                sys.stdout.flush()

                sleep_for = dt - (time.time() - tic)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass
    finally:
        print()  # newline after the \r-based status line
        try:
            link.send_estop()
        finally:
            link.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
