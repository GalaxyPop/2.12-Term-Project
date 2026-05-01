"""Kinematic simulator: replaces SerialLink + ESP32 for off-hardware testing.

Runs a unicycle model in a background thread. Consumes v/w commands via the
same interface as SerialLink; produces Telemetry with pose derived from the
sim state (plus optional Gaussian noise).
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from typing import Optional

import numpy as np

from jetson.comms.packets import Telemetry


@dataclass
class SimState:
    x: float = -0.3
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0


class FakeRobot:
    """Drop-in replacement for SerialLink for sim/replay testing."""

    def __init__(self, dt: float = 0.01):
        self.dt = dt
        self.state = SimState()
        self._lock = Lock()
        self._stop = Event()
        self._thread: Optional[Thread] = None

    def start(self) -> None:
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        import time
        while not self._stop.is_set():
            with self._lock:
                s = self.state
                s.x += s.v * np.cos(s.theta) * self.dt
                s.y += s.v * np.sin(s.theta) * self.dt
                s.theta += s.w * self.dt
            time.sleep(self.dt)

    def send_vel(self, v: float, w: float) -> None:
        with self._lock:
            self.state.v = v
            self.state.w = w

    def send_arm(self, x: float, y: float) -> None: pass
    def send_gripper(self, state: str) -> None: pass
    def send_ramp_mode(self, enable: bool, tray_level: bool = True) -> None: pass
    def send_estop(self) -> None: self.send_vel(0.0, 0.0)

    def latest_odom(self) -> Telemetry:
        with self._lock:
            s = self.state
            return Telemetry(
                t_ms=0, x=s.x, y=s.y, theta=s.theta,
                v_meas=s.v, w_meas=s.w, roll=0.0, flags=0,
            )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
