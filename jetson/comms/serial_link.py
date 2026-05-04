"""SerialLink: Jetson <-> ESP32 JSON-over-USB-CDC, extending lab1_2026.

Responsibilities:
  - Open /dev/ttyACM0 at 921600 baud (bumped from lab1's 115200)
  - Write VelCmd/ArmCmd/GripperCmd/RampModeCmd JSONs, newline-terminated
  - Continuously read Telemetry lines on a background thread, expose latest
  - Drop malformed lines, log errors

This replaces the lab1 MotorController class. The ESP32 firmware needs a
matching JSON parser extension on the src/robot/robot_main.cpp side
(currently src/robot/jetson_drive_test.cpp only handles cmd=motor|stop|status).
"""
from __future__ import annotations

import json
import logging
import time
from threading import Event, Lock, Thread
from typing import Optional

import serial
import serial.tools.list_ports  # noqa: F401  (left available for user scripts)

from jetson.comms.packets import (
    ArmCmd,
    GripperCmd,
    RampModeCmd,
    Telemetry,
    VelCmd,
)

log = logging.getLogger(__name__)


class SerialLink:
    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 921600):
        self.port = port
        self.baud = baud
        self._ser = None            # pyserial.Serial, opened in start()
        self._latest: Optional[Telemetry] = None
        self._lock = Lock()
        self._stop = Event()
        self._reader: Optional[Thread] = None
        self._seq = 0

    def start(self) -> None:
        """Open the port, wait for ESP32 ready banner, start reader thread."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        # ESP32-S3 autoresets when the host opens /dev/ttyACM0; the first
        # ~1 s of writes would be lost without this wait.
        time.sleep(2.0)
        self._ser.reset_input_buffer()

        # Drain the "{status: ready}" banner with a short deadline.
        # Absence is non-fatal (re-opens without a physical reset skip it).
        deadline = time.time() + 3.0
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="strict").strip()
                d = json.loads(line)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if d.get("status") == "ready":
                break
        else:
            log.warning("No {status:ready} banner from ESP32 on %s", self.port)

        self._reader = Thread(target=self._run, name="serial-reader", daemon=True)
        self._reader.start()

    def _run(self) -> None:
        """Read lines, parse JSON, drop malformed, update self._latest."""
        assert self._ser is not None
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()   # blocks up to 0.1 s
            except serial.SerialException as exc:
                # Transient USB hiccup (cable wiggle); log once and keep going.
                log.warning("SerialException in reader: %s", exc)
                time.sleep(0.05)
                continue
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="strict").strip()
            except UnicodeDecodeError:
                continue
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            # Skip banners / status replies; only telemetry has an "x" key.
            if "x" not in d or "th" not in d:
                continue
            try:
                tm = Telemetry.from_json(d)
            except (KeyError, ValueError, TypeError):
                continue
            with self._lock:
                self._latest = tm

    def send_vel(self, v: float, w: float) -> None:
        self._seq += 1
        self._write(VelCmd(v=v, w=w, seq=self._seq).to_json())

    def send_arm(self, x: float, y: float) -> None:
        self._write(ArmCmd(x=x, y=y).to_json())

    def send_gripper(self, state: str) -> None:
        assert state in ("open", "close")
        self._write(GripperCmd(state=state).to_json())

    def send_ramp_mode(self, enable: bool, tray_level: bool = True) -> None:
        self._write(RampModeCmd(enable=enable, tray_level=tray_level).to_json())

    def send_estop(self) -> None:
        self._write({"cmd": "estop"})

    def _write(self, obj: dict) -> None:
        if self._ser is None:
            raise RuntimeError("SerialLink.start() not called.")
        line = (json.dumps(obj) + "\n").encode("utf-8")
        self._ser.write(line)

    def latest_odom(self) -> Optional[Telemetry]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._ser is not None:
            self._ser.close()
