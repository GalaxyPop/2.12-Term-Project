"""Serial packet definitions between Jetson and ESP32.

JSON over newline-delimited UTF-8 (same as lab1_2026). Upgrade to COBS
binary only if latency becomes an issue.

Commands (Jetson -> ESP32):
    {"cmd":"vel",     "v": 0.25, "w": 0.8, "seq": 1234}
    {"cmd":"arm",     "x": 0.18, "y": 0.12}
    {"cmd":"gripper", "state": "close"}        # "open" | "close"
    {"cmd":"estop"}
    {"cmd":"ramp",    "enable": true, "tray_level": true}  # arm ESP32 IMU-leveling

Telemetry (ESP32 -> Jetson, 50 Hz):
    {"t": 12345678, "x": 0.51, "y": 0.33, "th": 1.57,
     "vm": 0.24, "wm": 0.79, "roll": 0.04, "flags": 0}
    flags bits: 0=estopped, 1=watchdog_tripped, 2=low_batt, 3=imu_fault
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag, auto


class TelemFlags(IntFlag):
    NONE = 0
    ESTOPPED = auto()
    WATCHDOG_TRIPPED = auto()
    LOW_BATTERY = auto()
    IMU_FAULT = auto()


@dataclass
class Telemetry:
    t_ms: int
    x: float
    y: float
    theta: float
    v_meas: float
    w_meas: float
    roll: float
    flags: int

    @classmethod
    def from_json(cls, d: dict) -> "Telemetry":
        return cls(
            t_ms=int(d["t"]),
            x=float(d["x"]), y=float(d["y"]), theta=float(d["th"]),
            v_meas=float(d["vm"]), w_meas=float(d["wm"]),
            roll=float(d.get("roll", 0.0)),
            flags=int(d.get("flags", 0)),
        )


@dataclass
class VelCmd:
    v: float
    w: float
    seq: int = 0

    def to_json(self) -> dict:
        return {"cmd": "vel", "v": self.v, "w": self.w, "seq": self.seq}


@dataclass
class ArmCmd:
    x: float
    y: float

    def to_json(self) -> dict:
        return {"cmd": "arm", "x": self.x, "y": self.y}


@dataclass
class GripperCmd:
    state: str   # "open" | "close"

    def to_json(self) -> dict:
        return {"cmd": "gripper", "state": self.state}


@dataclass
class RampModeCmd:
    enable: bool
    tray_level: bool

    def to_json(self) -> dict:
        return {"cmd": "ramp", "enable": self.enable, "tray_level": self.tray_level}
