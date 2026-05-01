"""Flat enum-based FSM.

Each state maps 1:1 onto a function in src/robot/robot_pseudo_functions.cpp.
The FSM runs on the Jetson at `tick_rate_hz`, reads the fused pose from the
EKF, consults the obstacle Event, and produces one of:
  - a "drive this path" decision (hand off to Pure Pursuit)
  - a "send this low-level command" decision (arm pose, gripper, ramp mode)
  - a state transition
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


class State(Enum):
    IDLE = auto()
    # --- sensing / alignment ---
    SEEK_TAG = auto()               # mirror findAndAlignAprilTag(tag_id)
    ALIGN_TO_TAG = auto()           # fine adjustments after seeing target tag
    # --- flat driving ---
    DRIVE_TO_WAYPOINT = auto()      # mirror executePathPlanTo(...)
    # --- ramp primitives ---
    TRAVERSE_RAMP_UP_EMPTY = auto()
    TRAVERSE_RAMP_DOWN_EMPTY = auto()
    TRAVERSE_RAMP_UP_LOADED = auto()   # enables ESP32 IMU tray-leveling
    TRAVERSE_RAMP_DOWN_LOADED = auto()
    # --- manipulation ---
    POSITION_FOR_GRIP = auto()
    GRAB_TRAY = auto()
    DEPOSIT_TRAY = auto()
    # --- system ---
    RECOVER = auto()                # generic recovery: back up, re-localize
    ERROR = auto()                  # unrecoverable; operator intervention
    DONE = auto()


@dataclass
class MissionStep:
    state: State
    goal_waypoint: Optional[str] = None   # key into arena.yaml waypoints
    tag_id: Optional[int] = None          # for SEEK_TAG / ALIGN_TO_TAG
    loaded: bool = False                  # for ramp states
    note: str = ""


# The 39-step challenge, declared in order. Each entry becomes an FSM
# transition. The script from robot_pseudo_functions.cpp translates to:
MISSION_39_STEPS: list[MissionStep] = [
    MissionStep(State.SEEK_TAG, tag_id=1, note="ramp_A base marker"),
    MissionStep(State.DRIVE_TO_WAYPOINT, goal_waypoint="base_of_slope_A"),
    # steps 4-12: 3x empty traversal cycles
    *[MissionStep(s, loaded=False) for _ in range(3) for s in (
        State.TRAVERSE_RAMP_UP_EMPTY,
        State.TRAVERSE_RAMP_DOWN_EMPTY,
    )],
    # ... (full mission to be expanded as we implement each state)
    # TODO: flesh out steps 13-39 from robot_pseudo_functions.cpp
]


@dataclass
class FSM:
    state: State = State.IDLE
    step_idx: int = 0
    tick_id: int = 0
    mission: list[MissionStep] = field(default_factory=lambda: MISSION_39_STEPS)

    def current_goal_waypoint(self) -> Optional[str]:
        if self.step_idx >= len(self.mission):
            return None
        return self.mission[self.step_idx].goal_waypoint

    def current_state_desired(self) -> State:
        if self.step_idx >= len(self.mission):
            return State.DONE
        return self.mission[self.step_idx].state

    def needs_replan(self, pose, goal_xy, deviation_threshold: float = 0.15) -> bool:
        """TODO: return True when pose strays from current path."""
        return False

    def tick(self, pose, blocked: bool, imu) -> None:
        """Advance the FSM one step based on current sensor state.

        This is where each state's entry/exit logic lives. For now it's a stub —
        implement state-by-state as we port robot_pseudo_functions.cpp.
        """
        self.tick_id += 1
        # TODO: per-state logic, transitions, timeouts, recovery.

    def mark_goal_reached(self) -> None:
        """Called by the controller when Pure Pursuit reports done=True."""
        self.step_idx = min(self.step_idx + 1, len(self.mission))
