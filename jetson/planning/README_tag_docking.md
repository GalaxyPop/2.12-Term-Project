# Tag docking controller

Module: `jetson/planning/tag_docker.py`
Keypose defs: `jetson/planning/tag_keypose.py`
Runner: `jetson/tools/park_to_tag_6.py`
Tests: `jetson/tests/test_tag_docker.py`
Gains: `jetson/config/gains.yaml` → `tag_docking:`

## Overview

`TagDocker` drives the robot from wherever it currently sees a target AprilTag to a pose defined relative to that tag — the "keypose" — using nothing but live `TagDetection`s. No global planner, no costmap, no EKF. It is intended for the final parking leg of a task (e.g. tray pickup at tag 6) after a coarse approach has put the tag in the camera frame.

One instance = one mission leg. Construct it with the target tag id, the camera-to-base extrinsic, a `TagKeyPose`, and the `tag_docking` gains dict. Call `step(detections, now)` at the control-loop rate and command the returned `(v, omega)`.

## The keypose spec

`TagKeyPose` is expressed in the **camera optical frame**:

- `target_depth_m`     — desired tag z in cam-optical (forward distance from the camera to the tag face).
- `target_lateral_m`   — desired tag x in cam-optical (horizontal offset in the image; 0 = centered).
- `target_yaw_rad`     — desired angle of the tag normal off the optical axis (0 = square-on).

The placeholder `TAG_6_PICKUP_PLACEHOLDER` is `(depth=0.30, lateral=0, yaw=0)` — tag centered horizontally in the camera, 30 cm ahead, square-on.

### Mapping to the base frame

Given the extrinsic in `robot.yaml` (camera mounted at `(0.15, 0, 0.20)` with `rpy = (-π/2, 0, -π/2)`), the optical-axis spec maps to a base-frame target of:

```
x_tag_base_target = cam_x_offset + target_depth = 0.15 + 0.30 = 0.45 m
y_tag_base_target = -target_lateral            = 0 m
psi_normal_target = pi                         (tag normal pointing back at base -X)
```

The controller computes this mapping in `_compute_target_in_base`, so arbitrary keyposes (non-zero lateral, off-axis yaw) work without code changes — edit the `TagKeyPose`, not the controller.

## Control law

Classical polar-coordinate go-to-pose regulator (Siciliano *Robotics* §11.6).

At each tick the controller:

1. Computes tag pose in base frame: `T_base_tag = T_base_cam @ det.T_cam_tag`.
2. Extracts tag center `(x_t, y_t)` and the angle `ψ_n` of the tag's out-of-face normal in base XY.
3. Derives the goal pose for the robot: sit on the tag normal at the target standoff, facing back at the tag:
   ```
   x_g = x_t + standoff · cos(ψ_n)
   y_g = y_t + standoff · sin(ψ_n)
   θ_g = ψ_n + π
   ```
4. Transforms the robot (origin in base) into the goal-aligned frame, yielding `(x_r, y_r, θ_r)`.
5. Applies the polar law:
   ```
   ρ = √(x_r² + y_r²)
   α = atan2(-y_r, -x_r) - θ_r        (heading error toward goal)
   β = -θ_r - α                       (orientation error at goal)

   v     = k_ρ · ρ
   omega = k_α · α + k_β · β
   ```
6. If `|α| > π/2` the target is behind: negates `v`, shifts `α`/`β` by π, so the robot parks in reverse instead of looping around.
7. Tapers `v` linearly below `slow_radius_m` for a gentle final approach, then saturates to `v_max_park` / `omega_max_park`.

**Stability requirement** (Siciliano): `k_ρ > 0`, `k_β < 0`, `k_α + 5/3·k_β − 2/π·k_ρ > 0`. The shipped defaults `(k_ρ, k_α, k_β) = (0.30, 0.80, −0.15)` satisfy it with margin.

## Termination

- **PARKED** — base-frame errors `|ex|, |ey| < tol_xy_m`, `|eψ| < tol_yaw_rad`, AND commanded `|v| < stop_v`, `|ω| < stop_omega`, sustained for `parked_debounce_frames` consecutive ticks. The debounce prevents premature latching when the controller is still slewing through the tolerance window.
- **LOST** — no detection of the target tag id for more than `tag_lost_timeout_s`. A single dropped frame does not trigger LOST; the controller holds `(0, 0)` during the grace window.
- **DOCKING** — every tick where neither PARKED nor LOST applies.

The runner exits 0 on PARKED and 1 on LOST / timeout, always emitting `(0, 0)` and an estop on the way out.

## How to run

Unit simulation (Jetson or any box with `numpy` + `pytest`):
```
python -m pytest jetson/tests/test_tag_docker.py -v
```

Live park (robot powered, RealSense connected, ESP32 on `/dev/ttyACM0`):
```
python -m jetson.tools.park_to_tag_6 --serial /dev/ttyACM0
```

Camera-only dry run (logs `(v, omega)` and error but doesn't drive):
```
python -m jetson.tools.park_to_tag_6 --dry-run
```

## Tuning guide

| Symptom                                           | Knob to nudge                        |
|---------------------------------------------------|--------------------------------------|
| Overshoots tag, bounces back                      | `k_rho` ↓  *or*  `slow_radius_m` ↑   |
| Rotates hunting around target heading             | `k_alpha` ↓  *or*  `omega_max_park` ↓ |
| Reaches position but never settles on yaw         | `k_beta` more negative (e.g. −0.25)  |
| Curves wide instead of heading straight at goal   | `k_alpha` ↑                          |
| PARKED latches while robot is visibly still moving| `stop_v` / `stop_omega` ↓, or `parked_debounce_frames` ↑ |
| Gives up too easily on brief occlusion            | `tag_lost_timeout_s` ↑               |
| Final pose outside gripper tolerance              | `tol_xy_m` / `tol_yaw_rad` ↓ (make controller keep trying longer) |

Always re-run the simulation tests after a gain change — the stability inequality must still hold.

## Known limits

- **Camera-relative, not gripper-relative.** The placeholder parks the *camera* 30 cm from the tag face. When the gripper geometry is finalized, replace `TAG_6_PICKUP_PLACEHOLDER` with a `TagKeyPose` whose `target_depth_m` / `target_lateral_m` place the *gripper* at the tray, accounting for the camera-to-gripper offset.
- **No obstacle awareness.** This controller does not consult `SafetyLogic` or the depth stop. If someone walks into the final-approach corridor, the robot will push through. Wrap the runner in the safety event once `ObstacleDetector` is restored.
- **No search behavior on LOST.** LOST stops the robot and exits. If the preprogrammed lane doesn't reliably leave the tag in view, add a search sweep (small yaw oscillation) before handing off to the docker.
- **Assumes flat ground.** The projection from `T_base_tag` to 2D assumes base and tag +Y (up) are aligned. Don't invoke during ramp traversal.

## Future work

- Wire into `FSM.ALIGN_TO_TAG` once `fsm.tick()` is implemented.
- Add a "drive-and-record" calibration tool for additional keyposes around the arena.
- Integrate the `SafetyLogic.should_stop()` gate so a human in the approach corridor halts the park.
- Handle reverse-parking keyposes (e.g. back-in docking) — the `reverse` branch is in place but unvalidated on hardware.
