# Camera Mount Guide — Intel RealSense D435i

This is the spec for how the RealSense D435i is physically attached to the
chassis, how we measure its pose, and how to verify it. The numbers that
come out of the measurement procedure get written into `robot.yaml` under
`camera_mount`. Everything in `jetson/localization/` and
`jetson/perception/` is downstream of these numbers, so getting them right
(to within a few mm and a few degrees) matters.

---

## 1. Why the mount matters

The Jetson's tag-localization pipeline does this chain:

    T_world_base = T_world_tag · inv(T_cam_tag) · inv(T_base_cameraOpt)

`T_cam_tag` comes from `pupil_apriltags`. `T_world_tag` comes from
`arena.yaml`. **`T_base_cameraOpt` comes entirely from this mount spec.**
A 3 cm error in camera height turns into a 3 cm systematic offset in
every localization estimate; a 2° error in camera pitch turns into
~3.5 cm of ground-plane error at a 1 m tag range. Measure carefully.

---

## 2. Where to mount it (physical requirements)

The D435i has to see:

1. **Tags on walls / tables / the dishwasher** — mounted at heights
   between ~15 cm (low floor markers, discouraged) and ~60 cm (typical
   table-mounted tag). Target operating range: 0.3 m – 4 m.
2. **Tags at the top of a ramp while climbing** — when the robot is on
   the ramp at ~18° pitch, a level-mounted camera points down at the
   ground 1 m ahead. To see the top-of-ramp tag we need either
   (a) the camera pitched UP relative to base_link to compensate, or
   (b) accept that tag reacquisition happens only before the ramp and
   after the robot crests it. **We are taking option (b)** because it
   keeps flat-ground localization accurate. Ramp traversal is open-loop
   on the ESP32 IMU; tags are re-locked after the crest.
3. **The near-field obstacle zone** — depth frustum must cover the
   ground plane from ~0.2 m to ~1 m in front of the robot, with
   >30° horizontal FOV. The D435i natively has ~86°H × 57°V depth
   FOV, so this is easy as long as the camera isn't pitched more
   than ~15° up.

### Recommended mount geometry

| Parameter                             | Recommended | Acceptable range |
|---------------------------------------|-------------|------------------|
| Height above ground (camera center)   | **0.20 m**  | 0.15 – 0.30 m    |
| Forward offset from base_link center  | **0.15 m**  | 0.05 – 0.25 m    |
| Lateral offset (y)                    | **0.00 m**  | keep centered    |
| Pitch relative to level               | **0°** (level) | -5° to +5°    |
| Roll                                  | **0°**      | must be zero     |
| Yaw relative to base_link +X          | **0°**      | must be zero     |

**Why 20 cm high, level, and forward:**
- High enough that the depth ROI covers ~0.2–0.8 m in front, which is the
  window the obstacle detector uses. A lower mount (e.g. 10 cm) works but
  shrinks the reactive stopping distance.
- Level gives you maximum tag range on vertical tags (the common case).
- Forward of base_link so the robot can still see obstacles in its
  footprint projection — if the camera sits behind base_link center, the
  near edge of the depth cone is already past the robot's nose.

**Don't do:**
- **Don't pitch the camera down** to stare at the ground. You lose wall
  tags, and the depth ROI trapezoid already handles the near-ground
  obstacle check.
- **Don't mount above ~30 cm** on a small chassis. The robot rolls/pitches
  as it climbs ramps; the higher the camera, the larger the translational
  sway, which shows up as EKF noise. (This is why we also gate tag updates
  by base tilt.)
- **Don't use the D435i's built-in IMU** for robot orientation. The BNO08x
  on the ESP32 is a 9-DoF fused sensor mounted on the chassis and is
  authoritative. The camera IMU is only useful for rolling-shutter
  compensation, which we don't need for static-tag localization.

---

## 3. Frame conventions (read this once, then never think about it again)

We follow REP-105-ish conventions:

```
base_link                   camera_link               camera_color_optical_frame
+X forward                  +X forward                +Z forward
+Y left                     +Y left                   +X right
+Z up                       +Z up                     +Y down
(ground plane, chassis)     (camera body, level       (the frame pupil_apriltags
                             when base is level)       reports pose in)
```

The static transform from `base_link` to `camera_color_optical_frame` is
the product of two rotations:

1. A translation + optional base-to-camera-body tilt (usually zero).
2. A fixed axis relabeling from body axes to optical axes:
   roll = -π/2, pitch = 0, yaw = -π/2.

When the camera body is level with base_link (our default, recommended
case), **use this exact `rotation_rpy` in `robot.yaml`**:

```yaml
camera_mount:
  translation_m: [0.15, 0.00, 0.20]
  rotation_rpy:  [-1.5708, 0.0, -1.5708]
```

If you tilt the camera body up or down by angle `α` (pitch, nose up is
positive), apply the extra pitch **before** the optical relabeling:

```
rpy_camera_mount = Rx(-π/2) · Ry(-π/2) · Ry(α)
```

We're computing this as `rpy_to_matrix(roll, pitch, yaw)` with XYZ-
intrinsic order, so in config terms a +10° (0.175 rad) upward body tilt
becomes:

```yaml
camera_mount:
  translation_m: [0.15, 0.00, 0.20]
  rotation_rpy:  [-1.5708, 0.175, -1.5708]  # pitched up 10°
```

Keep the up-tilt under ~5° unless you have a specific reason.

---

## 4. Measurement procedure

You need six numbers: `(tx, ty, tz, roll, pitch, yaw)` of
`camera_color_optical_frame` relative to `base_link`. Here's the
pragmatic approach — calipers and a tape measure, no fancy rig.

### 4a. Translation

1. With the robot on flat ground, identify **base_link's origin**: the
   midpoint between the two drive wheels, at the ground plane. Mark this
   on the floor with a pencil (tape a cross on the ground directly under
   the wheel-axis midpoint).
2. Identify the **camera optical center**. The D435i's color sensor is
   ~11 mm behind the front face of the camera body and ~17 mm to the
   right of the body's centerline. For a first pass, just use the center
   of the color lens hole — accuracy to within ±5 mm is fine.
3. Measure:
   - `tx` = forward distance from base_link mark to the point on the
     floor directly under the camera optical center, **in meters, along
     the robot's +X axis**.
   - `ty` = lateral distance (+Y = robot's left), **should be 0**. If
     it's not, your mount is crooked — straighten it now.
   - `tz` = height of the camera optical center above the floor.

Write these into `robot.yaml → camera_mount → translation_m`.

### 4b. Rotation (the easy case: camera mounted level)

If the camera body is physically level with the chassis (no mounting
tilt), use the default rotation: `[-1.5708, 0.0, -1.5708]`. Done.

### 4c. Rotation (the general case: camera pitched up/down)

If the mount pitches the camera by some angle, measure it and bake it
into the config:

1. Place a bubble level across the top of the camera body. Verify roll
   is zero. If it isn't, either shim the mount or get a new mount —
   rolled cameras are really annoying to reason about.
2. Measure pitch by placing a straight edge along the camera body's top
   surface and measuring the angle to horizontal with a digital
   inclinometer (phone apps work to ±0.5°). Positive = nose up.
3. Verify yaw is zero by dropping a plumb line from the front-center of
   the camera body to the floor; check that it lands on the robot's
   fore-aft centerline (marked from 4a.1).

Write `rotation_rpy = [-1.5708, <pitch_rad>, -1.5708]` (roll and yaw in
the *optical relabeling* stay fixed; the extra tilt goes in the pitch
slot).

---

## 5. Verification (do this before trusting anything downstream)

This is the single most valuable hour you'll spend on localization. Skip
it at your peril.

### 5a. Static tag test

1. Place a known AprilTag (ID 1, 10 cm, `tag36h11`) at a measured
   location: 1.00 m directly in front of base_link, upright on a block
   at 0.25 m height, facing the robot. So:
   `T_world_tag = (x=1.0, y=0.0, z=0.25, roll=-π/2, pitch=0, yaw=-π/2)`.
2. Add this tag to `arena.yaml` (temporarily).
3. Place the robot with base_link at world origin, facing +X.
4. Run the camera + detector (without the full main loop):
   ```bash
   python -m jetson.tools.tag_pose_debug --config jetson/config
   ```
   (Writeup to come — for now, a script that instantiates
   `RealSenseSource`, `TagDetector`, `TFTree`, and prints
   `tag_detection_to_world_pose(det, tf, 0, 0)` for every detection.)
5. **Expected output**: `(x, y, theta) ≈ (0.00, 0.00, 0.00)` with noise
   under ±2 cm and ±2°. If you're off by more than that, one of these
   is wrong:
     - Camera mount translation: check tape-measure numbers.
     - Camera mount rotation: check bubble level, check the
       `rotation_rpy` XYZ-order convention matches `rpy_to_matrix`.
     - Tag world pose in yaml: re-measure.
     - Tag physical pose: confirm with a protractor.

### 5b. Range sweep

Move the robot back along -X in 0.5 m increments. Confirm the computed
`(x, y)` increments match. If pose drifts as a function of range, the
**pitch** of the camera is almost certainly wrong (even 1° is visible
at 3 m).

### 5c. Lateral sweep

Move the robot laterally along ±Y. The computed y should track the
physical position to within ±2 cm. If it doesn't, `ty` in the translation
is wrong or the camera is yawed.

---

## 6. Special concerns for the ramps

While the robot is on a ramp, its base_link frame is pitched relative to
world. The localization pipeline handles this by **gating tag updates
when `|base_pitch| > 5°`** (configurable in `arena.yaml`:
`apriltag.max_base_tilt_for_ekf_rad`). Inside that gate, the EKF
dead-reckons from odometry only. This keeps flat-ground measurements
unbiased at the cost of no tag correction while climbing.

Consequences for mounting:
- You do **not** need to pitch the camera up to see tags during the
  climb. Those detections would be rejected anyway.
- You **do** want the camera level enough that, at the very top of the
  ramp (at the moment the robot crests and goes back to small pitch),
  the next target tag comes into frame quickly. With the recommended
  20 cm / 0° / forward mount, the tag must be roughly at the same height
  (~15–40 cm) and within ~60° of the forward axis.
- Tags placed on the **ramp surface itself** (glued flat to the slope,
  used for mid-climb servoing) only work if you relax the tilt gate AND
  account for the tag's own pitch in `arena.yaml`. This is a separate
  feature — punt on it for Stage 2 and revisit if ramp traversal needs
  closed-loop CV.

---

## 7. Field-of-view sanity check

At 20 cm height, level, with D435i color FOV ~69° × 42°:

| Distance ahead | Ground visible (near-far band) | Horizontal FOV coverage |
|----------------|---------------------------------|-------------------------|
| 0.5 m          | ~0.21 m to rest of frame        | ±0.34 m                 |
| 1.0 m          | (fully visible)                 | ±0.69 m                 |
| 2.0 m          | (fully visible)                 | ±1.37 m                 |
| 4.0 m          | (fully visible)                 | ±2.75 m                 |

The **near edge at 0.21 m** is why the obstacle ROI starts at
depth_min_mm=200 — anything closer is inside the bottom edge of the
frame or past the lens.

---

## 8. Summary: checklist before bolting it down

- [ ] Camera body is LEVEL (bubble level on top, both axes)
- [ ] Camera is CENTERED laterally (plumb line check)
- [ ] Camera is RIGID — no flex when chassis vibrates
- [ ] Cable strain relief so USB doesn't yank the mount
- [ ] `robot.yaml` camera_mount numbers match physical measurement
- [ ] Section 5a static-tag test passes to within ±2 cm / ±2°
- [ ] Section 5b range sweep confirms no pitch error
- [ ] Section 5c lateral sweep confirms no yaw error
