"""Named keyposes defined relative to an AprilTag.

A keypose describes where the robot should end up once parked in front of a
tag, expressed in the camera optical frame:
    target_depth_m    : tag z in cam-optical (forward distance from camera)
    target_lateral_m  : tag x in cam-optical (horizontal offset; + = right)
    target_yaw_rad    : angle of tag normal off the camera optical axis
                        (0 = square-on, tag face perpendicular to cam-Z)

The tag-docking controller consumes a TagKeyPose plus the camera-to-base
extrinsic to drive the robot to the corresponding base-frame pose.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TagKeyPose:
    tag_id: int
    target_depth_m: float
    target_lateral_m: float = 0.0
    target_yaw_rad: float = 0.0


# Placeholder for dishwasher-tray pickup: tag centered horizontally in the
# camera frame, 30 cm away, square-on. Refine once gripper geometry is fixed.
TAG_6_PICKUP_PLACEHOLDER = TagKeyPose(
    tag_id=6,
    target_depth_m=0.30,
    target_lateral_m=0.0,
    target_yaw_rad=0.0,
)
