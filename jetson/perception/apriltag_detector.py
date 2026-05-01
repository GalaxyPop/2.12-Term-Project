"""Threaded AprilTag detector.

Direct port of lab8_2026/apriltag_pose.py into a class that:
  - pulls frames from a RealSenseSource (not cv2.VideoCapture)
  - uses factory intrinsics (not the discarded webcam .npz)
  - emits TagDetection events instead of cv2.imshow
  - carries a full SE(3) T_cam_tag (4x4), not just R, t
  - handles per-tag size overrides from arena.yaml (some ramp tags may be
    smaller than the 10 cm default)

Detector params (family=tag36h11, quad_decimate=1.0, refine_edges=1) are
identical to lab8 and come from arena.yaml.
"""
from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from threading import Event, Lock, Thread
from typing import Optional

import numpy as np

from jetson.localization.pose_fuser import detection_to_T_cam_tag


@dataclass
class TagDetection:
    tag_id: int
    T_cam_tag: np.ndarray          # 4x4 SE(3): tag pose in camera_optical frame
    center_px: tuple[float, float]
    corners_px: np.ndarray         # (4, 2) float
    decision_margin: float
    range_m: float                 # ||t||, precomputed for EKF measurement noise
    stamp: float


class TagDetector(Thread):
    """Consumes frames from a RealSenseSource, publishes latest detections."""

    def __init__(self, source, arena_cfg: dict, tf_tree=None):
        super().__init__(daemon=True)
        self.source = source
        self.apriltag_cfg = arena_cfg["apriltag"]
        self._default_size = float(self.apriltag_cfg["default_size_m"])
        # Group tag sizes so we can batch detections by tag_size argument.
        self._tag_size_by_id: dict[int, float] = {}
        for t in arena_cfg.get("tags", []):
            self._tag_size_by_id[int(t["id"])] = float(t.get("size", self._default_size))
        self._size_classes: list[float] = sorted(
            {self._default_size, *self._tag_size_by_id.values()}
        )
        self._min_margin = float(self.apriltag_cfg.get("min_decision_margin", 0.0))
        self._tf_tree = tf_tree      # optional; used to reject unknown tags early
        self._detector = None        # pupil_apriltags.Detector, built in run()
        self._latest: list[TagDetection] = []
        self._lock = Lock()
        self._stop = Event()

    def _build_detector(self):
        """Mirror lab8_2026/apriltag_pose.py lines 24-32."""
        import pupil_apriltags as apriltag
        return apriltag.Detector(
            families=self.apriltag_cfg["family"],
            nthreads=1,
            quad_decimate=self.apriltag_cfg["quad_decimate"],
            quad_sigma=self.apriltag_cfg["quad_sigma"],
            refine_edges=self.apriltag_cfg["refine_edges"],
            decode_sharpening=self.apriltag_cfg["decode_sharpening"],
            debug=0,
        )

    def _effective_size(self, tag_id: int) -> float:
        return self._tag_size_by_id.get(tag_id, self._default_size)

    def run(self) -> None:
        """Loop: pull latest frame -> detect -> store list of TagDetection.

        If multiple tag sizes exist, we run detect() once per size class,
        filtering results to only the tags that actually have that size.
        (Cheaper alternative: run once with default size, then rescale the
        translation vector per-tag with t *= real_size / default_size.)
        """
        import cv2
        self._detector = self._build_detector()
        last_stamp = -1.0
        while not self._stop.is_set():
            frame = self.source.latest()
            if frame is None or frame.stamp == last_stamp:
                Event().wait(0.005)
                continue
            last_stamp = frame.stamp

            gray = cv2.cvtColor(frame.color, cv2.COLOR_BGR2GRAY)
            camera_params = list(self.source.camera_params)

            results: list[TagDetection] = []
            for size_m in self._size_classes:
                raw = self._detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=camera_params,
                    tag_size=size_m,
                )
                for r in raw:
                    # Only accept results whose true tag_size matches the one
                    # we passed (or if we don't know the tag, accept at default).
                    effective = self._effective_size(r.tag_id)
                    if abs(effective - size_m) > 1e-6:
                        continue
                    if r.decision_margin < self._min_margin:
                        continue
                    if self._tf_tree is not None and not self._tf_tree.has_tag(r.tag_id):
                        # unknown tag id: still report (useful for debugging)
                        pass
                    T_cam_tag = detection_to_T_cam_tag(r.pose_R, r.pose_t)
                    t = T_cam_tag[:3, 3]
                    results.append(TagDetection(
                        tag_id=int(r.tag_id),
                        T_cam_tag=T_cam_tag,
                        center_px=(float(r.center[0]), float(r.center[1])),
                        corners_px=np.asarray(r.corners, dtype=float),
                        decision_margin=float(r.decision_margin),
                        range_m=float(np.linalg.norm(t)),
                        stamp=frame.stamp,
                    ))

            with self._lock:
                self._latest = results

    def latest(self) -> list[TagDetection]:
        with self._lock:
            return list(self._latest)

    def stop(self) -> None:
        self._stop.set()
