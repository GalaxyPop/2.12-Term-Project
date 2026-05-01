"""Depth-ROI moving-obstacle detector.

Class-agnostic (we don't know what the obstacle will be). Raises an Event
whenever more than `min_pixel_count` pixels fall inside a near-field
trapezoidal ROI with depth in (depth_min_mm, depth_max_mm).

Runs at ~30 Hz. Hysteresis: must stay clear for N consecutive frames
before un-setting `detected`, to ride out frame-level jitter.
"""
from __future__ import annotations

from threading import Event, Thread
from typing import Optional

import numpy as np


class ObstacleDetector(Thread):
    """Sets `detected` Event when something enters the robot's forward cone."""

    def __init__(self, source, gains_cfg: dict):
        super().__init__(daemon=True)
        self.source = source
        cfg = gains_cfg["obstacle"]
        self.depth_min_mm = int(cfg["depth_min_mm"])
        self.depth_max_mm = int(cfg["depth_max_mm"])
        self.min_pixel_count = int(cfg["min_pixel_count"])
        self.hysteresis_clear_frames = int(cfg["hysteresis_clear_frames"])
        self._roi_cfg = cfg["roi_trapezoid"]
        self._roi_mask: Optional[np.ndarray] = None   # built lazily per depth shape
        self._stop = Event()
        self.detected = Event()
        self._clear_streak = 0

    def _build_roi_mask(self, h: int, w: int) -> np.ndarray:
        """Trapezoidal boolean mask (HxW bool)."""
        # TODO: construct trapezoid vertices from the fraction config and
        # cv2.fillPoly into a uint8 mask; cache for the image size.
        raise NotImplementedError

    def run(self) -> None:
        """Loop: read depth -> apply ROI mask -> threshold -> set/clear event."""
        # pseudocode:
        #   while not self._stop.is_set():
        #       frame = self.source.latest()
        #       if frame is None: continue
        #       d = frame.depth
        #       if self._roi_mask is None or self._roi_mask.shape != d.shape:
        #           self._roi_mask = self._build_roi_mask(*d.shape)
        #       in_roi = self._roi_mask
        #       near   = (d > self.depth_min_mm) & (d < self.depth_max_mm)
        #       n = int(np.count_nonzero(in_roi & near))
        #       if n >= self.min_pixel_count:
        #           self.detected.set(); self._clear_streak = 0
        #       else:
        #           self._clear_streak += 1
        #           if self._clear_streak >= self.hysteresis_clear_frames:
        #               self.detected.clear()
        raise NotImplementedError

    def stop(self) -> None:
        self._stop.set()
