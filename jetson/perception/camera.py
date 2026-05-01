"""RealSense D435 source. Thread-safe single-producer.

Replaces the cv2.VideoCapture(0) pattern from lab8_2026/apriltag_pose.py.
Factory intrinsics are exposed directly; no chessboard calibration needed.

Depth and color share one pipeline (with rs.align to color).

NOTE: this project uses a D435 (no IMU). Orientation for the EKF comes from
the ESP32 BNO08x via telemetry. If a D435i is swapped in later, IMU support
can be reintroduced here, but for this project there is no camera-side IMU.
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from typing import Optional

import numpy as np


@dataclass
class Frame:
    color: np.ndarray    # HxWx3 uint8 BGR
    depth: np.ndarray    # HxW uint16, aligned to color, units = mm
    stamp: float         # seconds (RealSense hardware timestamp)


class RealSenseSource:
    """Pulls aligned color+depth at configured rate.

    Typical usage:
        cam = RealSenseSource(camera_cfg)
        cam.start()
        fx, fy, cx, cy = cam.camera_params
        f = cam.latest()
        ...
        cam.stop()
    """

    def __init__(self, camera_cfg: dict):
        self.cfg = camera_cfg

        # Set at start()
        self._rs = None
        self._pipeline = None
        self._align = None
        self._intrinsics = None
        self._filters = []

        # Shared state
        self._latest: Optional[Frame] = None
        self._lock = Lock()
        self._stop = Event()
        self._thread: Optional[Thread] = None

    # ---------------- public API ----------------

    def start(self) -> None:
        import pyrealsense2 as rs
        self._rs = rs

        self._pipeline = rs.pipeline()
        rscfg = rs.config()

        c = self.cfg["streams"]["color"]
        d = self.cfg["streams"]["depth"]
        fmt_color = getattr(rs.format, c["format"])
        fmt_depth = getattr(rs.format, d["format"])
        rscfg.enable_stream(rs.stream.color, c["width"], c["height"], fmt_color, c["fps"])
        rscfg.enable_stream(rs.stream.depth, d["width"], d["height"], fmt_depth, d["fps"])

        profile = self._pipeline.start(rscfg)

        depth_sensor = profile.get_device().first_depth_sensor()
        preset_idx = self._resolve_preset(depth_sensor,
                                          self.cfg.get("depth_preset", "high_accuracy"))
        if preset_idx is not None:
            depth_sensor.set_option(rs.option.visual_preset, preset_idx)

        color_prof = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._intrinsics = color_prof.get_intrinsics()

        self._align = rs.align(rs.stream.color)
        self._filters = self._build_filters()

        self._thread = Thread(target=self._run, daemon=True, name="RealSense-color-depth")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        except Exception:
            pass

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    @property
    def camera_params(self) -> tuple[float, float, float, float]:
        i = self._intrinsics
        if i is None:
            raise RuntimeError("Call start() first.")
        return (i.fx, i.fy, i.ppx, i.ppy)

    @property
    def intrinsics(self) -> dict:
        i = self._intrinsics
        if i is None:
            raise RuntimeError("Call start() first.")
        return {
            "fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy,
            "coeffs": list(i.coeffs),
            "width": i.width, "height": i.height,
            "model": str(i.model),
        }

    # ---------------- internals ----------------

    def _resolve_preset(self, depth_sensor, preset_name: str) -> Optional[int]:
        rs = self._rs
        opt_range = depth_sensor.get_option_range(rs.option.visual_preset)
        want = preset_name.lower().replace(" ", "_").replace("-", "_")
        for i in range(int(opt_range.min), int(opt_range.max) + 1):
            desc = depth_sensor.get_option_value_description(rs.option.visual_preset, i)
            if desc.lower().replace(" ", "_") == want:
                return i
        return None

    def _build_filters(self) -> list:
        rs = self._rs
        fcfg = self.cfg.get("depth_filters", {}) or {}
        filters: list = []

        if fcfg.get("decimation", {}).get("enabled", False):
            f = rs.decimation_filter()
            f.set_option(rs.option.filter_magnitude, fcfg["decimation"]["magnitude"])
            filters.append(f)

        if fcfg.get("threshold", {}).get("enabled", False):
            f = rs.threshold_filter()
            f.set_option(rs.option.min_distance, fcfg["threshold"]["min_m"])
            f.set_option(rs.option.max_distance, fcfg["threshold"]["max_m"])
            filters.append(f)

        need_disparity = (fcfg.get("spatial", {}).get("enabled", False)
                          or fcfg.get("temporal", {}).get("enabled", False))
        if need_disparity:
            filters.append(rs.disparity_transform(True))

            if fcfg.get("spatial", {}).get("enabled", False):
                f = rs.spatial_filter()
                f.set_option(rs.option.filter_magnitude, fcfg["spatial"]["magnitude"])
                f.set_option(rs.option.filter_smooth_alpha, fcfg["spatial"]["alpha"])
                f.set_option(rs.option.filter_smooth_delta, fcfg["spatial"]["delta"])
                filters.append(f)

            if fcfg.get("temporal", {}).get("enabled", False):
                f = rs.temporal_filter()
                f.set_option(rs.option.filter_smooth_alpha, fcfg["temporal"]["alpha"])
                f.set_option(rs.option.filter_smooth_delta, fcfg["temporal"]["delta"])
                filters.append(f)

            filters.append(rs.disparity_transform(False))

        if fcfg.get("hole_filling", {}).get("enabled", False):
            f = rs.hole_filling_filter()
            f.set_option(rs.option.holes_fill, fcfg["hole_filling"]["mode"])
            filters.append(f)

        return filters

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                continue
            aligned = self._align.process(frames)
            cf = aligned.get_color_frame()
            df = aligned.get_depth_frame()
            if not cf or not df:
                continue
            for flt in self._filters:
                df = flt.process(df)
            color_np = np.asanyarray(cf.get_data())
            depth_np = np.asanyarray(df.get_data())
            stamp = cf.get_timestamp() * 1e-3
            with self._lock:
                self._latest = Frame(color_np, depth_np, stamp)
