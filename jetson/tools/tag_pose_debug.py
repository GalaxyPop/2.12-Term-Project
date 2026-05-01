#!/usr/bin/env python3
"""Standalone AprilTag detection test for Jetson + RealSense D435.

Self-contained — depends only on pyrealsense2, pupil_apriltags, opencv-python,
and numpy. Does NOT import the rest of the jetson/ package, so it works even
if other modules aren't finished.

NOTE: this project's camera is a D435 (no IMU). For the full robot the chassis
BNO08x on the ESP32 is the authoritative source of orientation. There is no
camera-side IMU code here.

What it does:
  - Opens the D435 at the resolution from camera.yaml (or CLI flags)
  - Reads factory color intrinsics directly from the SDK (no chessboard)
  - Runs pupil_apriltags.tag36h11 with estimate_tag_pose=True
  - Draws each detection (outline, id, xyz translation, yaw)
  - Prints the pose matrix T_cam_tag to stdout at ~2 Hz
  - Optional: shows the aligned depth frame as a colormap side-by-side

Usage:
    python3 jetson/tools/tag_pose_debug.py
    python3 jetson/tools/tag_pose_debug.py --depth
    python3 jetson/tools/tag_pose_debug.py --tag-size 0.10 --family tag36h11

Keyboard:
    q / ESC   quit
    s         save the current frame to /tmp/tag_debug_<stamp>.png
    p         pause (freeze the display; detection keeps running)
"""
from __future__ import annotations

import argparse
import sys
import time
from threading import Event, Lock, Thread
from typing import Optional

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--color-w", type=int, default=1280)
    ap.add_argument("--color-h", type=int, default=720)
    ap.add_argument("--depth-w", type=int, default=848)
    ap.add_argument("--depth-h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--tag-size", type=float, default=0.10,
                    help="Tag side length in meters (default 10 cm).")
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--quad-decimate", type=float, default=1.0)
    ap.add_argument("--quad-sigma", type=float, default=0.0)
    ap.add_argument("--refine-edges", type=int, default=1)
    ap.add_argument("--min-margin", type=float, default=25.0,
                    help="Reject detections below this decision_margin.")
    ap.add_argument("--depth", action="store_true",
                    help="Also display depth colormap beside the color frame.")
    ap.add_argument("--print-rate", type=float, default=2.0,
                    help="Console pose printout rate in Hz (default 2).")
    return ap.parse_args()


# ---------------- RealSense reader thread ----------------

class RSReader(Thread):
    def __init__(self, args):
        super().__init__(daemon=True)
        import pyrealsense2 as rs
        self.rs = rs
        self.args = args
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color,
                          args.color_w, args.color_h, rs.format.bgr8, args.fps)
        cfg.enable_stream(rs.stream.depth,
                          args.depth_w, args.depth_h, rs.format.z16, args.fps)
        profile = self.pipeline.start(cfg)

        # Set high-accuracy depth preset if available
        ds = profile.get_device().first_depth_sensor()
        opt_range = ds.get_option_range(rs.option.visual_preset)
        for i in range(int(opt_range.min), int(opt_range.max) + 1):
            if ds.get_option_value_description(rs.option.visual_preset, i).lower().startswith("high"):
                try:
                    ds.set_option(rs.option.visual_preset, i)
                except Exception:
                    pass
                break

        self.align = rs.align(rs.stream.color)
        cs = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = cs.get_intrinsics()

        self._stop = Event()
        self._lock = Lock()
        self._latest_color: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_stamp: float = 0.0

    @property
    def camera_params(self):
        i = self.intrinsics
        return (i.fx, i.fy, i.ppx, i.ppy)

    def run(self):
        while not self._stop.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except Exception as e:
                print(f"[rs] wait_for_frames failed: {e}", file=sys.stderr)
                continue
            aligned = self.align.process(frames)
            cf = aligned.get_color_frame()
            df = aligned.get_depth_frame()
            if not cf or not df:
                continue
            c = np.asanyarray(cf.get_data())
            d = np.asanyarray(df.get_data())
            stamp = cf.get_timestamp() * 1e-3
            with self._lock:
                self._latest_color = c
                self._latest_depth = d
                self._latest_stamp = stamp

    def latest(self):
        with self._lock:
            if self._latest_color is None:
                return None
            return (self._latest_color.copy(),
                    self._latest_depth.copy(),
                    self._latest_stamp)

    def stop(self):
        self._stop.set()
        try:
            self.pipeline.stop()
        except Exception:
            pass


# ---------------- Drawing ----------------

def draw_detection(img, det, color=(0, 255, 0)):
    corners = det.corners.astype(int)
    for i in range(4):
        cv2.line(img, tuple(corners[i]), tuple(corners[(i + 1) % 4]), color, 2)
    cx, cy = int(det.center[0]), int(det.center[1])
    t = det.pose_t.ravel()
    R = det.pose_R
    yaw_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

    cv2.putText(img, f"id:{det.tag_id}", (cx - 30, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(img,
                f"xyz=({t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f})m yaw={yaw_deg:+5.1f}d",
                (corners[0, 0], corners[0, 1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)


def draw_hud(img, fps, n_tags, paused):
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 36), (0, 0, 0), -1)
    cv2.putText(img, f"{fps:5.1f} fps   tags:{n_tags}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if paused:
        cv2.putText(img, "PAUSED", (w - 130, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def depth_to_colormap(depth_mm: np.ndarray) -> np.ndarray:
    d = np.clip(depth_mm, 0, 4000).astype(np.float32)
    d = (d / 4000.0 * 255.0).astype(np.uint8)
    return cv2.applyColorMap(d, cv2.COLORMAP_JET)


# ---------------- Main ----------------

def main():
    args = parse_args()

    try:
        import pupil_apriltags as apriltag
    except ImportError:
        print("Missing dependency: pip install pupil-apriltags", file=sys.stderr)
        return 1

    print("Opening RealSense pipeline...")
    cam = RSReader(args)
    cam.start()
    fx, fy, cx, cy = cam.camera_params
    print(f"Color intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print(f"Factory distortion: {list(cam.intrinsics.coeffs)}")

    detector = apriltag.Detector(
        families=args.family,
        nthreads=1,
        quad_decimate=args.quad_decimate,
        quad_sigma=args.quad_sigma,
        refine_edges=args.refine_edges,
        decode_sharpening=0.25,
        debug=0,
    )

    print(f"Detector ready (family={args.family}, tag_size={args.tag_size} m).")
    print("Press q or ESC to quit, s to save a frame, p to pause.")

    fps_alpha = 0.9
    fps = 0.0
    last_t = time.time()
    last_print = 0.0
    paused = False

    win = "AprilTag Debug (Jetson + D435)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    try:
        while True:
            got = cam.latest()
            if got is None:
                time.sleep(0.005)
                continue
            color, depth, stamp = got

            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[fx, fy, cx, cy],
                tag_size=args.tag_size,
            )
            detections = [d for d in detections if d.decision_margin >= args.min_margin]

            now = time.time()
            instant_fps = 1.0 / max(now - last_t, 1e-6)
            last_t = now
            fps = fps_alpha * fps + (1 - fps_alpha) * instant_fps if fps > 0 else instant_fps

            if now - last_print > 1.0 / max(args.print_rate, 0.1):
                last_print = now
                if not detections:
                    print(f"[{stamp:10.3f}] no tags")
                for d in detections:
                    t = d.pose_t.ravel()
                    R = d.pose_R
                    yaw_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                    rng = float(np.linalg.norm(t))
                    print(f"[{stamp:10.3f}] id={d.tag_id:3d}  "
                          f"range={rng:5.2f} m  "
                          f"t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})  "
                          f"yaw={yaw_deg:+6.1f} deg  "
                          f"margin={d.decision_margin:5.1f}")

            disp = color.copy()
            for d in detections:
                draw_detection(disp, d)
            draw_hud(disp, fps, len(detections), paused)

            if args.depth:
                dmap = depth_to_colormap(depth)
                if dmap.shape[:2] != disp.shape[:2]:
                    dmap = cv2.resize(dmap, (disp.shape[1], disp.shape[0]))
                combined = np.hstack([disp, dmap])
                cv2.imshow(win, combined)
            else:
                cv2.imshow(win, disp)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("s"):
                path = f"/tmp/tag_debug_{int(stamp)}.png"
                cv2.imwrite(path, disp)
                print(f"Saved {path}")
            elif key == ord("p"):
                paused = not paused
                while paused:
                    key2 = cv2.waitKey(50) & 0xFF
                    if key2 == ord("p"):
                        paused = False
                    elif key2 in (ord("q"), 27):
                        return 0
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main() or 0)
