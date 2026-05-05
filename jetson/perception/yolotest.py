import cv2
import numpy as np
from ultralytics import YOLO


model = YOLO("yolov8n.pt")

# Smoothing state
yolo_hits = 0
yolo_misses = 0
stable_person = False


def human_in_path(
    color_frame,
    depth_mm=None,
    max_depth_mm=600,      # ignore humans farther than 1.2 m
    min_depth_mm=160,       # ignore bad/super-close noise
    conf=0.40,
    min_area=1200,
    min_aspect=0.8,
    required_hits=4,
    required_misses=9,
    draw=True,
):
    """
    Detects a person/lower-body region in the robot's forward path.

    color_frame: BGR camera image from OpenCV / RealSense color frame
    depth_mm: optional depth image aligned to color image, in millimeters
    returns: (stable_person_detected, annotated_frame)
    """

    global yolo_hits, yolo_misses, stable_person

    frame = color_frame.copy()
    height, width = frame.shape[:2]

    # Lower-center ROI: where legs/shoes/person would appear in robot path
    roi_x1 = int(0.15 * width)
    roi_x2 = int(0.85 * width)
    roi_y1 = int(0.35 * height)
    roi_y2 = int(0.95 * height)

    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

    results = model(roi, classes=[0], conf=conf, verbose=False)

    raw_person_detected = False

    for r in results:
        for box in r.boxes:
            bx1, by1, bx2, by2 = box.xyxy[0]

            box_w = bx2 - bx1
            box_h = by2 - by1
            area = box_w * box_h
            aspect = box_h / max(box_w, 1)

            if area < min_area or aspect < min_aspect:
                continue

            # Convert box from ROI coordinates to full-frame coordinates
            x1 = int(bx1 + roi_x1)
            y1 = int(by1 + roi_y1)
            x2 = int(bx2 + roi_x1)
            y2 = int(by2 + roi_y1)

            depth_ok = True
            median_depth = None

            if depth_mm is not None:
                box_depth = depth_mm[y1:y2, x1:x2]

                valid_depth = box_depth[
                    (box_depth > min_depth_mm) &
                    (box_depth < max_depth_mm)
                ]

                # If no valid close depth pixels, ignore this detection
                if len(valid_depth) == 0:
                    depth_ok = False
                else:
                    median_depth = float(np.median(valid_depth))
                    depth_ok = median_depth < max_depth_mm

            if depth_ok:
                raw_person_detected = True

                if draw:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    label = "HUMAN"
                    if median_depth is not None:
                        label += f" {median_depth/1000:.2f} m"

                    cv2.putText(
                        frame,
                        label,
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

    # Temporal smoothing
    if raw_person_detected:
        yolo_hits += 1
        yolo_misses = 0
    else:
        yolo_misses += 1
        yolo_hits = 0

    if yolo_hits >= required_hits:
        stable_person = True

    if yolo_misses >= required_misses:
        stable_person = False

    if draw:
        # Draw scan ROI
        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

        label = "HUMAN IN PATH - STOP" if stable_person else "NO HUMAN IN PATH"
        color = (0, 255, 0) if stable_person else (0, 0, 255)

        cv2.putText(
            frame,
            label,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            color,
            2,
        )

    return stable_person, frame
