import numpy as np


# Safety modes
SAFETY_NORMAL = "normal"        # emergency depth + human detection
SAFETY_HUMAN_ONLY = "human_only"  # only human detection
SAFETY_OFF = "off"              # no safety stop


# ---------- depth emergency settings ----------
VERY_CLOSE_MM = 160
VERY_CLOSE_MIN_PIXELS = 2000

ROI_X1_FRAC = 0.15
ROI_X2_FRAC = 0.85
ROI_Y1_FRAC = 0.35
ROI_Y2_FRAC = 0.98


def very_close_depth_obstacle(depth_mm):
    """
    Returns:
        emergency_stop: bool
        close_count: int
        roi_box: (x1, y1, x2, y2)
    """
    h, w = depth_mm.shape

    x1 = int(ROI_X1_FRAC * w)
    x2 = int(ROI_X2_FRAC * w)
    y1 = int(ROI_Y1_FRAC * h)
    y2 = int(ROI_Y2_FRAC * h)

    roi = depth_mm[y1:y2, x1:x2]

    close = (roi > 0) & (roi < VERY_CLOSE_MM)
    close_count = int(np.count_nonzero(close))

    emergency_stop = close_count > VERY_CLOSE_MIN_PIXELS

    return emergency_stop, close_count, (x1, y1, x2, y2)


def should_stop(depth_mm, human_detected, safety_mode=SAFETY_NORMAL):
    """
    Combines depth emergency stop + YOLO human detection.
    """

    emergency_stop, close_count, roi_box = very_close_depth_obstacle(depth_mm)

    if safety_mode == SAFETY_NORMAL:
        stop = emergency_stop or human_detected

    elif safety_mode == SAFETY_HUMAN_ONLY:
        stop = human_detected

    elif safety_mode == SAFETY_OFF:
        stop = False

    else:
        raise ValueError(f"Unknown safety_mode: {safety_mode}")

    if emergency_stop:
        reason = "VERY CLOSE OBJECT"
    elif human_detected:
        reason = "HUMAN CLOSE"
    else:
        reason = "CLEAR"

    return stop, reason, close_count, roi_box
