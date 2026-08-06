from __future__ import annotations

from typing import Mapping, Optional, Tuple

import numpy as np


POSE_SLOT_QUOTAS = {
    "center_front": 2,
    "left": 2,
    "right": 2,
    "top": 2,
    "bottom": 2,
    "left_top": 2,
    "right_top": 2,
    "left_bottom": 2,
    "right_bottom": 2,
    "yaw_left": 2,
    "yaw_right": 2,
    "pitch_up": 2,
    "pitch_down": 2,
    "roll_cw": 2,
    "roll_ccw": 2,
    "near": 1,
    "far": 1,
}


def _available(name: str, filled: Mapping[str, int]) -> bool:
    return int(filled.get(name, 0)) < POSE_SLOT_QUOTAS[name]


def classify_pose_slot(
    corners: np.ndarray,
    pattern: Tuple[int, int],
    image_size: Tuple[int, int],
    filled: Mapping[str, int],
) -> Optional[str]:
    """Classify a detected board into one still-unfilled calibration pose slot."""
    columns, rows = pattern
    points = np.asarray(corners, np.float32).reshape(rows, columns, 2)
    width, height = image_size
    flat = points.reshape(-1, 2)
    center = flat.mean(axis=0) / np.asarray([width, height], np.float32)
    extent = flat.max(axis=0) - flat.min(axis=0)
    area_ratio = float(extent[0] * extent[1] / float(width * height))

    row_vector = points[0, -1] - points[0, 0]
    roll_deg = -float(np.degrees(np.arctan2(row_vector[1], row_vector[0])))
    left_height = float(np.linalg.norm(points[-1, 0] - points[0, 0]))
    right_height = float(np.linalg.norm(points[-1, -1] - points[0, -1]))
    top_width = float(np.linalg.norm(points[0, -1] - points[0, 0]))
    bottom_width = float(np.linalg.norm(points[-1, -1] - points[-1, 0]))
    yaw = (right_height - left_height) / max(right_height, left_height, 1.0)
    pitch = (bottom_width - top_width) / max(bottom_width, top_width, 1.0)

    candidates = []
    if abs(roll_deg) >= 12.0:
        candidates.append("roll_ccw" if roll_deg > 0 else "roll_cw")
    if abs(yaw) >= 0.12:
        candidates.append("yaw_left" if yaw > 0 else "yaw_right")
    if abs(pitch) >= 0.12:
        candidates.append("pitch_up" if pitch > 0 else "pitch_down")
    if area_ratio >= 0.22:
        candidates.append("near")
    elif area_ratio <= 0.025:
        candidates.append("far")

    horizontal = "left" if center[0] < 0.35 else "right" if center[0] > 0.65 else ""
    vertical = "top" if center[1] < 0.35 else "bottom" if center[1] > 0.65 else ""
    if horizontal and vertical:
        candidates.append(f"{horizontal}_{vertical}")
    elif horizontal:
        candidates.append(horizontal)
    elif vertical:
        candidates.append(vertical)
    else:
        candidates.append("center_front")

    for name in candidates:
        if _available(name, filled):
            return name
    return None

