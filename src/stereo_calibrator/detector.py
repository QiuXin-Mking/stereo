from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from .models import PoseFeatures


def detect_chessboard(gray: np.ndarray, pattern: Tuple[int, int]) -> Optional[np.ndarray]:
    """Return subpixel chessboard corners as an N×2 float32 array."""
    if gray is None or gray.ndim != 2:
        raise ValueError("gray image must have one channel")
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
    if not found or corners is None:
        return None
    return corners.reshape(-1, 2).astype(np.float32)


def detect_chessboard_with_retry(
    gray: np.ndarray, pattern: Tuple[int, int]
) -> Tuple[Optional[np.ndarray], str]:
    """Detect on the original image, then retry once with CLAHE for low light."""
    corners = detect_chessboard(gray, pattern)
    if corners is not None:
        return corners, "raw"
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    corners = detect_chessboard(clahe.apply(gray), pattern)
    return (corners, "clahe") if corners is not None else (None, "none")


def pose_features(corners: np.ndarray, image_size: Tuple[int, int]) -> PoseFeatures:
    """Summarize board location, projected area, and in-plane orientation."""
    points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    if points.size == 0:
        raise ValueError("corners must not be empty")
    width, height = image_size
    if width <= 0 or height <= 0:
        raise ValueError("image_size must be positive")

    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = points.mean(axis=0)
    projected_area = float(np.prod(maximum - minimum)) / float(width * height)
    rectangle = cv2.minAreaRect(points)
    box_width, box_height = rectangle[1]
    angle = float(rectangle[2])
    if box_width < box_height:
        angle += 90.0

    covariance = np.cov(points.T) if len(points) >= 3 else np.eye(2)
    eigenvalues = np.linalg.eigvalsh(covariance)
    perspective = 0.0
    if eigenvalues[-1] > 1e-9:
        perspective = float(eigenvalues[0] / eigenvalues[-1])

    return PoseFeatures(
        center_x=float(center[0] / width),
        center_y=float(center[1] / height),
        area_ratio=projected_area,
        angle_deg=angle,
        perspective=perspective,
    )
