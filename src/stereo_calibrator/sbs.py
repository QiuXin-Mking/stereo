from __future__ import annotations

from typing import Tuple

import numpy as np


def split_sbs(frame: np.ndarray, swap_eyes: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """Split an equal-width side-by-side frame without resizing it."""
    if frame is None or frame.ndim not in (2, 3):
        raise ValueError("frame must be a grayscale or color image")
    width = frame.shape[1]
    if width < 2 or width % 2:
        raise ValueError("SBS frame width must be even")
    midpoint = width // 2
    left = frame[:, :midpoint].copy()
    right = frame[:, midpoint:].copy()
    if swap_eyes:
        return right, left
    return left, right

