from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .camera_backend import CameraMode
from .sbs import split_sbs


WORLD_INTELLIGENT_LABEL = "world intelligent"
WORLD_INTELLIGENT_SIZE = (4000, 1200)
WORLD_INTELLIGENT_BAND_WIDTH = 160


@dataclass(frozen=True)
class CameraProfile:
    label: str
    mode: CameraMode
    per_eye_size: tuple[int, int]
    code_band_status: str
    split_kind: str


def has_world_intelligent_code_band(frame: np.ndarray) -> bool:
    if frame is None or frame.ndim != 3 or frame.shape[1] < 320:
        return False
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    band_dark = float(np.mean(gray[:, :WORLD_INTELLIGENT_BAND_WIDTH] < 12))
    adjacent_dark = float(
        np.mean(gray[:, WORLD_INTELLIGENT_BAND_WIDTH : 2 * WORLD_INTELLIGENT_BAND_WIDTH] < 12)
    )
    return band_dark >= 0.35 and band_dark - adjacent_dark >= 0.25


def detect_camera_profile(
    device_name: str, frame: np.ndarray, mode: CameraMode
) -> CameraProfile:
    is_name = "decxin camera" in device_name.casefold()
    is_size = (
        frame is not None
        and frame.shape[:2] == (WORLD_INTELLIGENT_SIZE[1], WORLD_INTELLIGENT_SIZE[0])
        and (mode.width, mode.height) == WORLD_INTELLIGENT_SIZE
    )
    if is_name and is_size:
        if not has_world_intelligent_code_band(frame):
            raise RuntimeError("world intelligent 码带识别失败")
        return CameraProfile(
            WORLD_INTELLIGENT_LABEL,
            mode,
            (1920, 1200),
            "通过（160 px）",
            "world",
        )
    return CameraProfile(
        "generic stereo",
        mode,
        (mode.width // 2, mode.height),
        "不适用",
        "equal",
    )


def split_profile_frame(
    frame: np.ndarray, profile: CameraProfile, swap_eyes: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    if profile.split_kind == "world":
        left = frame[:, 160:2080].copy()
        right = frame[:, 2080:4000].copy()
        if left.shape[:2] != (1200, 1920) or right.shape[:2] != (1200, 1920):
            raise RuntimeError("world intelligent 裁剪尺寸错误")
        return (right, left) if swap_eyes else (left, right)
    return split_sbs(frame, swap_eyes)
