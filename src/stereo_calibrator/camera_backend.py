from __future__ import annotations

from dataclasses import dataclass
from typing import Union

import cv2


@dataclass(frozen=True)
class CameraMode:
    width: int
    height: int
    fps: float
    fourcc: str

    def describe(self) -> str:
        return f"{self.fourcc} {self.width}x{self.height}@{self.fps:g}"


def _decode_fourcc(value: float) -> str:
    code = int(round(value))
    return "".join(chr((code >> (8 * offset)) & 0xFF) for offset in range(4)).rstrip("\x00")


def actual_camera_mode(cap: cv2.VideoCapture) -> CameraMode:
    return CameraMode(
        width=int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))),
        height=int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))),
        fps=float(cap.get(cv2.CAP_PROP_FPS)),
        fourcc=_decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC)),
    )


def open_linux_camera(device: str, mode: CameraMode) -> cv2.VideoCapture:
    """Open a V4L2 device and reject any silent format fallback."""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"无法打开 V4L2 相机 {device}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
    cap.set(cv2.CAP_PROP_FPS, mode.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(3):
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            raise RuntimeError(f"{device} 已打开但无法读取画面")

    actual = actual_camera_mode(cap)
    exact = (
        actual.width == mode.width
        and actual.height == mode.height
        and abs(actual.fps - mode.fps) <= 1.0
        and actual.fourcc == mode.fourcc
    )
    if not exact:
        cap.release()
        raise RuntimeError(
            f"相机模式不匹配：请求 {mode.describe()}，实际 {actual.describe()}。"
            "为避免错误标定，不允许静默降级。"
        )
    return cap


def open_platform_camera(
    device: Union[str, int], mode: CameraMode, platform_name: str
) -> cv2.VideoCapture:
    if platform_name.startswith("linux"):
        return open_linux_camera(str(device), mode)
    if platform_name == "darwin":
        cap = cv2.VideoCapture(int(device), cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"无法打开 AVFoundation 摄像头索引 {device}")
        return cap
    raise RuntimeError(f"不支持的相机平台: {platform_name}")
