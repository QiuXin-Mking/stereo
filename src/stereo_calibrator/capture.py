from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

import cv2
import numpy as np

from .detector import detect_chessboard
from .models import AcceptedPair, PoseFeatures
from .quality import evaluate_pair
from .sbs import split_sbs


def find_avfoundation_device_index(device_name: str) -> int:
    """Resolve a macOS camera name to its current AVFoundation index."""
    command = ["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    output = completed.stdout + "\n" + completed.stderr
    pattern = re.compile(r"\[(\d+)\]\s+" + re.escape(device_name) + r"\s*$", re.MULTILINE)
    match = pattern.search(output)
    if not match:
        available = re.findall(r"\[(\d+)\]\s+([^\r\n]+)", output)
        names = ", ".join(f"[{index}] {name.strip()}" for index, name in available[:20])
        raise RuntimeError(f"找不到视频设备 {device_name!r}。当前设备: {names or '无'}")
    return int(match.group(1))


def open_highest_camera(index: int, requested_size: Optional[Tuple[int, int]] = None) -> cv2.VideoCapture:
    """Open AVFoundation and request either an explicit or maximum-sized frame."""
    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if requested_size:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, requested_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_size[1])
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 99999)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 99999)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        cap.release()
        if sys.platform == "darwin":
            raise RuntimeError(
                f"无法打开 AVFoundation 摄像头索引 {index}。请到“系统设置 → 隐私与安全性 → 相机”"
                "允许当前终端或 Codex 访问相机，然后重新运行。"
            )
        raise RuntimeError(f"无法打开 AVFoundation 摄像头索引 {index}")
    for _ in range(8):
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap
    cap.release()
    raise RuntimeError(f"摄像头索引 {index} 已打开但无法读取画面")


def guidance_hint(history: Iterable[PoseFeatures]) -> str:
    occupied = set()
    for feature in history:
        column = min(2, max(0, int(feature.center_x * 3)))
        row = min(2, max(0, int(feature.center_y * 3)))
        occupied.add((row, column))
    labels = [
        ["左上", "上方", "右上"],
        ["左侧", "中央", "右侧"],
        ["左下", "下方", "右下"],
    ]
    for row, column in ((1, 1), (0, 0), (0, 2), (2, 0), (2, 2), (0, 1), (1, 0), (1, 2), (2, 1)):
        if (row, column) not in occupied:
            return f"将棋盘移到{labels[row][column]}，轻微倾斜并保持稳定"
    return "改变棋盘距离或倾角，补充不同姿态"


def _preview(left: np.ndarray, right: np.ndarray, width: int) -> np.ndarray:
    height = max(1, int(left.shape[0] * width / left.shape[1]))
    return np.hstack(
        [cv2.resize(left, (width, height), interpolation=cv2.INTER_AREA),
         cv2.resize(right, (width, height), interpolation=cv2.INTER_AREA)]
    )


def _delete_pair(pair: AcceptedPair) -> None:
    for path in (pair.left_path, pair.right_path, pair.raw_path):
        if path is not None and path.exists():
            path.unlink()
    metadata = pair.left_path.parent / f"{pair.index:04d}_metadata.json"
    if metadata.exists():
        metadata.unlink()


def capture_session(
    config: Mapping[str, object],
    session_dir: Path,
    square_size_m: float,
    swap_eyes: bool = False,
) -> List[AcceptedPair]:
    """Run guided automatic SBS capture and persist accepted source frames."""
    device_cfg = dict(config["device"])
    board_cfg = dict(config["board"])
    capture_cfg = dict(config["capture"])
    quality_cfg = dict(config["quality"])
    index = int(device_cfg.get("index", -1))
    if index < 0:
        index = find_avfoundation_device_index(str(device_cfg["name"]))

    requested_size = None
    if device_cfg.get("capture_width") and device_cfg.get("capture_height"):
        requested_size = (int(device_cfg["capture_width"]), int(device_cfg["capture_height"]))
    cap = open_highest_camera(index, requested_size)

    raw_dir = session_dir / "raw_sbs"
    accepted_dir = session_dir / "accepted"
    raw_dir.mkdir(parents=True, exist_ok=True)
    accepted_dir.mkdir(parents=True, exist_ok=True)
    pattern = (int(board_cfg["columns"]), int(board_cfg["rows"]))
    target = min(int(capture_cfg["target_pairs"]), int(capture_cfg["maximum_pairs"]))
    stable_seconds = float(capture_cfg["stable_seconds"])
    preview_width = int(capture_cfg["preview_width_per_eye"])
    accepted: List[AcceptedPair] = []
    paused = False
    candidate_since: Optional[float] = None
    candidate_feature: Optional[PoseFeatures] = None

    try:
        while len(accepted) < target:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("相机读取失败")
            left, right = split_sbs(frame, swap_eyes)
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            left_corners = detect_chessboard(left_gray, pattern)
            right_corners = detect_chessboard(right_gray, pattern)
            history = [item.features for item in accepted]
            decision = evaluate_pair(
                left_gray, right_gray, left_corners, right_corners, history, quality_cfg
            )

            now = time.monotonic()
            stable_progress = 0.0
            if not paused and decision.accepted and decision.features is not None:
                if candidate_feature is None or np.linalg.norm(
                    decision.features.as_array() - candidate_feature.as_array()
                ) > 0.025:
                    candidate_feature = decision.features
                    candidate_since = now
                stable_progress = min(1.0, (now - float(candidate_since)) / stable_seconds)
                if stable_progress >= 1.0:
                    pair_index = len(accepted)
                    raw_path = raw_dir / f"{pair_index:04d}_sbs.png"
                    left_path = accepted_dir / f"{pair_index:04d}_left.png"
                    right_path = accepted_dir / f"{pair_index:04d}_right.png"
                    if not all((cv2.imwrite(str(raw_path), frame), cv2.imwrite(str(left_path), left), cv2.imwrite(str(right_path), right))):
                        raise RuntimeError("写入标定图像失败，请检查磁盘空间")
                    pair = AcceptedPair(
                        pair_index,
                        left_path,
                        right_path,
                        raw_path,
                        left_corners.copy(),
                        right_corners.copy(),
                        (left.shape[1], left.shape[0]),
                        decision.features,
                        decision.metrics,
                    )
                    accepted.append(pair)
                    metadata = {
                        "index": pair_index,
                        "square_size_m": square_size_m,
                        "left_corners": pair.left_corners.tolist(),
                        "right_corners": pair.right_corners.tolist(),
                        "features": pair.features.__dict__,
                        "metrics": pair.metrics,
                    }
                    (accepted_dir / f"{pair_index:04d}_metadata.json").write_text(
                        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
                    )
                    candidate_since = None
                    candidate_feature = None
            else:
                candidate_since = None
                candidate_feature = None

            display_left = left.copy()
            display_right = right.copy()
            if left_corners is not None:
                cv2.drawChessboardCorners(display_left, pattern, left_corners.reshape(-1, 1, 2), True)
            if right_corners is not None:
                cv2.drawChessboardCorners(display_right, pattern, right_corners.reshape(-1, 1, 2), True)
            view = _preview(display_left, display_right, preview_width)
            status = "已暂停" if paused else decision.reason
            color = (0, 255, 0) if decision.accepted else (0, 0, 255)
            cv2.putText(view, f"{status}  accepted={len(accepted)}/{target}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)
            cv2.putText(view, guidance_hint(history), (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 200, 0), 2)
            cv2.putText(view, f"stable {stable_progress:.0%} | P pause | U undo | Q save+quit", (12, view.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imshow("SBS chessboard calibration", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                paused = not paused
            if key == ord("u") and accepted:
                _delete_pair(accepted.pop())
                candidate_since = None
                candidate_feature = None
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return accepted
