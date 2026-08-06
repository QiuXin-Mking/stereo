from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import Dict, List, Mapping, Optional

import cv2
import numpy as np

from .camera_backend import (
    CameraMode,
    linux_camera_name,
    open_first_supported_linux_camera,
)
from .camera_profile import CameraProfile, detect_camera_profile, split_profile_frame
from .detector import detect_chessboard, detect_chessboard_with_retry
from .exporter import export_result
from .models import AcceptedPair, PoseFeatures
from .pose_slots import POSE_SLOT_QUOTAS, classify_pose_slot
from .quality import evaluate_pair
from .solver import solve_stereo


SUPPORTED_RK3588_MODES = (
    CameraMode(4000, 1200, 30.0, "MJPG"),
    CameraMode(3840, 1080, 30.0, "MJPG"),
)


def manual_guidance(completed: int, target: int = 32) -> str:
    if completed >= target:
        return f"本轮 {target} 组采集完成"
    poses = (
        "棋盘放在中央并正对相机",
        "棋盘放在中央并正对相机",
        "将棋盘移到左侧",
        "将棋盘移到左侧",
        "将棋盘移到右侧",
        "将棋盘移到右侧",
        "将棋盘移到上方",
        "将棋盘移到上方",
        "将棋盘移到下方",
        "将棋盘移到下方",
        "将棋盘移到左上角",
        "将棋盘移到左上角",
        "将棋盘移到右上角",
        "将棋盘移到右上角",
        "将棋盘移到左下角",
        "将棋盘移到左下角",
        "将棋盘移到右下角",
        "将棋盘移到右下角",
        "棋盘向左偏航",
        "棋盘向左偏航",
        "棋盘向右偏航",
        "棋盘向右偏航",
        "棋盘向上俯仰",
        "棋盘向上俯仰",
        "棋盘向下俯仰",
        "棋盘向下俯仰",
        "棋盘顺时针旋转",
        "棋盘顺时针旋转",
        "棋盘逆时针旋转",
        "棋盘逆时针旋转",
        "将棋盘靠近相机",
        "将棋盘远离相机",
    )
    index = min(max(int(completed), 0), len(poses) - 1)
    return f"第 {index + 1}/{target} 组：{poses[index]}"


class HeadlessCalibrationEngine:
    """Own the V4L2 camera, guided capture state, solving, and preview bytes."""

    def __init__(
        self,
        config: Mapping[str, object],
        session_dir: Path,
        square_size_m: float,
        device: str,
        camera=None,
        device_name: Optional[str] = None,
    ) -> None:
        self.config = config
        self.session_dir = Path(session_dir)
        self.square_size_m = float(square_size_m)
        self.device = device
        self._camera = camera
        self._device_name = device_name or str(config["device"].get("name", device))
        self._selected_mode: Optional[CameraMode] = None
        self._profile: Optional[CameraProfile] = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._solve_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._accepted: List[AcceptedPair] = []
        self._manual_requests = 0
        self._manual_burst = []
        self._pose_slots = {name: 0 for name in POSE_SLOT_QUOTAS}
        self._preview = self._placeholder_preview()
        existing_manual = len(self._manual_pair_paths())
        self._status: Dict[str, object] = {
            "state": "starting",
            "device": device,
            "camera_label": "detecting",
            "mode": "探测中",
            "per_eye": "探测中",
            "code_band": "探测中",
            "accepted_pairs": 0,
            "manual_pairs": existing_manual,
            "saved_pairs": existing_manual,
            "detected_valid_pairs": 0,
            "auto_capture_enabled": True,
            "pose_slots": dict(self._pose_slots),
            "target_pairs": int(config["capture"]["target_pairs"]),
            "reason": "等待启动",
            "stable_progress": 0.0,
            "guidance": manual_guidance(
                existing_manual, int(config["capture"]["target_pairs"])
            ),
            "mono_rms_left": None,
            "mono_rms_right": None,
            "epipolar_p95": None,
            "result_dir": None,
            "error": None,
        }

    @staticmethod
    def _placeholder_preview() -> bytes:
        image = np.full((360, 1280, 3), 30, dtype=np.uint8)
        cv2.putText(
            image,
            "Waiting for RK3588 camera...",
            (330, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (230, 230, 230),
            2,
        )
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
        return encoded.tobytes() if ok else b""

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("采集引擎已经启动")
        self.session_dir.mkdir(parents=True, exist_ok=True)
        if self._camera is None:
            self._device_name = linux_camera_name(self.device)
            self._camera, self._selected_mode = open_first_supported_linux_camera(
                self.device, SUPPORTED_RK3588_MODES
            )
        with self._lock:
            self._status.update(state="capturing", reason="等待棋盘")
        self._thread = threading.Thread(target=self._run, name="calibration-capture", daemon=True)
        self._thread.start()

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def status_snapshot(self) -> Dict[str, object]:
        with self._lock:
            return dict(self._status)

    def latest_preview(self) -> bytes:
        with self._lock:
            return bytes(self._preview)

    def action(self, name: str) -> Dict[str, object]:
        if name not in {
            "pause", "resume", "undo", "solve", "stop", "manual_capture", "auto_on", "auto_off"
        }:
            return {"ok": False, "error": "不支持的操作"}
        with self._lock:
            state = str(self._status["state"])
            if name in {"auto_on", "auto_off"}:
                enabled = name == "auto_on"
                self._status.update(
                    auto_capture_enabled=enabled,
                    reason="自动采集已开启" if enabled else "自动采集已关闭",
                )
                return {"ok": True}
            if name == "manual_capture":
                if state != "capturing":
                    return {"ok": False, "error": "当前状态不能手动拍摄"}
                target = int(self._status["target_pairs"])
                if int(self._status["manual_pairs"]) + self._manual_requests >= target:
                    return {"ok": False, "error": f"本轮 {target} 组已采集完成"}
                self._manual_requests += 1
                return {"ok": True}
            if name == "pause":
                if state in {"solving", "pass", "retake", "error", "stopped"}:
                    return {"ok": False, "error": "当前状态不能暂停"}
                self._status.update(state="paused", reason="已暂停")
                return {"ok": True}
            if name == "resume":
                if state not in {"paused", "starting"}:
                    return {"ok": False, "error": "当前状态不能继续"}
                self._status.update(state="capturing", reason="等待棋盘")
                return {"ok": True}
            if name == "undo":
                if state == "solving":
                    return {"ok": False, "error": "求解中不能撤销"}
                if not self._accepted:
                    return {"ok": False, "error": "没有可撤销的图像对"}
                pair = self._accepted.pop()
                self._remove_pair_files(pair)
                self._status.update(accepted_pairs=len(self._accepted), reason="已撤销上一对")
                return {"ok": True}
            if name == "solve":
                minimum = int(self.config["capture"]["minimum_pairs"])
                saved_manual = len(self._manual_pair_paths())
                if max(len(self._accepted), saved_manual) < minimum:
                    return {"ok": False, "error": f"至少需要 {minimum} 对图像"}
                if state == "solving":
                    return {"ok": False, "error": "已经在求解"}
                self._solve_event.set()
                return {"ok": True}
            self._stop_event.set()
            self._status.update(state="stopped", reason="服务已停止")
            return {"ok": True}

    def _run(self) -> None:
        candidate_since: Optional[float] = None
        candidate_feature: Optional[PoseFeatures] = None
        candidate_slot: Optional[str] = None
        pattern = (int(self.config["board"]["columns"]), int(self.config["board"]["rows"]))
        stable_seconds = float(self.config["capture"]["stable_seconds"])
        target = int(self.config["capture"]["target_pairs"])
        try:
            while not self._stop_event.is_set():
                state = str(self.status_snapshot()["state"])
                if state == "paused":
                    time.sleep(0.05)
                    continue
                if self._solve_event.is_set():
                    self._solve()
                    return
                ok, frame = self._camera.read()
                if not ok or frame is None:
                    raise RuntimeError("V4L2 连续采帧失败")
                if self._selected_mode is None:
                    self._selected_mode = CameraMode(
                        frame.shape[1], frame.shape[0], 30.0, "MJPG"
                    )
                if self._profile is None:
                    self._profile = detect_camera_profile(
                        self._device_name, frame, self._selected_mode
                    )
                    with self._lock:
                        self._status.update(
                            camera_label=self._profile.label,
                            mode=self._profile.mode.describe(),
                            per_eye=(
                                f"{self._profile.per_eye_size[0]}x"
                                f"{self._profile.per_eye_size[1]}"
                            ),
                            code_band=self._profile.code_band_status,
                        )
                if (
                    frame.shape[1] != self._profile.mode.width
                    or frame.shape[0] != self._profile.mode.height
                ):
                    raise RuntimeError(
                        f"采集帧尺寸变化：期望 {self._profile.mode.width}x"
                        f"{self._profile.mode.height}，实际 {frame.shape[1]}x{frame.shape[0]}"
                    )
                left, right = split_profile_frame(
                    frame,
                    self._profile,
                    bool(self.config["device"].get("swap_eyes", False)),
                )
                with self._lock:
                    manual_requested = self._manual_requests > 0
                if manual_requested:
                    self._manual_burst.append((frame.copy(), left.copy(), right.copy()))
                    if len(self._manual_burst) >= 5:
                        burst = self._manual_burst[:5]
                        self._manual_burst.clear()
                        self._save_manual_burst(burst, pattern)
                        with self._lock:
                            self._manual_requests -= 1
                left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
                left_corners = detect_chessboard(left_gray, pattern)
                right_corners = detect_chessboard(right_gray, pattern)
                history = [pair.features for pair in self._accepted]
                decision = evaluate_pair(
                    left_gray,
                    right_gray,
                    left_corners,
                    right_corners,
                    history,
                    self.config["quality"],
                )
                now = time.monotonic()
                stable_progress = 0.0
                auto_enabled = bool(self.status_snapshot()["auto_capture_enabled"])
                slot = None
                if decision.accepted and left_corners is not None:
                    slot = classify_pose_slot(
                        left_corners,
                        pattern,
                        (left.shape[1], left.shape[0]),
                        self._pose_slots,
                    )
                if auto_enabled and slot is not None and decision.features is not None:
                    if candidate_slot != slot or candidate_feature is None or np.linalg.norm(
                        decision.features.as_array() - candidate_feature.as_array()
                    ) > 0.025:
                        candidate_feature = decision.features
                        candidate_since = now
                        candidate_slot = slot
                    stable_progress = min(1.0, (now - float(candidate_since)) / stable_seconds)
                    if stable_progress >= 1.0:
                        self._save_pair(
                            frame, left, right, left_corners, right_corners, decision, slot
                        )
                        candidate_since = None
                        candidate_feature = None
                        candidate_slot = None
                        stable_progress = 0.0
                else:
                    candidate_since = None
                    candidate_feature = None
                    candidate_slot = None

                self._update_preview(left, right, left_corners, right_corners, pattern)
                with self._lock:
                    manual_pairs = int(self._status["manual_pairs"])
                    self._status.update(
                        state="capturing",
                        accepted_pairs=len(self._accepted),
                        reason="本轮采集完成" if manual_pairs >= target else decision.reason,
                        stable_progress=stable_progress,
                        guidance=manual_guidance(manual_pairs, target),
                    )
        except Exception as error:
            with self._lock:
                self._status.update(state="error", reason="采集失败", error=str(error))
        finally:
            if self._camera is not None:
                self._camera.release()

    def _save_pair(
        self, frame, left, right, left_corners, right_corners, decision, slot: str
    ) -> None:
        manual_dir = self.session_dir / "manual"
        manual_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            index = int(self._status["manual_pairs"])
        raw_path = manual_dir / f"{index:04d}_sbs.jpg"
        left_path = manual_dir / f"{index:04d}_left.png"
        right_path = manual_dir / f"{index:04d}_right.png"
        if not all(
            (cv2.imwrite(str(raw_path), frame), cv2.imwrite(str(left_path), left), cv2.imwrite(str(right_path), right))
        ):
            raise RuntimeError("写入标定图像失败")
        pair = AcceptedPair(
            index=index,
            left_path=left_path,
            right_path=right_path,
            raw_path=raw_path,
            left_corners=left_corners.copy(),
            right_corners=right_corners.copy(),
            image_size=(left.shape[1], left.shape[0]),
            features=decision.features,
            metrics=decision.metrics,
        )
        metadata = {
            "index": index,
            "square_size_m": self.square_size_m,
            "left_corners": pair.left_corners.tolist(),
            "right_corners": pair.right_corners.tolist(),
            "features": pair.features.__dict__,
            "metrics": pair.metrics,
        }
        metadata.update({"source": "auto", "pose_slot": slot, "corners_detected": True})
        (manual_dir / f"{index:04d}_metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        with self._lock:
            self._accepted.append(pair)
            self._pose_slots[slot] += 1
            completed = index + 1
            self._status.update(
                accepted_pairs=len(self._accepted),
                manual_pairs=completed,
                saved_pairs=completed,
                detected_valid_pairs=int(self._status["detected_valid_pairs"]) + 1,
                pose_slots=dict(self._pose_slots),
                reason=f"自动保存 {slot}（第 {completed} 组）",
            )

    @staticmethod
    def _sharpness_score(left, right) -> float:
        scores = []
        for image in (left, right):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            scores.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        return min(scores)

    def _save_manual_burst(self, burst, pattern) -> None:
        scored = [
            (self._sharpness_score(left, right), frame, left, right)
            for frame, left, right in burst
        ]
        sharpness, frame, left, right = max(scored, key=lambda item: item[0])
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        corners_detected = (
            detect_chessboard(left_gray, pattern) is not None
            and detect_chessboard(right_gray, pattern) is not None
        )
        self._save_manual_snapshot(frame, left, right, sharpness, corners_detected)

    def _save_manual_snapshot(
        self, frame, left, right, sharpness: float = 0.0, corners_detected: bool = False
    ) -> None:
        manual_dir = self.session_dir / "manual"
        manual_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            index = int(self._status["manual_pairs"])
        paths = (
            manual_dir / f"{index:04d}_sbs.jpg",
            manual_dir / f"{index:04d}_left.png",
            manual_dir / f"{index:04d}_right.png",
        )
        if not all(
            (
                cv2.imwrite(str(paths[0]), frame),
                cv2.imwrite(str(paths[1]), left),
                cv2.imwrite(str(paths[2]), right),
            )
        ):
            raise RuntimeError("写入手动采集图像失败")
        metadata = {
            "index": index,
            "source": "manual",
            "captured_at_unix": time.time(),
            "sharpness": float(sharpness),
            "corners_detected": bool(corners_detected),
        }
        (manual_dir / f"{index:04d}_metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        with self._lock:
            completed = index + 1
            target = int(self._status["target_pairs"])
            valid = int(self._status["detected_valid_pairs"]) + int(corners_detected)
            self._status.update(
                manual_pairs=completed,
                saved_pairs=completed,
                detected_valid_pairs=valid,
                guidance=manual_guidance(completed, target),
                reason=(
                    "本轮采集完成"
                    if completed >= target
                    else f"已强制保存第 {completed} 组"
                ),
            )

    def _update_preview(self, left, right, left_corners, right_corners, pattern) -> None:
        left_view = left.copy()
        right_view = right.copy()
        if left_corners is not None:
            cv2.drawChessboardCorners(left_view, pattern, left_corners.reshape(-1, 1, 2), True)
        if right_corners is not None:
            cv2.drawChessboardCorners(right_view, pattern, right_corners.reshape(-1, 1, 2), True)
        width = int(self.config["capture"]["preview_width_per_eye"])
        height = int(left.shape[0] * width / left.shape[1])
        preview = np.hstack(
            [
                cv2.resize(left_view, (width, height), interpolation=cv2.INTER_AREA),
                cv2.resize(right_view, (width, height), interpolation=cv2.INTER_AREA),
            ]
        )
        ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            with self._lock:
                self._preview = encoded.tobytes()

    def _manual_pair_paths(self):
        manual_dir = self.session_dir / "manual"
        pairs = []
        for left_path in sorted(manual_dir.glob("*_left.png")):
            right_path = manual_dir / left_path.name.replace("_left.png", "_right.png")
            if right_path.exists():
                pairs.append((left_path, right_path))
        return pairs

    def _load_manual_pairs(self, pattern):
        pairs: List[AcceptedPair] = []
        rejected: List[int] = []
        manual_dir = self.session_dir / "manual"
        for left_path, right_path in self._manual_pair_paths():
            index = int(left_path.name.split("_", 1)[0])
            left = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
            right = cv2.imread(str(right_path), cv2.IMREAD_COLOR)
            if left is None or right is None or left.shape != right.shape:
                rejected.append(index)
                continue
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            left_corners, left_method = detect_chessboard_with_retry(left_gray, pattern)
            right_corners, right_method = detect_chessboard_with_retry(right_gray, pattern)
            if left_corners is None or right_corners is None:
                rejected.append(index)
                continue
            decision = evaluate_pair(
                left_gray,
                right_gray,
                left_corners,
                right_corners,
                [],
                self.config["quality"],
            )
            if not decision.accepted or decision.features is None:
                rejected.append(index)
                continue
            raw_path = manual_dir / f"{index:04d}_sbs.jpg"
            pairs.append(
                AcceptedPair(
                    index=index,
                    left_path=left_path,
                    right_path=right_path,
                    raw_path=raw_path if raw_path.exists() else None,
                    left_corners=left_corners.copy(),
                    right_corners=right_corners.copy(),
                    image_size=(left.shape[1], left.shape[0]),
                    features=decision.features,
                    metrics={
                        **decision.metrics,
                        "left_detection_enhanced": float(left_method == "clahe"),
                        "right_detection_enhanced": float(right_method == "clahe"),
                    },
                )
            )
        return pairs, rejected

    def _solve(self) -> None:
        with self._lock:
            self._status.update(state="solving", reason="正在检测手动样本角点", stable_progress=0.0)
        pattern = (int(self.config["board"]["columns"]), int(self.config["board"]["rows"]))
        pairs, rejected = self._load_manual_pairs(pattern)
        with self._lock:
            self._status.update(detected_valid_pairs=len(pairs), accepted_pairs=len(pairs))
        minimum = int(self.config["capture"]["minimum_pairs"])
        if len(pairs) < minimum:
            missing_slots = [
                name for name, quota in POSE_SLOT_QUOTAS.items()
                if self._pose_slots.get(name, 0) < quota
            ]
            with self._lock:
                self._status.update(
                    state="retake",
                    reason="有效样本不足，请补拍",
                    accepted_pairs=len(pairs),
                    error=(
                        f"已检测 {len(self._manual_pair_paths())} 对，"
                        f"有效 {len(pairs)} 对，至少需要 {minimum} 对；"
                        f"不合格编号：{rejected}；"
                        f"建议补拍：{missing_slots[:4]}"
                    ),
                )
            return
        with self._lock:
            self._status.update(accepted_pairs=len(pairs), reason="正在计算内外参")
        result = solve_stereo(pairs, pattern, self.square_size_m, self.config["validation"])
        output_name = "final" if result.passed else "diagnostic"
        output_dir = self.session_dir / "results" / output_name
        export_result(result, output_dir, pairs[0], self.square_size_m)
        with self._lock:
            self._status.update(
                state="pass" if result.passed else "retake",
                reason="标定通过" if result.passed else "需要补拍",
                mono_rms_left=result.mono_rms_left,
                mono_rms_right=result.mono_rms_right,
                epipolar_p95=result.epipolar_p95,
                result_dir=str(output_dir),
                error="；".join(result.failure_reasons) if result.failure_reasons else None,
            )

    @staticmethod
    def _remove_pair_files(pair: AcceptedPair) -> None:
        for path in (pair.raw_path, pair.left_path, pair.right_path):
            if path is not None and path.exists():
                path.unlink()
        metadata = pair.left_path.parent / f"{pair.index:04d}_metadata.json"
        if metadata.exists():
            metadata.unlink()
