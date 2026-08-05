from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import Dict, List, Mapping, Optional

import cv2
import numpy as np

from .camera_backend import CameraMode, open_linux_camera
from .detector import detect_chessboard
from .exporter import export_result
from .models import AcceptedPair, PoseFeatures
from .quality import evaluate_pair
from .sbs import split_sbs
from .solver import solve_stereo


RK3588_MODE = CameraMode(3840, 1080, 30.0, "MJPG")


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
    ) -> None:
        self.config = config
        self.session_dir = Path(session_dir)
        self.square_size_m = float(square_size_m)
        self.device = device
        self._camera = camera
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._solve_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._accepted: List[AcceptedPair] = []
        self._manual_requests = 0
        self._preview = self._placeholder_preview()
        self._status: Dict[str, object] = {
            "state": "starting",
            "device": device,
            "mode": "MJPG 3840x1080@30",
            "per_eye": "1920x1080",
            "accepted_pairs": 0,
            "manual_pairs": 0,
            "target_pairs": int(config["capture"]["target_pairs"]),
            "reason": "等待启动",
            "stable_progress": 0.0,
            "guidance": manual_guidance(0, int(config["capture"]["target_pairs"])),
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
            self._camera = open_linux_camera(self.device, RK3588_MODE)
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
        if name not in {"pause", "resume", "undo", "solve", "stop", "manual_capture"}:
            return {"ok": False, "error": "不支持的操作"}
        with self._lock:
            state = str(self._status["state"])
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
                if len(self._accepted) < minimum:
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
        pattern = (int(self.config["board"]["columns"]), int(self.config["board"]["rows"]))
        stable_seconds = float(self.config["capture"]["stable_seconds"])
        target = int(self.config["capture"]["target_pairs"])
        try:
            while not self._stop_event.is_set():
                state = str(self.status_snapshot()["state"])
                if state == "paused":
                    time.sleep(0.05)
                    continue
                if self._solve_event.is_set() or len(self._accepted) >= target:
                    self._solve()
                    return
                ok, frame = self._camera.read()
                if not ok or frame is None:
                    raise RuntimeError("V4L2 连续采帧失败")
                if frame.shape[1] != RK3588_MODE.width or frame.shape[0] != RK3588_MODE.height:
                    raise RuntimeError(
                        f"采集帧尺寸变化：期望 3840x1080，实际 {frame.shape[1]}x{frame.shape[0]}"
                    )
                left, right = split_sbs(frame, bool(self.config["device"].get("swap_eyes", False)))
                with self._lock:
                    manual_requested = self._manual_requests > 0
                    if manual_requested:
                        self._manual_requests -= 1
                if manual_requested:
                    self._save_manual_snapshot(frame, left, right)
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
                if decision.accepted and decision.features is not None:
                    if candidate_feature is None or np.linalg.norm(
                        decision.features.as_array() - candidate_feature.as_array()
                    ) > 0.025:
                        candidate_feature = decision.features
                        candidate_since = now
                    stable_progress = min(1.0, (now - float(candidate_since)) / stable_seconds)
                    if stable_progress >= 1.0:
                        self._save_pair(frame, left, right, left_corners, right_corners, decision)
                        candidate_since = None
                        candidate_feature = None
                        stable_progress = 0.0
                else:
                    candidate_since = None
                    candidate_feature = None

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

    def _save_pair(self, frame, left, right, left_corners, right_corners, decision) -> None:
        raw_dir = self.session_dir / "raw_sbs"
        accepted_dir = self.session_dir / "accepted"
        raw_dir.mkdir(parents=True, exist_ok=True)
        accepted_dir.mkdir(parents=True, exist_ok=True)
        index = len(self._accepted)
        raw_path = raw_dir / f"{index:04d}_sbs.jpg"
        left_path = accepted_dir / f"{index:04d}_left.png"
        right_path = accepted_dir / f"{index:04d}_right.png"
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
        (accepted_dir / f"{index:04d}_metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        with self._lock:
            self._accepted.append(pair)

    def _save_manual_snapshot(self, frame, left, right) -> None:
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
        with self._lock:
            completed = index + 1
            target = int(self._status["target_pairs"])
            self._status.update(
                manual_pairs=completed,
                guidance=manual_guidance(completed, target),
                reason="本轮采集完成" if completed >= target else f"已保存第 {completed} 组",
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

    def _solve(self) -> None:
        with self._lock:
            self._status.update(state="solving", reason="正在计算内外参", stable_progress=0.0)
            pairs = list(self._accepted)
        pattern = (int(self.config["board"]["columns"]), int(self.config["board"]["rows"]))
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
