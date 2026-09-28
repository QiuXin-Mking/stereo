from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time
from typing import Callable, Mapping, Optional

import cv2
import numpy as np

from .imu_decoder import DecodedImuFrame, DeviceClock, decode_vertical_band


@dataclass(frozen=True)
class CaptureSummary:
    duration_seconds: float
    total_frames: int
    decoded_frames: int
    left_frames: int
    right_frames: int
    imu_samples: int
    decode_ratio: float


class KalibrRecorder:
    def __init__(
        self,
        session_dir: Path,
        decoder: Callable[..., DecodedImuFrame] = decode_vertical_band,
    ) -> None:
        self.session_dir = Path(session_dir)
        self.dataset_dir = self.session_dir / "kalibr"
        self._decoder = decoder
        self._clock = DeviceClock()
        self._state = "ready"
        self._codec = "MJPG"
        self._container = "avi"
        self._grayscale = True
        self._fps = 30.0
        self._jpeg_quality = 95
        self._display_interval_s = 0.5
        self._display_samples = 10
        self._started_monotonic: Optional[float] = None
        self._stopped_monotonic: Optional[float] = None
        self._cam0_writer = None
        self._cam1_writer = None
        self._video_width: Optional[int] = None
        self._video_height: Optional[int] = None
        self._total_frames = 0
        self._decoded_frames = 0
        self._decode_failures = 0
        self._left_frames = 0
        self._right_frames = 0
        self._imu_samples = 0
        self._magnetic_samples = 0
        self._duplicate_imu_timestamps = 0
        self._last_imu_timestamp_ns: Optional[int] = None
        self._imu_records: list[dict] = []
        self._display_imu: list[dict] = []
        self._last_display_monotonic: Optional[float] = None
        self._error: Optional[str] = None
        self._startup_info: dict[str, object] = {}

    def configure(self, config: Mapping[str, object]) -> None:
        video = dict(config.get("video") or {})
        display = dict(config.get("display") or {})
        self._codec = str(video.get("codec", self._codec))
        self._container = str(video.get("container", self._container)).lstrip(".")
        self._grayscale = bool(video.get("grayscale", self._grayscale))
        self._fps = float(video.get("fps", self._fps))
        self._jpeg_quality = int(video.get("jpeg_quality", self._jpeg_quality))
        self._display_interval_s = float(
            display.get("interval_s", self._display_interval_s)
        )
        self._display_samples = int(display.get("samples", self._display_samples))
        if self._fps <= 0:
            raise ValueError("video.fps 必须大于 0")
        if self._display_samples < 1:
            raise ValueError("display.samples 必须大于 0")

    def set_startup_info(self, info: dict[str, object]) -> None:
        self._startup_info = dict(info)

    def start(self, started_monotonic: Optional[float] = None) -> None:
        if self._state != "ready":
            raise RuntimeError("Kalibr 录制已经开始")
        self.dataset_dir.mkdir(parents=True, exist_ok=False)
        self._started_monotonic = (
            time.monotonic() if started_monotonic is None else float(started_monotonic)
        )
        self._state = "recording"
        self._write_manifest(complete=False, integrity_passed=False)

    def ingest(
        self,
        raw_frame: np.ndarray,
        left: np.ndarray,
        right: np.ndarray,
        frame_idx: int,
    ) -> None:
        if self._state != "recording":
            return
        self._total_frames += 1
        left_frame = self._gray(left) if self._grayscale else left
        right_frame = self._gray(right) if self._grayscale else right
        self._ensure_writers(left_frame)
        self._cam0_writer.write(left_frame)
        self._cam1_writer.write(right_frame)
        self._left_frames += 1
        self._right_frames += 1
        try:
            decoded = self._decoder(raw_frame, int(frame_idx), self._clock)
        except (RuntimeError, ValueError):
            self._decode_failures += 1
            return
        self._decoded_frames += 1
        self._magnetic_samples += len(decoded.magnetic_samples)
        for sample in decoded.samples:
            if (
                self._last_imu_timestamp_ns is not None
                and sample.timestamp_ns <= self._last_imu_timestamp_ns
            ):
                self._duplicate_imu_timestamps += 1
                continue
            self._imu_records.append(self._sample_record(sample))
            self._last_imu_timestamp_ns = sample.timestamp_ns
            self._imu_samples += 1
        self._update_display()

    def stop(self, stopped_monotonic: Optional[float] = None) -> CaptureSummary:
        if self._state != "recording":
            raise RuntimeError("当前没有正在进行的 Kalibr 录制")
        self._stopped_monotonic = (
            time.monotonic() if stopped_monotonic is None else float(stopped_monotonic)
        )
        self._state = "recorded"
        self._close_writers()
        self._write_imu_json()
        summary = self._summary()
        self._write_metadata(complete=True, error=None, summary=summary)
        integrity = self._finalize_integrity(summary)
        self._write_manifest(complete=True, integrity_passed=bool(integrity["passed"]))
        return summary

    def abort(
        self, reason: str, stopped_monotonic: Optional[float] = None
    ) -> None:
        if self._state not in {"recording", "ready"}:
            return
        self._error = str(reason)
        self._stopped_monotonic = (
            time.monotonic() if stopped_monotonic is None else float(stopped_monotonic)
        )
        if self._state == "ready":
            self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self._state = "error"
        self._close_writers()
        self._write_imu_json()
        self._write_metadata(complete=False, error=self._error, summary=self._summary())
        self._write_manifest(complete=False, integrity_passed=False)

    def snapshot(self) -> dict[str, object]:
        now = self._stopped_monotonic
        if now is None and self._started_monotonic is not None:
            now = time.monotonic()
        duration = 0.0
        if self._started_monotonic is not None and now is not None:
            duration = max(0.0, now - self._started_monotonic)
        return {
            "state": self._state,
            "duration_seconds": duration,
            "total_frames": self._total_frames,
            "decoded_frames": self._decoded_frames,
            "decode_ratio": (
                self._decoded_frames / self._total_frames if self._total_frames else 0.0
            ),
            "left_frames": self._left_frames,
            "right_frames": self._right_frames,
            "imu_samples": self._imu_samples,
            "display_imu": list(self._display_imu),
            "error": self._error,
            "dataset_dir": str(self.dataset_dir),
            "manifest": str(self.dataset_dir / "dataset_manifest.json"),
        }

    def _ensure_writers(self, frame: np.ndarray) -> None:
        if self._cam0_writer is not None:
            return
        height, width = frame.shape[:2]
        self._video_width = int(width)
        self._video_height = int(height)
        fourcc = cv2.VideoWriter_fourcc(*self._codec)
        cam0_path = self.dataset_dir / f"cam0.{self._container}"
        cam1_path = self.dataset_dir / f"cam1.{self._container}"
        self._cam0_writer = cv2.VideoWriter(
            str(cam0_path), fourcc, self._fps, (width, height),
            isColor=not self._grayscale,
        )
        self._cam1_writer = cv2.VideoWriter(
            str(cam1_path), fourcc, self._fps, (width, height),
            isColor=not self._grayscale,
        )
        if not self._cam0_writer.isOpened() or not self._cam1_writer.isOpened():
            raise RuntimeError("无法打开 Kalibr 视频写入器（codec/container 可能不受 OpenCV 支持）")
        if self._codec == "MJPG":
            self._cam0_writer.set(cv2.VIDEOWRITER_PROP_QUALITY, self._jpeg_quality)
            self._cam1_writer.set(cv2.VIDEOWRITER_PROP_QUALITY, self._jpeg_quality)

    def _update_display(self) -> None:
        now = time.monotonic()
        if (
            self._last_display_monotonic is not None
            and (now - self._last_display_monotonic) < self._display_interval_s
        ):
            return
        self._last_display_monotonic = now
        self._display_imu = list(self._imu_records[-self._display_samples:])

    @staticmethod
    def _sample_record(sample) -> dict:
        return {
            "timestamp_ns": sample.timestamp_ns,
            "frame_idx": sample.frame_idx,
            "t_us": sample.raw_t_us,
            "gyro_rps": list(sample.gyro_rps),
            "accel_mps2": list(sample.accel_mps2),
        }

    @staticmethod
    def _gray(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return image
        if image.ndim == 3 and image.shape[2] == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        raise ValueError("Kalibr 图像必须为灰度或 BGR")

    def _summary(self) -> CaptureSummary:
        duration = 0.0
        if self._started_monotonic is not None and self._stopped_monotonic is not None:
            duration = max(0.0, self._stopped_monotonic - self._started_monotonic)
        return CaptureSummary(
            duration_seconds=duration,
            total_frames=self._total_frames,
            decoded_frames=self._decoded_frames,
            left_frames=self._left_frames,
            right_frames=self._right_frames,
            imu_samples=self._imu_samples,
            decode_ratio=(
                self._decoded_frames / self._total_frames if self._total_frames else 0.0
            ),
        )

    def _close_writers(self) -> None:
        for writer in (self._cam0_writer, self._cam1_writer):
            if writer is not None:
                writer.release()
        self._cam0_writer = None
        self._cam1_writer = None

    def _write_imu_json(self) -> None:
        payload = {
            "schema_version": 1,
            "session_id": self.session_dir.name,
            "image_width": self._video_width,
            "image_height": self._video_height,
            "fps": self._fps,
            "codec": self._codec,
            "samples": self._imu_records,
        }
        self._atomic_json(self.dataset_dir / "imu.json", payload)

    def _write_metadata(
        self, complete: bool, error: Optional[str], summary: CaptureSummary
    ) -> None:
        capture = {**asdict(summary), "complete": bool(complete), "error": error}
        stats = {
            "total_frames": self._total_frames,
            "decoded_frames": self._decoded_frames,
            "decode_failures": self._decode_failures,
            "decode_ratio": summary.decode_ratio,
            "imu_samples": self._imu_samples,
            "magnetic_samples": self._magnetic_samples,
            "duplicate_imu_timestamps": self._duplicate_imu_timestamps,
        }
        self._atomic_json(self.dataset_dir / "capture.json", capture)
        self._atomic_json(self.dataset_dir / "decoder_stats.json", stats)

    def _write_manifest(self, *, complete: bool, integrity_passed: bool) -> None:
        payload = {
            "schema_version": 1,
            "session_id": self.session_dir.name,
            "dataset_dir": str(self.dataset_dir.resolve()),
            "started_monotonic": self._started_monotonic,
            "stopped_monotonic": self._stopped_monotonic,
            "state": self._state,
            "complete": bool(complete),
            "integrity_passed": bool(integrity_passed),
            "required_artifacts": [
                "imu.json", "capture.json", "decoder_stats.json",
                f"cam0.{self._container}", f"cam1.{self._container}", "integrity.json",
            ],
            "startup": dict(self._startup_info),
        }
        self._atomic_json(self.dataset_dir / "dataset_manifest.json", payload)

    def _finalize_integrity(self, summary: CaptureSummary) -> dict[str, object]:
        reasons: list[str] = []
        required_files = ("imu.json", "capture.json", "decoder_stats.json")
        missing = [
            name for name in required_files
            if not (self.dataset_dir / name).is_file()
        ]
        if missing:
            reasons.append("缺少必需产物：" + ", ".join(missing))
        for name in (f"cam0.{self._container}", f"cam1.{self._container}"):
            path = self.dataset_dir / name
            if not path.is_file() or path.stat().st_size == 0:
                reasons.append(f"缺少或空视频文件：{name}")
        sample_count = 0
        try:
            with (self.dataset_dir / "imu.json").open(encoding="utf-8") as handle:
                imu = json.load(handle)
            sample_count = len(imu.get("samples", []))
        except (OSError, json.JSONDecodeError):
            reasons.append("imu.json 无法解析")
        if sample_count != self._imu_samples:
            reasons.append(
                f"imu.json 样本数 {sample_count} 与记录 {self._imu_samples} 不一致"
            )
        if summary.left_frames != summary.right_frames:
            reasons.append("停止时左右帧计数不一致")
        report = {
            "passed": not reasons,
            "reasons": reasons,
            "dataset_dir": str(self.dataset_dir.resolve()),
            "required_files": {
                name: (self.dataset_dir / name).is_file() for name in required_files
            },
            "total_frames": self._total_frames,
            "decoded_frames": self._decoded_frames,
            "left_frames": summary.left_frames,
            "right_frames": summary.right_frames,
            "imu_samples": summary.imu_samples,
        }
        self._atomic_json(self.dataset_dir / "integrity.json", report)
        return report

    @staticmethod
    def _atomic_json(path: Path, payload: dict[str, object]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(path)
