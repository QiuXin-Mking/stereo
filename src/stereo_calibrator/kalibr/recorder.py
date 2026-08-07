from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time
from typing import Callable, Optional

import cv2
import numpy as np

from .imu_decoder import DecodedImuFrame, DeviceClock, decode_vertical_band


IMU_HEADER = (
    "timestamp_ns", "omega_x", "omega_y", "omega_z",
    "alpha_x", "alpha_y", "alpha_z", "frame_idx", "t_us",
)
FRAMES_HEADER = (
    "frame_idx", "image_timestamp_ns", "exp_start_ns", "exp_end_ns",
    "payload_bytes", "imu_samples", "cam0_path", "cam1_path",
)


@dataclass(frozen=True)
class CaptureSummary:
    duration_seconds: float
    total_frames: int
    decoded_frames: int
    left_images: int
    right_images: int
    imu_samples: int
    decode_ratio: float


class KalibrRecorder:
    def __init__(
        self,
        session_dir: Path,
        decoder: Callable[..., DecodedImuFrame] = decode_vertical_band,
        image_stride: int = 3,
    ) -> None:
        if int(image_stride) < 1:
            raise ValueError("image_stride 必须大于 0")
        self.session_dir = Path(session_dir)
        self.dataset_dir = self.session_dir / "kalibr"
        self.image_stride = int(image_stride)
        self._decoder = decoder
        self._clock = DeviceClock()
        self._state = "ready"
        self._started_monotonic: Optional[float] = None
        self._stopped_monotonic: Optional[float] = None
        self._imu_handle = None
        self._frames_handle = None
        self._imu_writer = None
        self._frames_writer = None
        self._total_frames = 0
        self._decoded_frames = 0
        self._decode_failures = 0
        self._left_images = 0
        self._right_images = 0
        self._imu_samples = 0
        self._magnetic_samples = 0
        self._duplicate_imu_timestamps = 0
        self._duplicate_image_timestamps = 0
        self._last_imu_timestamp_ns: Optional[int] = None
        self._last_image_timestamp_ns: Optional[int] = None
        self._error: Optional[str] = None

    def start(self, started_monotonic: Optional[float] = None) -> None:
        if self._state != "ready":
            raise RuntimeError("Kalibr 录制已经开始")
        self.dataset_dir.mkdir(parents=True, exist_ok=False)
        (self.dataset_dir / "cam0").mkdir()
        (self.dataset_dir / "cam1").mkdir()
        self._imu_handle = (self.dataset_dir / "imu0.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._frames_handle = (self.dataset_dir / "frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._imu_writer = csv.writer(self._imu_handle)
        self._frames_writer = csv.writer(self._frames_handle)
        self._imu_writer.writerow(IMU_HEADER)
        self._frames_writer.writerow(FRAMES_HEADER)
        self._started_monotonic = (
            time.monotonic() if started_monotonic is None else float(started_monotonic)
        )
        self._state = "recording"

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
            self._imu_writer.writerow(
                (
                    sample.timestamp_ns,
                    *sample.gyro_rps,
                    *sample.accel_mps2,
                    sample.frame_idx,
                    sample.raw_t_us,
                )
            )
            self._last_imu_timestamp_ns = sample.timestamp_ns
            self._imu_samples += 1

        cam0_path = ""
        cam1_path = ""
        if int(frame_idx) % self.image_stride == 0:
            if (
                self._last_image_timestamp_ns is not None
                and decoded.image_timestamp_ns <= self._last_image_timestamp_ns
            ):
                self._duplicate_image_timestamps += 1
            else:
                cam0_path, cam1_path = self._write_image_pair(
                    decoded.image_timestamp_ns, left, right
                )
                self._last_image_timestamp_ns = decoded.image_timestamp_ns
        self._frames_writer.writerow(
            (
                decoded.frame_idx,
                decoded.image_timestamp_ns,
                decoded.exp_start_ns,
                decoded.exp_end_ns,
                decoded.payload_bytes,
                len(decoded.samples),
                cam0_path,
                cam1_path,
            )
        )
        self._imu_handle.flush()
        self._frames_handle.flush()

    def stop(self, stopped_monotonic: Optional[float] = None) -> CaptureSummary:
        if self._state != "recording":
            raise RuntimeError("当前没有正在进行的 Kalibr 录制")
        self._stopped_monotonic = (
            time.monotonic() if stopped_monotonic is None else float(stopped_monotonic)
        )
        self._state = "recorded"
        self._close_files()
        summary = self._summary()
        self._write_metadata(complete=True, error=None, summary=summary)
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
        self._close_files()
        self._write_metadata(complete=False, error=self._error, summary=self._summary())

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
            "left_images": self._left_images,
            "right_images": self._right_images,
            "imu_samples": self._imu_samples,
            "error": self._error,
        }

    def _write_image_pair(
        self, timestamp_ns: int, left: np.ndarray, right: np.ndarray
    ) -> tuple[str, str]:
        left_gray = self._gray(left)
        right_gray = self._gray(right)
        left_name = f"{timestamp_ns}.png"
        right_name = f"{timestamp_ns}.png"
        left_final = self.dataset_dir / "cam0" / left_name
        right_final = self.dataset_dir / "cam1" / right_name
        left_temp = self.dataset_dir / "cam0" / f".{timestamp_ns}.tmp.png"
        right_temp = self.dataset_dir / "cam1" / f".{timestamp_ns}.tmp.png"
        try:
            if not cv2.imwrite(str(left_temp), left_gray):
                raise RuntimeError("写入 Kalibr 左图失败")
            if not cv2.imwrite(str(right_temp), right_gray):
                raise RuntimeError("写入 Kalibr 右图失败")
            left_temp.replace(left_final)
            right_temp.replace(right_final)
        except Exception:
            left_temp.unlink(missing_ok=True)
            right_temp.unlink(missing_ok=True)
            left_final.unlink(missing_ok=True)
            right_final.unlink(missing_ok=True)
            raise
        self._left_images += 1
        self._right_images += 1
        return f"cam0/{left_name}", f"cam1/{right_name}"

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
            left_images=self._left_images,
            right_images=self._right_images,
            imu_samples=self._imu_samples,
            decode_ratio=(
                self._decoded_frames / self._total_frames if self._total_frames else 0.0
            ),
        )

    def _close_files(self) -> None:
        for handle in (self._imu_handle, self._frames_handle):
            if handle is not None and not handle.closed:
                handle.flush()
                handle.close()

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
            "duplicate_image_timestamps": self._duplicate_image_timestamps,
        }
        self._atomic_json(self.dataset_dir / "capture.json", capture)
        self._atomic_json(self.dataset_dir / "decoder_stats.json", stats)

    @staticmethod
    def _atomic_json(path: Path, payload: dict[str, object]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(path)
