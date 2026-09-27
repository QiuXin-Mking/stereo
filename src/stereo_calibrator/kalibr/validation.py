from __future__ import annotations

import csv
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping

import cv2
import numpy as np

@dataclass(frozen=True)
class ValidationReport:
    passed: bool
    reasons: tuple[str, ...]
    metrics: dict[str, float | int | bool]


def _read_json(path: Path) -> dict[str, object]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError) as error:
        raise ValueError(f"Kalibr 数据文件无效：{path.name}") from error


def _read_imu(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    timestamps = []
    gyro = []
    accel = []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                timestamps.append(int(row["timestamp_ns"]))
                gyro.append(tuple(float(row[name]) for name in ("omega_x", "omega_y", "omega_z")))
                accel.append(tuple(float(row[name]) for name in ("alpha_x", "alpha_y", "alpha_z")))
    except (FileNotFoundError, KeyError, TypeError, ValueError) as error:
        raise ValueError("Kalibr IMU CSV 无效") from error
    return (
        np.asarray(timestamps, dtype=np.int64),
        np.asarray(gyro, dtype=np.float64),
        np.asarray(accel, dtype=np.float64),
    )


def _board_coverage(
    cam0: Path, cam1: Path, common_names: set[str]
) -> tuple[int, int]:
    detections = 0
    cells: set[tuple[int, int]] = set()
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    detector = cv2.aruco.ArucoDetector(dictionary)
    names = sorted(common_names)
    # Keep validation bounded on RK3588 while preserving temporal/spatial
    # coverage: a long recording commonly contains hundreds of saved images,
    # but the gate only needs 60 valid stereo observations.
    if len(names) > 90:
        indices = np.linspace(0, len(names) - 1, 90, dtype=int)
        names = [names[int(index)] for index in indices]
    for name in names:
        left = cv2.imread(str(cam0 / name), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(cam1 / name), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            continue
        left_corners, left_ids, _ = detector.detectMarkers(left)
        right_corners, right_ids, _ = detector.detectMarkers(right)
        # The 1920x1200 stored eye images make AprilGrid tags small enough that
        # the native detector can miss most of them. Retry at 1.5x only when
        # the first pass cannot establish a usable grid.
        if left_ids is None or len(left_ids) < 4:
            scaled = cv2.resize(left, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            left_corners, left_ids, _ = detector.detectMarkers(scaled)
        if right_ids is None or len(right_ids) < 4:
            scaled = cv2.resize(right, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            right_corners, right_ids, _ = detector.detectMarkers(scaled)
        if left_ids is None or right_ids is None:
            continue
        left_grid_corners = [
            corners for corners, marker_id in zip(left_corners, left_ids.reshape(-1))
            if 0 <= int(marker_id) < 48
        ]
        right_grid_corners = [
            corners for corners, marker_id in zip(right_corners, right_ids.reshape(-1))
            if 0 <= int(marker_id) < 48
        ]
        if len(left_grid_corners) < 4 or len(right_grid_corners) < 4:
            continue
        detections += 1
        center = np.asarray(left_grid_corners).reshape(-1, 2).mean(axis=0)
        column = min(2, max(0, int(center[0] / left.shape[1] * 3)))
        row = min(2, max(0, int(center[1] / left.shape[0] * 3)))
        cells.add((column, row))
    return detections, len(cells)


def validate_dataset(
    dataset_dir: Path, config: Mapping[str, object]
) -> ValidationReport:
    dataset = Path(dataset_dir)
    integrity_path = dataset / "integrity.json"
    integrity: dict[str, object] = {}
    if integrity_path.is_file():
        integrity = _read_json(integrity_path)
    manifest_path = dataset / "dataset_manifest.json"
    manifest: dict[str, object] = {}
    if manifest_path.is_file():
        manifest = _read_json(manifest_path)
    capture = _read_json(dataset / "capture.json")
    decoder = _read_json(dataset / "decoder_stats.json")
    cam0 = dataset / "cam0"
    cam1 = dataset / "cam1"
    cam0_names = {path.name for path in cam0.glob("*.png")}
    cam1_names = {path.name for path in cam1.glob("*.png")}
    common_names = cam0_names & cam1_names
    timestamps, gyro, accel = _read_imu(dataset / "imu0.csv")

    reasons = []
    if not manifest_path.is_file():
        reasons.append("缺少录制清单：dataset_manifest.json")
    if not integrity_path.is_file():
        reasons.append("缺少录制完整性报告：integrity.json")
    elif not bool(integrity.get("passed", False)) and not integrity.get("reasons"):
        reasons.append("录制完整性检查未通过")
    integrity_reasons = [str(item) for item in integrity.get("reasons", [])]
    reasons.extend(integrity_reasons)
    duration = float(capture.get("duration_seconds", 0.0))
    minimum_duration = float(config["minimum_duration_seconds"])
    if not bool(capture.get("complete", False)):
        reasons.append("录制未完整结束")
    if duration < minimum_duration:
        reasons.append(f"录制时长不足 {minimum_duration:g} 秒")
    if cam0_names != cam1_names:
        reasons.append("左右图像数量不一致")
    decode_ratio = float(decoder.get("decode_ratio", 0.0))
    minimum_ratio = float(config["minimum_decode_ratio"])
    decoded_frames = int(decoder.get("decoded_frames", capture.get("decoded_frames", 0)))
    imu_sample_count = int(decoder.get("imu_samples", capture.get("imu_samples", timestamps.size)))
    allow_override = bool(config.get("allow_decode_ratio_override", False))
    minimum_decoded_frames = int(config.get("minimum_decoded_frames", 0))
    minimum_imu_samples = int(config.get("minimum_imu_samples", 0))
    decode_ratio_overridden = bool(
        decode_ratio < minimum_ratio
        and allow_override
        and decoded_frames >= minimum_decoded_frames
        and imu_sample_count >= minimum_imu_samples
    )
    if decode_ratio < minimum_ratio and not decode_ratio_overridden:
        reasons.append(f"码带解码成功率低于 {minimum_ratio * 100:g}%")

    monotonic = bool(
        timestamps.size >= 2 and np.all(np.diff(timestamps) > 0)
    )
    if not monotonic:
        reasons.append("时间戳不单调")
    positive_deltas = np.diff(timestamps)
    positive_deltas = positive_deltas[positive_deltas > 0]
    imu_rate = (
        float(1_000_000_000.0 / np.median(positive_deltas))
        if positive_deltas.size
        else 0.0
    )
    minimum_rate = float(config["minimum_imu_rate_hz"])
    maximum_rate = float(config["maximum_imu_rate_hz"])
    if not minimum_rate <= imu_rate <= maximum_rate:
        reasons.append(f"IMU 频率不在 {minimum_rate:g}–{maximum_rate:g}Hz")

    gyro_spans = np.ptp(gyro, axis=0) if gyro.size else np.zeros(3)
    accel_spans = np.ptp(accel, axis=0) if accel.size else np.zeros(3)
    weak_gyro_axes = ["XYZ"[index] for index, value in enumerate(gyro_spans) if value < 0.35]
    weak_accel_axes = ["XYZ"[index] for index, value in enumerate(accel_spans) if value < 1.5]
    if weak_gyro_axes:
        reasons.append("陀螺仪运动激励不足：" + "/".join(weak_gyro_axes) + " 轴")
    if weak_accel_axes:
        reasons.append("加速度运动激励不足：" + "/".join(weak_accel_axes) + " 轴")

    board_detections, coverage_cells = _board_coverage(cam0, cam1, common_names)
    if board_detections < 60:
        reasons.append("双眼同时检测到 AprilGrid 的图像少于 60 帧")
    if coverage_cells < 5:
        reasons.append("AprilGrid 视场覆盖不足 5 个区域")

    metrics: dict[str, float | int | bool] = {
        "duration_seconds": duration,
        "left_images": len(cam0_names),
        "right_images": len(cam1_names),
        "decode_ratio": decode_ratio,
        "decoded_frames": decoded_frames,
        "decode_ratio_overridden": decode_ratio_overridden,
        "imu_samples": int(timestamps.size),
        "imu_rate_hz": imu_rate,
        "timestamps_monotonic": monotonic,
        "gyro_span_x": float(gyro_spans[0]),
        "gyro_span_y": float(gyro_spans[1]),
        "gyro_span_z": float(gyro_spans[2]),
        "accel_span_x": float(accel_spans[0]),
        "accel_span_y": float(accel_spans[1]),
        "accel_span_z": float(accel_spans[2]),
        "board_detections": board_detections,
        "coverage_cells": coverage_cells,
        "integrity_passed": bool(integrity.get("passed", False)) if integrity else False,
        "manifest_present": bool(manifest),
    }
    return ValidationReport(
        passed=not reasons,
        reasons=tuple(reasons),
        metrics=metrics,
    )
