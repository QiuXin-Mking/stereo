from __future__ import annotations

import json
from pathlib import Path
from typing import Dict

import cv2
import numpy as np

from .models import AcceptedPair, CalibrationResult


SCHEMA_VERSION = "1.0"


def _matrix_json(matrix: np.ndarray) -> Dict[str, object]:
    array = np.asarray(matrix)
    if array.ndim == 1:
        array = array.reshape(-1, 1)
    return {
        "rows": int(array.shape[0]),
        "cols": int(array.shape[1]),
        "data": array.astype(float).ravel().tolist(),
    }


def _matrices(result: CalibrationResult) -> Dict[str, np.ndarray]:
    return {
        "left_camera_matrix": result.left_camera_matrix,
        "left_distortion": result.left_distortion,
        "right_camera_matrix": result.right_camera_matrix,
        "right_distortion": result.right_distortion,
        "rotation": result.rotation,
        "translation": result.translation,
        "essential": result.essential,
        "fundamental": result.fundamental,
        "rectification_left": result.rectification_left,
        "rectification_right": result.rectification_right,
        "projection_left": result.projection_left,
        "projection_right": result.projection_right,
        "disparity_to_depth": result.disparity_to_depth,
    }


def _write_yaml(path: Path, result: CalibrationResult, square_size_m: float) -> None:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise RuntimeError(f"无法写入 {path}")
    storage.write("schema_version", SCHEMA_VERSION)
    storage.write("image_width", int(result.image_size[0]))
    storage.write("image_height", int(result.image_size[1]))
    storage.write("square_size_m", float(square_size_m))
    storage.write("passed", int(result.passed))
    storage.write("mono_rms_left", result.mono_rms_left)
    storage.write("mono_rms_right", result.mono_rms_right)
    storage.write("stereo_rms", result.stereo_rms)
    storage.write("epipolar_median", result.epipolar_median)
    storage.write("epipolar_p95", result.epipolar_p95)
    for name, matrix in _matrices(result).items():
        storage.write(name, matrix)
    storage.release()


def _write_preview(path: Path, result: CalibrationResult, sample_pair: AcceptedPair) -> None:
    left = cv2.imread(str(sample_pair.left_path), cv2.IMREAD_COLOR)
    right = cv2.imread(str(sample_pair.right_path), cv2.IMREAD_COLOR)
    if left is None or right is None:
        raise RuntimeError("无法读取预览样本")
    left_rectified = cv2.remap(left, result.left_map1, result.left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right, result.right_map1, result.right_map2, cv2.INTER_LINEAR)
    preview = np.hstack([left_rectified, right_rectified])
    for y in range(0, preview.shape[0], 40):
        cv2.line(preview, (0, y), (preview.shape[1] - 1, y), (0, 255, 0), 1)
    if not cv2.imwrite(str(path), preview):
        raise RuntimeError(f"无法写入 {path}")


def export_result(
    result: CalibrationResult,
    output_dir: Path,
    sample_pair: AcceptedPair,
    square_size_m: float,
) -> None:
    """Write stable OpenCV, JSON, NumPy, maps, and preview artifacts."""
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_yaml(output_dir / "stereo_calibration.yaml", result, square_size_m)

    metadata = {
        "schema_version": SCHEMA_VERSION,
        "image_size": list(result.image_size),
        "square_size_m": square_size_m,
        "length_unit": "meter",
        "passed": result.passed,
        "quality": {
            "mono_rms_left": result.mono_rms_left,
            "mono_rms_right": result.mono_rms_right,
            "stereo_rms": result.stereo_rms,
            "epipolar_median": result.epipolar_median,
            "epipolar_p95": result.epipolar_p95,
            "valid_pair_count": result.valid_pair_count,
            "rejected_indices": result.rejected_indices,
            "failure_reasons": result.failure_reasons,
        },
        "matrices": {name: _matrix_json(matrix) for name, matrix in _matrices(result).items()},
    }
    (output_dir / "stereo_calibration.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    np.savez(
        output_dir / "stereo_calibration.npz",
        schema_version=np.asarray(SCHEMA_VERSION),
        image_size=np.asarray(result.image_size, dtype=np.int32),
        square_size_m=np.asarray(square_size_m),
        **_matrices(result),
    )
    np.savez(
        output_dir / "rectify_maps.npz",
        left_map1=result.left_map1,
        left_map2=result.left_map2,
        right_map1=result.right_map1,
        right_map2=result.right_map2,
    )
    maps = cv2.FileStorage(str(output_dir / "rectify_maps.yml.gz"), cv2.FILE_STORAGE_WRITE)
    if not maps.isOpened():
        raise RuntimeError("无法写入 rectify_maps.yml.gz")
    maps.write("image_width", int(result.image_size[0]))
    maps.write("image_height", int(result.image_size[1]))
    maps.write("left_map1", result.left_map1)
    maps.write("left_map2", result.left_map2)
    maps.write("right_map1", result.right_map1)
    maps.write("right_map2", result.right_map2)
    maps.release()
    quality = dict(metadata["quality"])
    quality["passed"] = result.passed
    (output_dir / "quality_report.json").write_text(
        json.dumps(quality, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    _write_preview(output_dir / "rectification_preview.png", result, sample_pair)
