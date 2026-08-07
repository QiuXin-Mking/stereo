#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


def _camera_summary(camera: dict[str, Any]) -> dict[str, Any]:
    fx, fy, cx, cy = (float(value) for value in camera["intrinsics"])
    return {
        "model": camera["camera_model"],
        "distortion_model": camera["distortion_model"],
        "resolution": camera["resolution"],
        "K": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        "D": camera["distortion_coeffs"],
    }


def _matrix(value: Any) -> list[list[float]]:
    matrix = [[float(item) for item in row] for row in value]
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("Kalibr 外参矩阵不是 4x4")
    return matrix


def summarize(results_dir: Path, output_path: Path) -> dict[str, Any]:
    results = Path(results_dir)
    camera_chain = yaml.safe_load((results / "camchain.yaml").read_text(encoding="utf-8"))
    imu_chain = yaml.safe_load((results / "camchain-imucam.yaml").read_text(encoding="utf-8"))
    stereo_transform = _matrix(camera_chain["cam1"]["T_cn_cnm1"])
    translation = [stereo_transform[index][3] for index in range(3)]
    cam0_imu = _matrix(imu_chain["cam0"]["T_cam_imu"])
    bag_info_path = results / "bag_info.json"
    bag_info = (
        json.loads(bag_info_path.read_text(encoding="utf-8"))
        if bag_info_path.is_file()
        else None
    )
    summary = {
        "format_version": 1,
        "provisional_imu_noise": True,
        "cameras": {
            "cam0": _camera_summary(camera_chain["cam0"]),
            "cam1": _camera_summary(camera_chain["cam1"]),
        },
        "stereo": {
            "T_cam1_cam0": stereo_transform,
            "R": [row[:3] for row in stereo_transform[:3]],
            "T_m": translation,
            "baseline_m": math.sqrt(sum(value * value for value in translation)),
        },
        "imu": {
            "T_cam0_imu": cam0_imu,
            "time_offset_seconds": float(imu_chain["cam0"]["timeshift_cam_imu"]),
        },
        "bag": bag_info,
    }
    output = Path(output_path)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    temporary.replace(output)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    summarize(arguments.results, arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

