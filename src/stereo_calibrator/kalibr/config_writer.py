from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import yaml

from .validation import ValidationReport


KALIBR_COMMIT = "1f60227442d25e36365ef5f72cd80b9666d73467"


@dataclass(frozen=True)
class KalibrConfigPaths:
    target: Path
    imu: Path
    pipeline: Path


def _atomic_text(path: Path, text: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def write_kalibr_configs(
    dataset_dir: Path,
    report: ValidationReport,
    session_id: str,
) -> KalibrConfigPaths:
    if not report.passed:
        raise ValueError("数据质量未通过，不能生成 Kalibr 配置")
    dataset = Path(dataset_dir)
    dataset.mkdir(parents=True, exist_ok=True)
    target_path = dataset / "target.yaml"
    imu_path = dataset / "imu.yaml"
    pipeline_path = dataset / "pipeline.json"

    target = {
        "target_type": "aprilgrid",
        "tagCols": 8,
        "tagRows": 6,
        "tagSize": 0.020,
        "tagSpacing": 0.30,
    }
    imu = {
        "rostopic": "/imu0",
        "update_rate": float(report.metrics["imu_rate_hz"]),
        "accelerometer_noise_density": 0.02,
        "accelerometer_random_walk": 0.002,
        "gyroscope_noise_density": 0.002,
        "gyroscope_random_walk": 0.0002,
        "provisional": True,
    }
    pipeline = {
        "session_id": str(session_id),
        "kalibr_commit": KALIBR_COMMIT,
        "topics": ["/cam0/image_raw", "/cam1/image_raw", "/imu0"],
        "camera_models": ["pinhole-radtan", "pinhole-radtan"],
        "target": {
            "type": "aprilgrid",
            "tag_columns": 8,
            "tag_rows": 6,
            "tag_size_m": 0.020,
            "tag_spacing": 0.30,
        },
        "quality": report.metrics,
        "provisional_imu_noise": True,
    }
    _atomic_text(
        target_path,
        yaml.safe_dump(target, allow_unicode=True, sort_keys=False),
    )
    _atomic_text(
        imu_path,
        yaml.safe_dump(imu, allow_unicode=True, sort_keys=False),
    )
    _atomic_text(
        pipeline_path,
        json.dumps(pipeline, ensure_ascii=False, indent=2),
    )
    return KalibrConfigPaths(target=target_path, imu=imu_path, pipeline=pipeline_path)
