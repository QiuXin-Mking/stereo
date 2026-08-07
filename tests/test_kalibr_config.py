import json

import pytest
import yaml

from stereo_calibrator.kalibr.config_writer import write_kalibr_configs
from stereo_calibrator.kalibr.validation import ValidationReport


def passing_report():
    return ValidationReport(
        passed=True,
        reasons=(),
        metrics={"imu_rate_hz": 300.0, "decode_ratio": 0.95},
    )


def test_writes_exact_target_and_provisional_imu_yaml(tmp_path):
    paths = write_kalibr_configs(tmp_path, passing_report(), session_id="session-1")
    target = yaml.safe_load(paths.target.read_text())
    imu = yaml.safe_load(paths.imu.read_text())

    assert target == {
        "target_type": "checkerboard",
        "targetCols": 8,
        "targetRows": 5,
        "rowSpacingMeters": 0.020,
        "colSpacingMeters": 0.020,
    }
    assert imu["rostopic"] == "/imu0"
    assert imu["update_rate"] == pytest.approx(300.0)
    assert imu["accelerometer_noise_density"] == 0.02
    assert imu["accelerometer_random_walk"] == 0.002
    assert imu["gyroscope_noise_density"] == 0.002
    assert imu["gyroscope_random_walk"] == 0.0002
    assert imu["provisional"] is True


def test_pipeline_manifest_is_pinned_and_atomic(tmp_path):
    paths = write_kalibr_configs(tmp_path, passing_report(), session_id="session-2")
    manifest = json.loads(paths.pipeline.read_text())

    assert manifest["topics"] == ["/cam0/image_raw", "/cam1/image_raw", "/imu0"]
    assert manifest["camera_models"] == ["pinhole-radtan", "pinhole-radtan"]
    assert manifest["kalibr_commit"] == "1f60227442d25e36365ef5f72cd80b9666d73467"
    assert manifest["session_id"] == "session-2"
    assert not list(tmp_path.glob("*.tmp"))
