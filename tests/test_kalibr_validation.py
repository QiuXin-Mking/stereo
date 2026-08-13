import csv
import json

import cv2
import numpy as np
import pytest

from stereo_calibrator.kalibr.validation import validate_dataset


def default_config():
    return {
        "minimum_duration_seconds": 60,
        "minimum_decode_ratio": 0.90,
        "minimum_imu_rate_hz": 250,
        "maximum_imu_rate_hz": 400,
    }


def _checkerboard(offset):
    image = np.full((240, 320), 255, np.uint8)
    x0, y0 = offset
    square = 20
    for row in range(6):
        for column in range(9):
            color = 0 if (row + column) % 2 == 0 else 255
            image[
                y0 + row * square : y0 + (row + 1) * square,
                x0 + column * square : x0 + (column + 1) * square,
            ] = color
    return image


def _aprilgrid(offset):
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    board = cv2.aruco.GridBoard((8, 6), 40, 12, dictionary)
    target = board.generateImage((404, 300), marginSize=8, borderBits=1)
    image = np.full((900, 1200), 255, np.uint8)
    x0, y0 = offset
    image[y0:y0 + target.shape[0], x0:x0 + target.shape[1]] = target
    return image


def make_dataset(root, mutation=None, with_boards=False):
    dataset = root / "kalibr"
    cam0 = dataset / "cam0"
    cam1 = dataset / "cam1"
    cam0.mkdir(parents=True)
    cam1.mkdir()
    duration = 59.0 if mutation == "short" else 61.0
    (dataset / "capture.json").write_text(
        json.dumps({"complete": True, "duration_seconds": duration}), encoding="utf-8"
    )
    ratio = 0.89 if mutation == "decode_89_percent" else 0.95
    (dataset / "decoder_stats.json").write_text(
        json.dumps({"decode_ratio": ratio}), encoding="utf-8"
    )
    positions = ((20, 20), (398, 20), (776, 20), (20, 570), (776, 570))
    image_count = 60 if with_boards else 1
    for index in range(image_count):
        image = _aprilgrid(positions[index % len(positions)]) if with_boards else np.zeros((8, 8), np.uint8)
        cv2.imwrite(str(cam0 / f"{index}.png"), image)
        cv2.imwrite(str(cam1 / f"{index}.png"), image)
    if mutation == "pair_mismatch":
        cv2.imwrite(str(cam0 / "extra.png"), np.zeros((8, 8), np.uint8))

    rate = 200 if mutation == "imu_200hz" else 300
    sample_count = int(duration * rate) + 1
    step_ns = int(1_000_000_000 / rate)
    with (dataset / "imu0.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "timestamp_ns", "omega_x", "omega_y", "omega_z",
            "alpha_x", "alpha_y", "alpha_z", "frame_idx", "t_us",
        ))
        for index in range(sample_count):
            timestamp = index * step_ns
            if mutation == "timestamp_regression" and index == 10:
                timestamp = 0
            sign = -1.0 if index % 2 else 1.0
            writer.writerow((
                timestamp,
                0.4 * sign, 0.5 * sign, 0.6 * sign,
                1.0 * sign, 1.2 * sign, 9.8 + 1.0 * sign,
                index // 10, timestamp // 1000,
            ))
    return dataset


@pytest.mark.parametrize("mutation,reason", [
    ("short", "录制时长不足 60 秒"),
    ("pair_mismatch", "左右图像数量不一致"),
    ("decode_89_percent", "码带解码成功率低于 90%"),
    ("imu_200hz", "IMU 频率不在 250–400Hz"),
    ("timestamp_regression", "时间戳不单调"),
])
def test_validation_rejects_mandatory_failures(tmp_path, mutation, reason):
    report = validate_dataset(make_dataset(tmp_path, mutation), default_config())

    assert report.passed is False
    assert reason in report.reasons


def test_validation_accepts_excited_dataset_with_board_coverage(tmp_path):
    report = validate_dataset(
        make_dataset(tmp_path, with_boards=True), default_config()
    )

    assert report.passed is True
    assert report.reasons == ()
    assert report.metrics["board_detections"] >= 60
    assert report.metrics["coverage_cells"] >= 5
    assert 299 < report.metrics["imu_rate_hz"] < 301


def test_validation_accepts_aprilgrid_used_by_kalibr(tmp_path):
    dataset = make_dataset(tmp_path)
    cam0 = dataset / "cam0"
    cam1 = dataset / "cam1"
    for path in (*cam0.glob("*.png"), *cam1.glob("*.png")):
        path.unlink()
    positions = ((20, 20), (398, 20), (776, 20), (20, 570), (776, 570))
    for index in range(60):
        image = _aprilgrid(positions[index % len(positions)])
        cv2.imwrite(str(cam0 / f"{index}.png"), image)
        cv2.imwrite(str(cam1 / f"{index}.png"), image)

    report = validate_dataset(dataset, default_config())

    assert report.passed is True
    assert report.metrics["board_detections"] >= 60
    assert report.metrics["coverage_cells"] >= 5
