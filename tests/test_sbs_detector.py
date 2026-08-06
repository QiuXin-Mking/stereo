import numpy as np
import pytest

import stereo_calibrator.detector as detector
from stereo_calibrator.detector import detect_chessboard_with_retry, pose_features
from stereo_calibrator.sbs import split_sbs


def test_split_sbs_preserves_left_and_right_halves():
    frame = np.zeros((4, 8, 3), dtype=np.uint8)
    frame[:, :4] = 10
    frame[:, 4:] = 20

    left, right = split_sbs(frame)

    assert left.shape == right.shape == (4, 4, 3)
    assert int(left.mean()) == 10
    assert int(right.mean()) == 20


def test_split_sbs_swaps_eyes_when_requested():
    frame = np.zeros((4, 8, 3), dtype=np.uint8)
    frame[:, :4] = 10
    frame[:, 4:] = 20

    left, right = split_sbs(frame, swap_eyes=True)

    assert int(left.mean()) == 20
    assert int(right.mean()) == 10


def test_split_sbs_rejects_odd_width():
    with pytest.raises(ValueError, match="even"):
        split_sbs(np.zeros((4, 7, 3), dtype=np.uint8))


def test_pose_features_are_normalized():
    corners = np.array([[10, 10], [30, 10], [10, 30], [30, 30]], dtype=np.float32)

    feature = pose_features(corners, (100, 50))

    assert feature.center_x == pytest.approx(0.2)
    assert feature.center_y == pytest.approx(0.4)
    assert feature.area_ratio == pytest.approx(0.08)


def test_pose_features_rejects_empty_corners():
    with pytest.raises(ValueError, match="corners"):
        pose_features(np.empty((0, 2), dtype=np.float32), (100, 50))


def test_detection_retry_returns_raw_without_enhancement(monkeypatch):
    corners = np.zeros((40, 2), np.float32)
    calls = []
    monkeypatch.setattr(detector, "detect_chessboard", lambda image, _pattern: calls.append(image) or corners)

    result, method = detect_chessboard_with_retry(np.zeros((40, 40), np.uint8), (8, 5))

    assert result is corners
    assert method == "raw"
    assert len(calls) == 1


def test_detection_retry_uses_clahe_after_raw_failure(monkeypatch):
    corners = np.zeros((40, 2), np.float32)
    results = iter((None, corners))
    monkeypatch.setattr(detector, "detect_chessboard", lambda *_args: next(results))

    result, method = detect_chessboard_with_retry(np.zeros((40, 40), np.uint8), (8, 5))

    assert result is corners
    assert method == "clahe"
