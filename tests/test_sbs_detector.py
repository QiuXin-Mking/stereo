import numpy as np
import pytest

from stereo_calibrator.detector import pose_features
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
