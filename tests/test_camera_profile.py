import numpy as np
import pytest

from stereo_calibrator.camera_backend import CameraMode
from stereo_calibrator.camera_profile import detect_camera_profile, split_profile_frame


def world_frame():
    frame = np.full((1200, 4000, 3), 100, np.uint8)
    frame[:, :160] = 0
    frame[:, 160:2080] = 50
    frame[:, 2080:] = 100
    return frame


def test_three_matching_conditions_assign_world_intelligent():
    profile = detect_camera_profile(
        "DECXIN Camera: DECXIN Camera",
        world_frame(),
        CameraMode(4000, 1200, 30, "MJPG"),
    )

    assert profile.label == "world intelligent"
    assert profile.code_band_status == "通过（160 px）"
    assert profile.per_eye_size == (1920, 1200)


@pytest.mark.parametrize(
    ("name", "shape"),
    [("Other Camera", (1200, 4000)), ("DECXIN Camera", (1080, 3840))],
)
def test_name_or_resolution_mismatch_does_not_assign_world_label(name, shape):
    frame = np.full((*shape, 3), 100, np.uint8)
    mode = CameraMode(shape[1], shape[0], 30, "MJPG")

    assert detect_camera_profile(name, frame, mode).label == "generic stereo"


def test_matching_name_and_size_reject_missing_code_band():
    frame = np.full((1200, 4000, 3), 100, np.uint8)

    with pytest.raises(RuntimeError, match="码带识别失败"):
        detect_camera_profile("DECXIN Camera", frame, CameraMode(4000, 1200, 30, "MJPG"))


def test_world_split_removes_code_band_and_preserves_exact_pixels():
    frame = world_frame()
    profile = detect_camera_profile(
        "DECXIN Camera", frame, CameraMode(4000, 1200, 30, "MJPG")
    )

    left, right = split_profile_frame(frame, profile)

    assert left.shape == right.shape == (1200, 1920, 3)
    assert np.all(left == 50)
    assert np.all(right == 100)


def test_generic_split_and_swap_remain_compatible():
    frame = np.zeros((1080, 3840, 3), np.uint8)
    frame[:, :1920] = 1
    frame[:, 1920:] = 2
    profile = detect_camera_profile(
        "UVC Camera 1", frame, CameraMode(3840, 1080, 30, "MJPG")
    )

    left, right = split_profile_frame(frame, profile, swap_eyes=True)

    assert left.shape == right.shape == (1080, 1920, 3)
    assert np.all(left == 2)
    assert np.all(right == 1)
