import cv2
import numpy as np
import pytest

import stereo_calibrator.capture as capture_module
from stereo_calibrator.capture import open_highest_camera
from stereo_calibrator.detector import pose_features
from stereo_calibrator.quality import evaluate_pair


def regular_corners(offset_x=0.0):
    xs, ys = np.meshgrid(np.linspace(35, 125, 9), np.linspace(30, 90, 6))
    return np.column_stack([xs.ravel() + offset_x, ys.ravel()]).astype(np.float32)


def textured_image():
    image = np.full((120, 160), 40, dtype=np.uint8)
    image[::4, :] = 210
    image[:, ::4] = 210
    return image


def thresholds():
    return {
        "minimum_sharpness": 50.0,
        "maximum_clipped_ratio": 0.60,
        "minimum_edge_margin_ratio": 0.02,
        "minimum_novelty": 0.10,
    }


def test_blurry_pair_is_rejected():
    gray = np.full((120, 160), 127, dtype=np.uint8)
    corners = regular_corners()

    decision = evaluate_pair(gray, gray, corners, corners, [], thresholds())

    assert not decision.accepted
    assert decision.reason == "图像模糊"


def test_overexposed_pair_is_rejected_before_blur():
    gray = np.full((120, 160), 255, dtype=np.uint8)
    corners = regular_corners()

    decision = evaluate_pair(gray, gray, corners, corners, [], thresholds())

    assert not decision.accepted
    assert decision.reason == "曝光异常"


def test_board_too_close_to_edge_is_rejected():
    gray = textured_image()
    corners = regular_corners(offset_x=-33)

    decision = evaluate_pair(gray, gray, corners, corners, [], thresholds())

    assert not decision.accepted
    assert decision.reason == "棋盘距离画面边缘太近"


def test_duplicate_pose_is_rejected():
    gray = textured_image()
    corners = regular_corners()
    history = [pose_features(corners, (160, 120))]

    decision = evaluate_pair(gray, gray, corners, corners, history, thresholds())

    assert not decision.accepted
    assert "重复" in decision.reason


def test_distinct_sharp_pair_is_accepted():
    gray = textured_image()
    corners = regular_corners(offset_x=20)
    old = pose_features(regular_corners(offset_x=-20), (160, 120))

    decision = evaluate_pair(gray, gray, corners, corners, [old], thresholds())

    assert decision.accepted
    assert decision.reason == "质量通过"
    assert decision.metrics["sharpness"] > 50


def test_macos_camera_open_failure_includes_permission_path(monkeypatch):
    class ClosedCapture:
        def set(self, *_args):
            return False

        def isOpened(self):
            return False

        def release(self):
            return None

    monkeypatch.setattr(capture_module.cv2, "VideoCapture", lambda *_args: ClosedCapture())
    with pytest.raises(RuntimeError, match="隐私与安全性.*相机"):
        open_highest_camera(0)
