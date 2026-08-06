import cv2
import numpy as np
import pytest

import stereo_calibrator.camera_backend as camera_backend
from stereo_calibrator.camera_backend import CameraMode, open_linux_camera


class FakeCapture:
    def __init__(self, width, height, fps, fourcc, opened=True):
        self.values = {
            cv2.CAP_PROP_FRAME_WIDTH: float(width),
            cv2.CAP_PROP_FRAME_HEIGHT: float(height),
            cv2.CAP_PROP_FPS: float(fps),
            cv2.CAP_PROP_FOURCC: float(cv2.VideoWriter_fourcc(*fourcc)),
        }
        self.opened = opened
        self.released = False

    def set(self, _prop, _value):
        return True

    def get(self, prop):
        return self.values.get(prop, 0.0)

    def isOpened(self):
        return self.opened

    def read(self):
        return True, np.zeros((16, 32, 3), dtype=np.uint8)

    def release(self):
        self.released = True


def test_linux_backend_rejects_silent_resolution_fallback(monkeypatch):
    fake = FakeCapture(1920, 1080, 30, "MJPG")
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_args: fake)

    with pytest.raises(RuntimeError, match="3840x1080"):
        open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG"))

    assert fake.released


def test_linux_backend_rejects_wrong_pixel_format(monkeypatch):
    fake = FakeCapture(3840, 1080, 30, "YUYV")
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_args: fake)

    with pytest.raises(RuntimeError, match="MJPG"):
        open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG"))


def test_linux_backend_accepts_exact_mjpg_mode(monkeypatch):
    fake = FakeCapture(3840, 1080, 30, "MJPG")
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_args: fake)

    capture = open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG"))

    assert capture is fake
    assert not fake.released


def test_linux_backend_reports_open_failure(monkeypatch):
    fake = FakeCapture(0, 0, 0, "MJPG", opened=False)
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_args: fake)

    with pytest.raises(RuntimeError, match="/dev/video0"):
        open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG"))


def test_mode_probe_returns_first_exact_supported_mode(monkeypatch):
    calls = []
    expected = CameraMode(3840, 1080, 30, "MJPG")
    fake = FakeCapture(3840, 1080, 30, "MJPG")

    def fake_open(_device, mode):
        calls.append(mode)
        if mode.width == 4000:
            raise RuntimeError("unsupported")
        return fake

    monkeypatch.setattr(camera_backend, "open_linux_camera", fake_open)

    capture, mode = camera_backend.open_first_supported_linux_camera(
        "/dev/video0",
        [CameraMode(4000, 1200, 30, "MJPG"), expected],
    )

    assert capture is fake
    assert mode == expected
    assert [item.width for item in calls] == [4000, 3840]


def test_linux_camera_name_reads_video_node_name(monkeypatch, tmp_path):
    name_file = tmp_path / "name"
    name_file.write_text("DECXIN Camera: DECXIN Camera\n")
    monkeypatch.setattr(camera_backend, "_video_name_path", lambda _device: name_file)

    assert camera_backend.linux_camera_name("/dev/video0") == "DECXIN Camera: DECXIN Camera"
