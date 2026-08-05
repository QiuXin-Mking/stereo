import json
import time

import numpy as np

from stereo_calibrator.headless import HeadlessCalibrationEngine


class FakeCamera:
    def __init__(self):
        self.released = False

    def read(self):
        return True, np.zeros((1080, 3840, 3), dtype=np.uint8)

    def release(self):
        self.released = True


def make_config():
    return {
        "device": {"name": "RK camera"},
        "board": {"columns": 9, "rows": 6},
        "capture": {
            "target_pairs": 32,
            "minimum_pairs": 20,
            "maximum_pairs": 40,
            "stable_seconds": 0.8,
            "preview_width_per_eye": 720,
        },
        "quality": {
            "minimum_sharpness": 80.0,
            "maximum_clipped_ratio": 0.08,
            "minimum_edge_margin_ratio": 0.025,
            "minimum_novelty": 0.12,
        },
        "validation": {
            "maximum_mono_rms": 0.7,
            "maximum_epipolar_median": 0.35,
            "maximum_epipolar_p95": 1.0,
            "maximum_outlier_fraction": 0.2,
        },
    }


def test_solve_is_rejected_below_twenty_pairs(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    response = engine.action("solve")

    assert response == {"ok": False, "error": "至少需要 20 对图像"}


def test_pause_resume_and_invalid_action(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    assert engine.action("pause")["ok"]
    assert engine.status_snapshot()["state"] == "paused"
    assert engine.action("resume")["ok"]
    assert engine.status_snapshot()["state"] == "capturing"
    assert engine.action("erase") == {"ok": False, "error": "不支持的操作"}


def test_status_snapshot_is_json_safe(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    encoded = json.dumps(engine.status_snapshot(), ensure_ascii=False)

    assert '"device": "/dev/video0"' in encoded
    assert '"mode": "MJPG 3840x1080@30"' in encoded


def test_stop_changes_state_without_starting_worker(tmp_path):
    camera = FakeCamera()
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=camera)

    assert engine.action("stop")["ok"]

    assert engine.status_snapshot()["state"] == "stopped"


def test_manual_capture_saves_next_frame_without_accepting_it(tmp_path):
    camera = FakeCamera()
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=camera)
    engine.start()

    response = engine.action("manual_capture")
    deadline = time.monotonic() + 3.0
    while engine.status_snapshot().get("manual_pairs", 0) < 1 and time.monotonic() < deadline:
        time.sleep(0.02)
    engine.action("stop")
    engine.join(timeout=2)

    assert response == {"ok": True}
    assert engine.status_snapshot()["manual_pairs"] == 1
    assert engine.status_snapshot()["accepted_pairs"] == 0
    assert (tmp_path / "manual" / "0000_sbs.jpg").stat().st_size > 0
    assert (tmp_path / "manual" / "0000_left.png").stat().st_size > 0
    assert (tmp_path / "manual" / "0000_right.png").stat().st_size > 0
    assert camera.released
