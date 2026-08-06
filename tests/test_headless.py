import json
import time

import numpy as np

import stereo_calibrator.headless as headless
from stereo_calibrator.headless import HeadlessCalibrationEngine
from stereo_calibrator.models import PoseFeatures, QualityDecision


class FakeCamera:
    def __init__(self):
        self.released = False

    def read(self):
        return True, np.zeros((1080, 3840, 3), dtype=np.uint8)

    def release(self):
        self.released = True


class WorldCamera(FakeCamera):
    def read(self):
        frame = np.full((1200, 4000, 3), 100, np.uint8)
        frame[:, :160] = 0
        return True, frame


class BurstCamera(FakeCamera):
    def __init__(self):
        super().__init__()
        self.index = 0

    def read(self):
        self.index += 1
        frame = np.full((1080, 3840, 3), 100, np.uint8)
        if self.index % 5 == 0:
            frame[:, ::8] = 255
        return True, frame


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


def test_solve_accepts_twenty_saved_manual_pairs(tmp_path):
    manual = tmp_path / "manual"
    manual.mkdir()
    for index in range(20):
        (manual / f"{index:04d}_left.png").touch()
        (manual / f"{index:04d}_right.png").touch()
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    assert engine.action("solve") == {"ok": True}


def test_existing_manual_pairs_are_restored_in_web_status(tmp_path):
    manual = tmp_path / "manual"
    manual.mkdir()
    for index in range(20):
        (manual / f"{index:04d}_left.png").touch()
        (manual / f"{index:04d}_right.png").touch()

    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    assert engine.status_snapshot()["manual_pairs"] == 20
    assert engine.status_snapshot()["guidance"] == "第 21/32 组：棋盘向右偏航"


def test_load_manual_pairs_detects_corners_and_skips_bad_pair(tmp_path, monkeypatch):
    manual = tmp_path / "manual"
    manual.mkdir()
    image = np.full((120, 192, 3), 127, np.uint8)
    for index in range(2):
        headless.cv2.imwrite(str(manual / f"{index:04d}_left.png"), image)
        headless.cv2.imwrite(str(manual / f"{index:04d}_right.png"), image)
    corners = np.zeros((40, 2), np.float32)
    detections = iter((corners, corners, None, corners))
    monkeypatch.setattr(
        headless,
        "detect_chessboard_with_retry",
        lambda *_args: (next(detections), "raw"),
    )
    decision = QualityDecision(True, "质量通过", {"sharpness": 100.0}, PoseFeatures(0.5, 0.5, 0.2))
    monkeypatch.setattr(headless, "evaluate_pair", lambda *_args: decision)
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    pairs, rejected = engine._load_manual_pairs((8, 5))

    assert [pair.index for pair in pairs] == [0]
    assert rejected == [1]


def test_pause_resume_and_invalid_action(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    assert engine.action("pause")["ok"]
    assert engine.status_snapshot()["state"] == "paused"
    assert engine.action("resume")["ok"]
    assert engine.status_snapshot()["state"] == "capturing"
    assert engine.action("erase") == {"ok": False, "error": "不支持的操作"}


def test_auto_capture_switch_defaults_on_and_can_be_toggled(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    assert engine.status_snapshot()["auto_capture_enabled"] is True
    assert engine.action("auto_off") == {"ok": True}
    assert engine.status_snapshot()["auto_capture_enabled"] is False
    assert engine.action("auto_on") == {"ok": True}
    assert engine.status_snapshot()["auto_capture_enabled"] is True


def test_status_snapshot_is_json_safe(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())

    encoded = json.dumps(engine.status_snapshot(), ensure_ascii=False)

    assert '"device": "/dev/video0"' in encoded
    assert '"mode": "探测中"' in encoded
    assert '"camera_label": "detecting"' in encoded


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
    metadata = json.loads((tmp_path / "manual" / "0000_metadata.json").read_text())
    assert metadata["source"] == "manual"
    assert metadata["corners_detected"] is False
    assert metadata["sharpness"] >= 0
    assert camera.released


def test_manual_capture_selects_sharpest_of_five_frames(tmp_path, monkeypatch):
    camera = BurstCamera()
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=camera)
    monkeypatch.setattr(headless, "detect_chessboard", lambda *_args: None)
    engine.start()

    assert engine.action("manual_capture") == {"ok": True}
    deadline = time.monotonic() + 3.0
    while engine.status_snapshot()["manual_pairs"] < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    engine.action("stop")
    engine.join(2)

    saved = headless.cv2.imread(str(tmp_path / "manual" / "0000_left.png"))
    assert saved is not None
    assert saved.std() > 10


def test_manual_guidance_advances_through_fixed_capture_sequence():
    assert headless.manual_guidance(0) == "第 1/32 组：棋盘放在中央并正对相机"
    assert headless.manual_guidance(2) == "第 3/32 组：将棋盘移到左侧"
    assert headless.manual_guidance(10) == "第 11/32 组：将棋盘移到左上角"
    assert headless.manual_guidance(30) == "第 31/32 组：将棋盘靠近相机"
    assert headless.manual_guidance(32) == "本轮 32 组采集完成"


def test_manual_capture_rejects_after_target(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())
    engine._status.update(state="capturing", manual_pairs=32)

    assert engine.action("manual_capture") == {"ok": False, "error": "本轮 32 组已采集完成"}


def test_manual_capture_does_not_queue_beyond_target(tmp_path):
    engine = HeadlessCalibrationEngine(make_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())
    engine._status.update(state="capturing", manual_pairs=31)

    assert engine.action("manual_capture") == {"ok": True}
    assert engine.action("manual_capture") == {"ok": False, "error": "本轮 32 组已采集完成"}


def test_world_camera_resolves_status_and_cropped_preview(tmp_path, monkeypatch):
    engine = HeadlessCalibrationEngine(
        make_config(),
        tmp_path,
        0.020,
        "/dev/video0",
        camera=WorldCamera(),
        device_name="DECXIN Camera",
    )
    monkeypatch.setattr(headless, "detect_chessboard", lambda *_args: None)

    engine.start()
    deadline = time.monotonic() + 2
    while engine.status_snapshot()["camera_label"] == "detecting" and time.monotonic() < deadline:
        time.sleep(0.01)
    engine.action("stop")
    engine.join(2)
    status = engine.status_snapshot()

    assert status["camera_label"] == "world intelligent"
    assert status["mode"] == "MJPG 4000x1200@30"
    assert status["per_eye"] == "1920x1200"
    assert status["code_band"] == "通过（160 px）"


def test_matching_world_camera_without_band_enters_error(tmp_path):
    camera = WorldCamera()
    camera.read = lambda: (True, np.full((1200, 4000, 3), 100, np.uint8))
    engine = HeadlessCalibrationEngine(
        make_config(),
        tmp_path,
        0.020,
        "/dev/video0",
        camera=camera,
        device_name="DECXIN Camera",
    )

    engine.start()
    engine.join(2)

    assert engine.status_snapshot()["state"] == "error"
    assert "码带识别失败" in engine.status_snapshot()["error"]
