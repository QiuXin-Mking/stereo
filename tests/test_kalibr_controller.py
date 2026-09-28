import json
from pathlib import Path
import threading

import numpy as np

from stereo_calibrator.kalibr.controller import KalibrController


class FakeRecorder:
    def __init__(self, session_dir):
        self.session_dir = Path(session_dir)
        self.dataset_dir = self.session_dir / "kalibr"
        self.state = "ready"
        self.ingested = 0

    def start(self):
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self.state = "recording"

    def ingest(self, *_args):
        if self.state == "recording":
            self.ingested += 1

    def stop(self):
        self.state = "recorded"
        (self.dataset_dir / "integrity.json").write_text(
            json.dumps({"passed": True, "reasons": []}), encoding="utf-8"
        )

    def abort(self, _reason):
        self.state = "error"

    def snapshot(self):
        return {"state": self.state, "total_frames": self.ingested}


def make_controller(tmp_path):
    return KalibrController(tmp_path / "session-1", {}, recorder_factory=FakeRecorder)


def test_happy_path_and_ingest(tmp_path):
    controller = make_controller(tmp_path)
    assert controller.action("kalibr_start")["ok"] is True
    assert controller.snapshot()["state"] == "recording"
    frame = np.zeros((4, 4), np.uint8)
    controller.ingest(frame, frame, frame, 1)
    assert controller.snapshot()["total_frames"] == 1
    assert controller.action("kalibr_stop")["ok"] is True
    assert controller.snapshot()["state"] == "recorded"


def test_validate_and_solve_are_unsupported(tmp_path):
    controller = make_controller(tmp_path)
    controller.action("kalibr_start")
    controller.action("kalibr_stop")

    for action in ("kalibr_validate", "kalibr_solve"):
        result = controller.action(action)
        assert result["ok"] is False
        assert "仅支持录制" in result["error"]


def test_start_backs_up_existing_dataset(tmp_path):
    controller = make_controller(tmp_path)
    controller.action("kalibr_start")
    controller.action("kalibr_stop")
    old_dataset = tmp_path / "session-1/kalibr"
    assert old_dataset.exists()

    assert controller.action("kalibr_start")["ok"] is True
    backups = list((tmp_path / "session-1").glob("kalibr_backup_*"))
    assert len(backups) == 1
    assert (backups[0] / "integrity.json").is_file()


def test_stop_waits_for_inflight_ingest_before_closing_recorder(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    stopped = threading.Event()

    class BlockingRecorder(FakeRecorder):
        def ingest(self, *_args):
            entered.set()
            assert release.wait(timeout=2)
            super().ingest(*_args)

        def stop(self):
            super().stop()
            stopped.set()

    controller = KalibrController(
        tmp_path / "session-1", {}, recorder_factory=BlockingRecorder
    )
    controller.action("kalibr_start")
    frame = np.zeros((4, 4), np.uint8)
    ingest_thread = threading.Thread(
        target=controller.ingest, args=(frame, frame, frame, 1)
    )
    stop_thread = threading.Thread(target=controller.action, args=("kalibr_stop",))
    ingest_thread.start()
    assert entered.wait(timeout=1)
    stop_thread.start()

    assert not stopped.wait(timeout=0.1)
    release.set()
    ingest_thread.join(timeout=2)
    stop_thread.join(timeout=2)
    assert stopped.is_set()
    assert controller.snapshot()["state"] == "recorded"
