import json
from pathlib import Path
import threading

import numpy as np
import pytest

from stereo_calibrator.kalibr.controller import KalibrController
from stereo_calibrator.kalibr.runtime import KalibrRuntime
from stereo_calibrator.kalibr.validation import ValidationReport


class FakeRecorder:
    def __init__(self, session_dir):
        self.dataset_dir = Path(session_dir) / "kalibr"
        self.state = "ready"
        self.ingested = 0

    def start(self):
        self.dataset_dir.mkdir(parents=True)
        self.state = "recording"

    def ingest(self, *_args):
        if self.state == "recording":
            self.ingested += 1

    def stop(self):
        self.state = "recorded"

    def abort(self, _reason):
        self.state = "error"

    def snapshot(self):
        return {"state": self.state, "total_frames": self.ingested}


class FakeRuntime:
    def __init__(self, running=False):
        self.running = running
        self.launches = 0

    def launch(self, _dataset_dir, session_id):
        self.launches += 1
        self.running = True
        return f"stereo-kalibr-{session_id}"

    def status(self, _name):
        return {"exists": True, "running": self.running, "exit_code": None}

    def logs(self, _name, tail=100):
        return f"last {tail} lines"


def passing_report():
    return ValidationReport(
        passed=True,
        reasons=(),
        metrics={"imu_rate_hz": 320.0},
    )


def make_controller(tmp_path, runtime=None, validator=lambda *_: passing_report()):
    return KalibrController(
        tmp_path / "session-1",
        {
            "minimum_duration_seconds": 60,
            "minimum_decode_ratio": 0.9,
            "minimum_imu_rate_hz": 250,
            "maximum_imu_rate_hz": 400,
        },
        recorder_factory=FakeRecorder,
        validator=validator,
        config_writer=lambda dataset, _report, _session: (
            (Path(dataset) / "target.yaml").write_text("target", encoding="utf-8")
        ),
        runtime=runtime or FakeRuntime(),
    )


def test_happy_path_and_ingest(tmp_path):
    controller = make_controller(tmp_path)
    assert controller.action("kalibr_start") == {"ok": True}
    assert controller.snapshot()["state"] == "recording"
    frame = np.zeros((4, 4), np.uint8)
    controller.ingest(frame, frame, frame, 1)
    assert controller.snapshot()["total_frames"] == 1
    assert controller.action("kalibr_stop") == {"ok": True}
    assert controller.action("kalibr_validate") == {"ok": True}
    assert controller.snapshot()["state"] == "ready_to_solve"
    assert controller.action("kalibr_solve") == {"ok": True}
    assert controller.snapshot()["state"] == "bagging"


def test_solve_automatically_validates_recorded_dataset(tmp_path):
    runtime = FakeRuntime()
    controller = make_controller(tmp_path, runtime=runtime)
    controller.action("kalibr_start")
    controller.action("kalibr_stop")

    result = controller.action("kalibr_solve")

    assert result == {"ok": True}
    assert controller.snapshot()["state"] == "bagging"
    assert runtime.launches == 1


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
        tmp_path / "session-1",
        {},
        recorder_factory=BlockingRecorder,
        runtime=FakeRuntime(),
    )
    controller.action("kalibr_start")
    frame = np.zeros((4, 4), np.uint8)
    ingest_thread = threading.Thread(
        target=controller.ingest, args=(frame, frame, frame, 1)
    )
    stop_thread = threading.Thread(
        target=controller.action, args=("kalibr_stop",)
    )
    ingest_thread.start()
    assert entered.wait(timeout=1)
    stop_thread.start()

    assert not stopped.wait(timeout=0.1)
    release.set()
    ingest_thread.join(timeout=2)
    stop_thread.join(timeout=2)
    assert stopped.is_set()
    assert controller.snapshot()["state"] == "recorded"


def test_validation_failure_preserves_reasons(tmp_path):
    report = ValidationReport(False, ("码带解码成功率低于 90%",), {})
    controller = make_controller(tmp_path, validator=lambda *_: report)
    controller.action("kalibr_start")
    controller.action("kalibr_stop")

    result = controller.action("kalibr_validate")

    assert result == {"ok": False, "reasons": ["码带解码成功率低于 90%"]}
    assert controller.snapshot()["state"] == "retake"
    assert controller.snapshot()["validation_reasons"] == ["码带解码成功率低于 90%"]


def test_repeated_solve_reuses_running_job_after_restart(tmp_path):
    runtime = FakeRuntime(running=True)
    controller = make_controller(tmp_path, runtime=runtime)
    controller.action("kalibr_start")
    controller.action("kalibr_stop")
    controller.action("kalibr_validate")
    assert controller.action("kalibr_solve") == {"ok": True}
    assert runtime.launches == 1

    restored = make_controller(tmp_path, runtime=runtime)
    assert restored.action("kalibr_solve") == {"ok": True, "reused": True}
    assert runtime.launches == 1


def test_stage_file_and_artifact_allow_list(tmp_path):
    controller = make_controller(tmp_path)
    controller.action("kalibr_start")
    dataset = tmp_path / "session-1/kalibr"
    (dataset / "results").mkdir()
    (dataset / "results/summary.json").write_text("{}", encoding="utf-8")
    (dataset / "stage.json").write_text(
        json.dumps({"stage": "pass", "message": "done"}), encoding="utf-8"
    )

    assert controller.snapshot()["state"] == "pass"
    assert "summary" in controller.artifacts()
    assert controller.artifact_path("summary") == dataset / "results/summary.json"
    with pytest.raises(ValueError, match="非法产物"):
        controller.artifact_path("../../etc/passwd")


def test_runtime_command_is_isolated_and_bounded(tmp_path, monkeypatch):
    calls = []

    class Completed:
        returncode = 0
        stdout = "container-id\n"
        stderr = ""

    monkeypatch.setattr("subprocess.run", lambda command, **kwargs: calls.append(command) or Completed())
    runtime = KalibrRuntime("stereo-kalibr:test")

    assert runtime.launch(tmp_path, "abc") == "stereo-kalibr-abc"
    command = calls[0]
    assert "--network" in command and "none" in command
    assert "--memory" in command and "6g" in command
    assert "--read-only" in command
    assert f"{tmp_path.resolve()}:/data/kalibr:rw" in command
