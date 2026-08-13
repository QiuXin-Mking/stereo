from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import shutil
import threading
from typing import Callable, Mapping, Optional

from .config_writer import write_kalibr_configs
from .recorder import KalibrRecorder
from .runtime import KalibrRuntime
from .validation import ValidationReport, validate_dataset


ARTIFACTS = {
    "summary": "results/summary.json",
    "target": "target.yaml",
    "imu": "imu.yaml",
    "pipeline": "pipeline.json",
    "bag": "results/dataset.bag",
    "camchain": "results/camchain.yaml",
    "camchain_imu": "results/camchain-imucam.yaml",
    "camera_report": "results/dataset-report-cam.pdf",
    "imu_report": "results/dataset-report-imucam.pdf",
    "camera_log": "results/cameras.log",
    "imu_log": "results/imu-camera.log",
}

PIPELINE_STATES = {
    "bagging", "camera_calibrating", "imu_calibrating", "pass", "retake", "error"
}


class KalibrController:
    def __init__(
        self,
        session_dir: Path,
        config: Mapping[str, object],
        *,
        recorder_factory: Callable[[Path], object] = KalibrRecorder,
        validator: Callable[[Path, Mapping[str, object]], ValidationReport] = validate_dataset,
        config_writer: Callable[[Path, ValidationReport, str], object] = write_kalibr_configs,
        runtime: Optional[KalibrRuntime] = None,
    ) -> None:
        self.session_dir = Path(session_dir)
        self.session_id = self.session_dir.name
        self.dataset_dir = self.session_dir / "kalibr"
        self.config = dict(config)
        self._recorder_factory = recorder_factory
        self._recorder = recorder_factory(self.session_dir)
        self._validator = validator
        self._config_writer = config_writer
        self._runtime = runtime or KalibrRuntime(
            "stereo-kalibr-rk3588:1f60227442d25e36365ef5f72cd80b9666d73467"
        )
        self._lock = threading.RLock()
        self._state = "ready"
        self._reasons: list[str] = []
        self._metrics: dict[str, object] = {}
        self._container_name: Optional[str] = None
        self._error: Optional[str] = None
        self._restore_job()

    def action(self, name: str) -> dict[str, object]:
        with self._lock:
            try:
                if name == "kalibr_start":
                    return self._start()
                if name == "kalibr_stop":
                    return self._stop()
                if name == "kalibr_validate":
                    return self._validate()
                if name == "kalibr_solve":
                    return self._solve()
                return {"ok": False, "error": f"未知 Kalibr 操作：{name}"}
            except (OSError, RuntimeError, ValueError) as error:
                self._state = "error"
                self._error = str(error)
                self._persist_job()
                return {"ok": False, "error": self._error}

    def ingest(self, raw_frame, left, right, frame_idx: int) -> None:
        with self._lock:
            if self._state == "recording":
                self._recorder.ingest(raw_frame, left, right, int(frame_idx))

    def abort(self, reason: str) -> None:
        if self._state == "recording":
            self._recorder.abort(reason)
        self._state = "error"
        self._error = str(reason)
        self._persist_job()

    def snapshot(self) -> dict[str, object]:
        self._poll_pipeline()
        recorder = self._recorder.snapshot()
        result = {
            **recorder,
            "state": self._state,
            "validation_reasons": list(self._reasons),
            "validation_metrics": dict(self._metrics),
            "container_name": self._container_name,
            "error": self._error,
            "artifacts": self.artifacts(),
        }
        if self._container_name:
            result["logs"] = self._runtime.logs(self._container_name, tail=100)
        else:
            result["logs"] = ""
        return result

    def artifacts(self) -> dict[str, str]:
        return {
            key: filename
            for key, filename in ARTIFACTS.items()
            if (self.dataset_dir / filename).is_file()
        }

    def artifact_path(self, key: str) -> Path:
        filename = ARTIFACTS.get(str(key))
        if filename is None:
            raise ValueError("非法产物")
        path = self.dataset_dir / filename
        if not path.is_file() or self.dataset_dir.resolve() not in path.resolve().parents:
            raise ValueError("非法产物")
        return path

    def _start(self) -> dict[str, object]:
        if self._state == "recording":
            return {"ok": True, "reused": True}
        if self._state not in {"ready", "retake", "error", "pass"}:
            raise RuntimeError("当前状态不能开始录制")
        if self.dataset_dir.exists():
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup = self.session_dir / f"kalibr_backup_{stamp}"
            suffix = 1
            while backup.exists():
                backup = self.session_dir / f"kalibr_backup_{stamp}_{suffix}"
                suffix += 1
            shutil.move(str(self.dataset_dir), str(backup))
            self._recorder = self._recorder_factory(self.session_dir)
        self._recorder.start()
        self._state = "recording"
        self._reasons = []
        self._metrics = {}
        self._container_name = None
        self._error = None
        return {"ok": True}

    def _stop(self) -> dict[str, object]:
        if self._state != "recording":
            raise RuntimeError("当前没有正在进行的 Kalibr 录制")
        self._recorder.stop()
        self._state = "recorded"
        self._persist_job()
        return {"ok": True}

    def _validate(self) -> dict[str, object]:
        if self._state not in {"recorded", "retake"}:
            raise RuntimeError("请先停止录制再检查数据")
        self._state = "validating"
        report = self._validator(self.dataset_dir, self.config)
        self._reasons = list(report.reasons)
        self._metrics = dict(report.metrics)
        self._atomic_json(self.dataset_dir / "validation.json", asdict(report))
        if not report.passed:
            self._state = "retake"
            self._persist_job()
            return {"ok": False, "reasons": list(report.reasons)}
        self._config_writer(self.dataset_dir, report, self.session_id)
        self._state = "ready_to_solve"
        self._persist_job()
        return {"ok": True}

    def _solve(self) -> dict[str, object]:
        if self._container_name:
            status = self._runtime.status(self._container_name)
            if bool(status.get("running", False)):
                return {"ok": True, "reused": True}
        if self._state in {"recorded", "retake"}:
            validation = self._validate()
            if not bool(validation.get("ok")):
                return validation
        if self._state != "ready_to_solve":
            raise RuntimeError("数据尚未通过检查，不能求解")
        self._container_name = self._runtime.launch(self.dataset_dir, self.session_id)
        self._state = "bagging"
        self._persist_job()
        return {"ok": True}

    def _poll_pipeline(self) -> None:
        stage_path = self.dataset_dir / "stage.json"
        if stage_path.is_file():
            try:
                stage = json.loads(stage_path.read_text(encoding="utf-8"))
                state = str(stage.get("stage", stage.get("state", "")))
                if state in PIPELINE_STATES:
                    self._state = state
                if state == "error":
                    self._error = str(stage.get("message", "Kalibr 求解失败"))
            except (OSError, json.JSONDecodeError):
                pass
        if self._container_name and self._state not in {"pass", "retake", "error"}:
            status = self._runtime.status(self._container_name)
            if bool(status.get("exists")) and not bool(status.get("running")):
                if status.get("exit_code") == 0:
                    self._state = "pass"
                else:
                    self._state = "error"
                    self._error = "Kalibr 容器异常退出"
                self._persist_job()

    def _restore_job(self) -> None:
        path = self.dataset_dir / "job.json"
        if not path.is_file():
            return
        try:
            job = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        self._state = str(job.get("state", "error"))
        self._container_name = job.get("container_name") or None
        self._reasons = [str(reason) for reason in job.get("validation_reasons", [])]
        self._metrics = dict(job.get("validation_metrics", {}))
        self._error = job.get("error") or None

    def _persist_job(self) -> None:
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self._atomic_json(
            self.dataset_dir / "job.json",
            {
                "session_id": self.session_id,
                "state": self._state,
                "container_name": self._container_name,
                "validation_reasons": self._reasons,
                "validation_metrics": self._metrics,
                "error": self._error,
            },
        )

    @staticmethod
    def _atomic_json(path: Path, payload: object) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(path)
