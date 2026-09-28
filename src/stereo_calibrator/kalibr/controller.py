from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import shutil
import threading
from typing import Callable, Mapping, Optional

from .recorder import KalibrRecorder


class KalibrController:
    """Recording-only state machine for Kalibr Camera+IMU capture."""

    def __init__(
        self,
        session_dir: Path,
        config: Mapping[str, object],
        *,
        recorder_factory: Callable[[Path], object] = KalibrRecorder,
    ) -> None:
        self.session_dir = Path(session_dir)
        self.session_id = self.session_dir.name
        self.dataset_dir = self.session_dir / "kalibr"
        self.config = dict(config)
        self._recorder_factory = recorder_factory
        self._recorder = recorder_factory(self.session_dir)
        self._configure_recorder()
        self._lock = threading.RLock()
        self._state = "ready"
        self._reasons: list[str] = []

    def _configure_recorder(self) -> None:
        configure = getattr(self._recorder, "configure", None)
        if callable(configure):
            configure(self.config)

    def action(self, name: str) -> dict[str, object]:
        with self._lock:
            try:
                if name == "kalibr_start":
                    return self._start()
                if name == "kalibr_stop":
                    return self._stop()
                if name in {"kalibr_validate", "kalibr_solve"}:
                    return {"ok": False, "error": "当前版本仅支持录制，不支持 Kalibr 求解"}
                return {"ok": False, "error": f"未知 Kalibr 操作：{name}"}
            except RuntimeError as error:
                return {"ok": False, "state": self._state, "error": str(error)}
            except (OSError, ValueError) as error:
                self._state = "error"
                return {"ok": False, "state": self._state, "error": str(error)}

    def ingest(self, raw_frame, left, right, frame_idx: int) -> None:
        with self._lock:
            if self._state == "recording":
                self._recorder.ingest(raw_frame, left, right, int(frame_idx))

    def set_startup_info(self, info: Mapping[str, object]) -> None:
        setter = getattr(self._recorder, "set_startup_info", None)
        if callable(setter):
            setter(dict(info))

    def abort(self, reason: str) -> None:
        if self._state == "recording":
            self._recorder.abort(reason)
        self._state = "error"

    def snapshot(self) -> dict[str, object]:
        recorder = self._recorder.snapshot()
        return {
            **recorder,
            "state": self._state,
            "reasons": list(self._reasons),
        }

    def _start(self) -> dict[str, object]:
        if self._state == "recording":
            return {"ok": True, "state": self._state, "reused": True}
        if self._state not in {"ready", "recorded", "retake", "error"}:
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
            self._configure_recorder()
        self._recorder.start()
        self._state = "recording"
        self._reasons = []
        return {"ok": True, "state": self._state}

    def _stop(self) -> dict[str, object]:
        if self._state != "recording":
            raise RuntimeError("当前没有正在进行的 Kalibr 录制")
        self._recorder.stop()
        integrity_path = self.dataset_dir / "integrity.json"
        integrity = self._read_json(integrity_path) if integrity_path.is_file() else {}
        if not integrity_path.is_file():
            integrity = {
                "passed": False,
                "reasons": ["停止录制后缺少完整性报告：integrity.json"],
            }
        self._state = "recorded" if bool(integrity.get("passed", False)) else "retake"
        self._reasons = [str(item) for item in integrity.get("reasons", [])]
        return self._result(self._state == "recorded", self._reasons)

    def _result(self, ok: bool, reasons: Optional[list[str]] = None) -> dict[str, object]:
        return {
            "ok": bool(ok),
            "state": self._state,
            "reasons": list(reasons or []),
            "dataset_dir": str(self.dataset_dir),
        }

    @staticmethod
    def _read_json(path: Path) -> dict[str, object]:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return value if isinstance(value, dict) else {}
