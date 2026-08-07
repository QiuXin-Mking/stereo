from __future__ import annotations

import json
from pathlib import Path
import subprocess
from typing import Optional


class KalibrRuntime:
    """Small Docker adapter kept separate from the web/camera threads."""

    def __init__(self, image: str) -> None:
        self.image = str(image)

    def launch(self, dataset_dir: Path, session_id: str) -> str:
        name = f"stereo-kalibr-{session_id}"
        command = [
            "docker", "run", "-d", "--name", name,
            "--label", f"stereo.kalibr.session={session_id}",
            "--network", "none", "--memory", "6g", "--read-only",
            "-v", f"{Path(dataset_dir).resolve()}:/data/kalibr:rw",
            self.image,
        ]
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip() or "Kalibr 容器启动失败")
        return name

    def status(self, container_name: str) -> dict[str, object]:
        completed = subprocess.run(
            ["docker", "inspect", "--format", "{{json .State}}", container_name],
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            return {"exists": False, "running": False, "exit_code": None}
        try:
            state = json.loads(completed.stdout)
        except json.JSONDecodeError:
            return {"exists": True, "running": False, "exit_code": None}
        exit_code: Optional[int] = None
        if not bool(state.get("Running", False)):
            exit_code = int(state.get("ExitCode", 1))
        return {
            "exists": True,
            "running": bool(state.get("Running", False)),
            "exit_code": exit_code,
        }

    def logs(self, container_name: str, tail: int = 100) -> str:
        completed = subprocess.run(
            ["docker", "logs", "--tail", str(int(tail)), container_name],
            capture_output=True,
            text=True,
        )
        return completed.stdout + completed.stderr

