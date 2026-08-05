from pathlib import Path
import shutil
import subprocess


def test_launcher_includes_offline_remote_dependencies(tmp_path):
    project = tmp_path / "project"
    package = project / "src" / "stereo_calibrator"
    dependencies = project / ".remote-deps"
    package.mkdir(parents=True)
    dependencies.mkdir()
    shutil.copy2(Path(__file__).resolve().parents[1] / "calibrate", project / "calibrate")
    (project / "calibrate").chmod(0o755)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "cli.py").write_text(
        "import remote_probe\nprint(remote_probe.VALUE)\n", encoding="utf-8"
    )
    (dependencies / "remote_probe.py").write_text("VALUE = 'offline-deps-loaded'\n", encoding="utf-8")

    completed = subprocess.run(
        [str(project / "calibrate")], capture_output=True, text=True, check=False
    )

    assert completed.returncode == 0
    assert completed.stdout.strip() == "offline-deps-loaded"


def test_launcher_skips_broken_virtualenv(tmp_path):
    project = tmp_path / "project"
    package = project / "src" / "stereo_calibrator"
    dependencies = project / ".remote-deps"
    broken_python = project / ".venv" / "bin" / "python"
    package.mkdir(parents=True)
    dependencies.mkdir()
    broken_python.parent.mkdir(parents=True)
    shutil.copy2(Path(__file__).resolve().parents[1] / "calibrate", project / "calibrate")
    (project / "calibrate").chmod(0o755)
    broken_python.write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
    broken_python.chmod(0o755)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "cli.py").write_text(
        "import remote_probe\nprint(remote_probe.VALUE)\n", encoding="utf-8"
    )
    (dependencies / "remote_probe.py").write_text("VALUE = 'system-python-used'\n", encoding="utf-8")

    completed = subprocess.run(
        [str(project / "calibrate")], capture_output=True, text=True, check=False
    )

    assert completed.returncode == 0
    assert completed.stdout.strip() == "system-python-used"
