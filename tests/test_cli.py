from pathlib import Path

import stereo_calibrator.cli as cli
from stereo_calibrator.cli import main


def test_cli_help_mentions_default_device(capsys):
    assert main(["--help"]) == 0

    output = capsys.readouterr().out
    assert "UVC Camera 1" in output
    assert "--square-mm" in output


def test_cli_rejects_nonpositive_square_size(capsys):
    assert main(["--square-mm", "0", "--dry-run"]) == 2

    assert "方格边长" in capsys.readouterr().err


def test_cli_dry_run_creates_no_session(tmp_path, capsys):
    assert main(["--square-mm", "20.05", "--session-root", str(tmp_path), "--dry-run"]) == 0

    output = capsys.readouterr().out
    assert "9x6" in output
    assert "20.050 mm" in output
    assert list(Path(tmp_path).iterdir()) == []


def test_web_dry_run_prints_rk_mode(tmp_path, capsys):
    code = main(
        [
            "--web",
            "--device",
            "/dev/video0",
            "--square-mm",
            "20",
            "--session-root",
            str(tmp_path),
            "--dry-run",
        ]
    )

    assert code == 0
    output = capsys.readouterr().out
    assert "RK3588 模式=自动探测" in output
    assert "4000x1200/3840x1080 MJPG@30" in output
    assert "http://0.0.0.0:8765" in output
    assert list(Path(tmp_path).iterdir()) == []


def test_backup_previous_web_session_copies_latest_manual_session(tmp_path):
    sessions = tmp_path / "sessions"
    backups = tmp_path / "backups"
    (sessions / "20260805_100000" / "manual").mkdir(parents=True)
    latest_manual = sessions / "20260805_110000" / "manual"
    latest_manual.mkdir(parents=True)
    (latest_manual / "0000_sbs.jpg").write_bytes(b"latest-frame")

    result = cli.backup_previous_web_session(sessions, backups)

    assert result == backups / "20260805_110000"
    assert (result / "manual" / "0000_sbs.jpg").read_bytes() == b"latest-frame"
    assert (latest_manual / "0000_sbs.jpg").read_bytes() == b"latest-frame"


def test_backup_previous_web_session_never_overwrites_existing_backup(tmp_path):
    sessions = tmp_path / "sessions"
    backups = tmp_path / "backups"
    manual = sessions / "20260805_110000" / "manual"
    manual.mkdir(parents=True)
    (manual / "0000_sbs.jpg").write_bytes(b"new-data")
    existing = backups / "20260805_110000"
    existing.mkdir(parents=True)
    (existing / "sentinel.txt").write_bytes(b"keep")

    result = cli.backup_previous_web_session(sessions, backups)

    assert result is None
    assert (existing / "sentinel.txt").read_bytes() == b"keep"
    assert not (existing / "manual").exists()
