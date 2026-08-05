from pathlib import Path

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
