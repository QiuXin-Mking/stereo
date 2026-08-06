from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
START = PROJECT_ROOT / "deploy" / "start.sh"


def test_deploy_start_uses_safe_fixed_endpoints():
    text = START.read_text(encoding="utf-8")

    assert "root@192.168.100.200" in text
    assert "127.0.0.1:18765:127.0.0.1:8765" in text
    assert "ExitOnForwardFailure=yes" in text
    assert "BatchMode=yes" in text


def test_deploy_start_checks_health_and_tracks_runtime():
    text = START.read_text(encoding="utf-8")

    assert "/api/status" in text
    assert 'RUNTIME_DIR="$SCRIPT_DIR/.runtime"' in text
    assert "tunnel.pid" in text
    assert "world_intelligent_calibrate.log" in text
    assert "setsid -f ./calibrate" in text


def test_deploy_start_is_executable():
    assert START.stat().st_mode & 0o111


def test_runtime_directory_is_ignored():
    ignore = (PROJECT_ROOT / ".gitignore").read_text(encoding="utf-8")

    assert "deploy/.runtime/" in ignore.splitlines()
