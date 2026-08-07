from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_start_keeps_direct_tunnel_and_checks_runtime():
    text = (PROJECT_ROOT / "deploy/start.sh").read_text(encoding="utf-8")
    assert "127.0.0.1:18765:127.0.0.1:8765" in text
    assert "/api/status" in text and "kalibr" in text.lower()
    assert "18766" not in text


def test_installer_validates_archive_and_never_installs_ros_on_host():
    text = (PROJECT_ROOT / "deploy/install-kalibr-runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "sha256sum -c" in text
    assert "docker load" in text
    assert "uname -m" in text
    assert "4194304" in text
    assert "kalibr_calibrate_cameras --help" in text
    assert "kalibr_calibrate_imu_camera --help" in text
    assert "apt install ros-" not in text
    assert "desktop-full" not in text


def test_docs_describe_both_complete_workflows():
    readme = (PROJECT_ROOT / "README.md").read_text(encoding="utf-8")
    manual = (PROJECT_ROOT / "docs/零基础使用手册.md").read_text(encoding="utf-8")
    for text in (readme, manual):
        assert "OpenCV Chessboard" in text
        assert "Kalibr Camera+IMU" in text
        assert "开始录制" in text
        assert "检查数据" in text
        assert "开始 Kalibr 求解" in text
    assert "还没有接入 `manual/`" not in manual
    assert "拍完后不要点击“开始求解”" not in manual
