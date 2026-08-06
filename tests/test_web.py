import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from stereo_calibrator.web import create_server


class FakeEngine:
    def __init__(self):
        self.actions = []

    def status_snapshot(self):
        return {
            "state": "capturing",
            "camera_label": "world intelligent",
            "code_band": "通过（160 px）",
            "accepted_pairs": 0,
            "manual_pairs": 0,
            "target_pairs": 32,
            "mode": "MJPG 3840x1080@30",
            "per_eye": "1920x1080",
            "reason": "等待棋盘",
            "stable_progress": 0.0,
            "guidance": "移到中央",
            "mono_rms_left": None,
            "mono_rms_right": None,
            "epipolar_p95": None,
            "result_dir": None,
            "error": None,
        }

    def latest_preview(self):
        return b"\xff\xd8fake-jpeg\xff\xd9"

    def action(self, name):
        self.actions.append(name)
        if name not in {"pause", "resume", "undo", "solve", "stop", "manual_capture"}:
            return {"ok": False, "error": "不支持的操作"}
        return {"ok": True}


@pytest.fixture
def running_server():
    engine = FakeEngine()
    server = create_server(engine, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    yield engine, base_url
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def post_json(url, payload):
    request = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    return urlopen(request, timeout=2)


def test_home_page_contains_preview_and_controls(running_server):
    _, base_url = running_server

    html = urlopen(base_url + "/", timeout=2).read().decode("utf-8")

    assert "/stream.mjpg" in html
    assert '<img id="preview" src="/stream.mjpg"' not in html
    assert "开始求解" in html
    assert "暂停" in html
    assert "手动拍摄" in html
    assert "act('manual_capture')" in html
    assert "手动素材" not in html
    assert 'id="manualCount"' not in html
    assert "s.manual_pairs" in html
    assert 'id="cameraLabel"' in html
    assert 'id="codeBand"' in html
    assert "s.camera_label" in html
    assert "s.code_band" in html
    assert "停止服务" not in html
    assert "act('stop')" not in html


def test_status_endpoint_returns_engine_snapshot(running_server):
    _, base_url = running_server

    payload = json.load(urlopen(base_url + "/api/status", timeout=2))

    assert payload["state"] == "capturing"
    assert payload["accepted_pairs"] == 0


def test_valid_action_reaches_engine(running_server):
    engine, base_url = running_server

    payload = json.load(post_json(base_url + "/api/action", {"action": "pause"}))

    assert payload == {"ok": True}
    assert engine.actions == ["pause"]


def test_manual_capture_action_reaches_engine(running_server):
    engine, base_url = running_server

    payload = json.load(post_json(base_url + "/api/action", {"action": "manual_capture"}))

    assert payload == {"ok": True}
    assert engine.actions == ["manual_capture"]


def test_action_endpoint_rejects_unknown_action(running_server):
    _, base_url = running_server

    with pytest.raises(HTTPError) as captured:
        post_json(base_url + "/api/action", {"action": "erase"})

    assert captured.value.code == 400
    assert json.load(captured.value)["error"] == "不支持的操作"


def test_mjpeg_stream_starts_with_frame_boundary(running_server):
    _, base_url = running_server

    response = urlopen(base_url + "/stream.mjpg", timeout=2)
    first_bytes = response.read(80)
    response.close()

    assert b"--frame" in first_bytes
    assert b"Content-Type: image/jpeg" in first_bytes
