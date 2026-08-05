# RK3588 Headless Web Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the existing SBS chessboard calibration on RK3588 using strict V4L2 MJPEG 3840×1080@30 capture and a browser-based headless control surface, then deploy and verify it through Git over SSH.

**Architecture:** A platform-specific camera opener feeds a platform-independent `HeadlessCalibrationEngine`. The engine owns the camera and calibration state in one worker thread; a standard-library `ThreadingHTTPServer` only exposes immutable status snapshots, the latest resized MJPEG preview, and validated control actions.

**Tech Stack:** Python 3.9, OpenCV/NumPy/PyYAML, Linux V4L2, Python `http.server`, HTML/JavaScript, pytest, Git/SSH, OpenCV C++ 4.5.1.

## Global Constraints

- RK3588 input is `/dev/video0`, MJPEG `3840×1080 @ 30 FPS`; any actual-mode mismatch is fatal.
- Full-resolution SBS frames are used for detection, saving, and solving; only browser previews are resized.
- Do not install or replace OpenCV on RK3588; use its existing Python 5.0.0 and C++ 4.5.1 installations.
- Web server binds `0.0.0.0:8765`, has no authentication, and is intended only for the trusted local network.
- HTTP actions are restricted to `pause`, `resume`, `undo`, `solve`, and `stop`.
- Solving is rejected below 20 accepted pairs; target remains 32 pairs.
- Existing macOS AVFoundation and offline solver/export behavior must remain green.
- Each task follows red-green TDD and ends with a Git commit.

---

### Task 1: Strict Linux V4L2 Camera Backend

**Files:**
- Create: `src/stereo_calibrator/camera_backend.py`
- Modify: `src/stereo_calibrator/capture.py`
- Create: `tests/test_camera_backend.py`

**Interfaces:**
- Produces: `CameraMode(width: int, height: int, fps: float, fourcc: str)`.
- Produces: `open_linux_camera(device: str, mode: CameraMode) -> cv2.VideoCapture`.
- Produces: `actual_camera_mode(cap: cv2.VideoCapture) -> CameraMode`.
- Produces: `open_platform_camera(...) -> cv2.VideoCapture`, retaining AVFoundation behavior on macOS.

- [ ] **Step 1: Write failing backend tests**

```python
def test_linux_backend_rejects_silent_resolution_fallback(monkeypatch):
    fake = FakeCapture(width=1920, height=1080, fps=30, fourcc="MJPG")
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_: fake)
    with pytest.raises(RuntimeError, match="3840x1080"):
        open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG"))

def test_linux_backend_accepts_exact_mjpg_mode(monkeypatch):
    fake = FakeCapture(width=3840, height=1080, fps=30, fourcc="MJPG")
    monkeypatch.setattr(camera_backend.cv2, "VideoCapture", lambda *_: fake)
    assert open_linux_camera("/dev/video0", CameraMode(3840, 1080, 30, "MJPG")) is fake
```

- [ ] **Step 2: Run red test**

Run: `.venv/bin/python -m pytest tests/test_camera_backend.py -q`  
Expected: import failure for `camera_backend`.

- [ ] **Step 3: Implement exact mode negotiation**

Open Linux using `cv2.VideoCapture(device, cv2.CAP_V4L2)`, set FOURCC before width/height/FPS, read warm-up frames, then compare returned width, height, FPS tolerance ±1, and FOURCC. On mismatch release and raise with requested and actual modes. Move macOS open logic behind `open_platform_camera` and make the existing GUI capture use it.

- [ ] **Step 4: Run Task 1 and regression tests**

Run: `.venv/bin/python -m pytest tests/test_camera_backend.py tests/test_quality.py -q`  
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/stereo_calibrator/camera_backend.py src/stereo_calibrator/capture.py tests/test_camera_backend.py
git commit -m "feat: add strict RK3588 V4L2 camera backend"
```

### Task 2: Headless Calibration Engine

**Files:**
- Create: `src/stereo_calibrator/headless.py`
- Modify: `src/stereo_calibrator/models.py`
- Create: `tests/test_headless.py`

**Interfaces:**
- Consumes: platform camera backend, SBS split, detector, quality gate, solver, and exporter.
- Produces: `HeadlessCalibrationEngine(config, session_dir, square_size_m, device)`, with `start()`, `status_snapshot()`, `latest_preview()`, `action(name)`, and `join()`.
- Produces states: `starting`, `capturing`, `paused`, `solving`, `pass`, `retake`, `error`, `stopped`.

- [ ] **Step 1: Write failing state/action tests**

```python
def test_solve_is_rejected_below_twenty_pairs(tmp_path):
    engine = HeadlessCalibrationEngine(test_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())
    response = engine.action("solve")
    assert response == {"ok": False, "error": "至少需要 20 对图像"}

def test_pause_resume_and_invalid_action(tmp_path):
    engine = HeadlessCalibrationEngine(test_config(), tmp_path, 0.020, "/dev/video0", camera=FakeCamera())
    assert engine.action("pause")["ok"]
    assert engine.status_snapshot()["state"] == "paused"
    assert engine.action("resume")["ok"]
    assert engine.action("erase") == {"ok": False, "error": "不支持的操作"}
```

- [ ] **Step 2: Run red test**

Run: `.venv/bin/python -m pytest tests/test_headless.py -q`  
Expected: import failure for `HeadlessCalibrationEngine`.

- [ ] **Step 3: Implement worker-owned state and preview**

The worker reads exact-mode frames, detects both boards, evaluates quality, maintains the stable timer, saves accepted raw/left/right images and metadata, and JPEG-encodes a 1440-pixel-wide annotated preview. Status snapshots contain JSON-safe values only. Auto-solve at target; manual solve is queued at 20+ pairs. `undo` removes only the latest saved pair outside `solving`; `stop` sets the stop event and always releases the camera.

- [ ] **Step 4: Run headless and solver regression tests**

Run: `.venv/bin/python -m pytest tests/test_headless.py tests/test_solver_exporter.py -q`  
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/stereo_calibrator/headless.py src/stereo_calibrator/models.py tests/test_headless.py
git commit -m "feat: add headless calibration engine"
```

### Task 3: Browser MJPEG Control Server

**Files:**
- Create: `src/stereo_calibrator/web.py`
- Create: `tests/test_web.py`

**Interfaces:**
- Consumes: the five `HeadlessCalibrationEngine` methods from Task 2.
- Produces: `create_server(engine, host: str, port: int) -> ThreadingHTTPServer`.
- Produces endpoints `GET /`, `GET /api/status`, `GET /stream.mjpg`, and `POST /api/action`.

- [ ] **Step 1: Write failing HTTP contract tests**

```python
def test_status_endpoint_returns_engine_snapshot(running_server):
    payload = json.load(urlopen(running_server.url + "/api/status"))
    assert payload["state"] == "capturing"
    assert payload["accepted_pairs"] == 0

def test_action_endpoint_rejects_unknown_action(running_server):
    response = post_json(running_server.url + "/api/action", {"action": "erase"})
    assert response.status == 400
    assert json.load(response)["error"] == "不支持的操作"
```

- [ ] **Step 2: Run red test**

Run: `.venv/bin/python -m pytest tests/test_web.py -q`  
Expected: import failure for `create_server`.

- [ ] **Step 3: Implement standard-library HTTP server and page**

Serve embedded UTF-8 HTML with two-eye MJPEG preview, status cards, quality reason, stable progress, guidance text, metrics, result path, and five action buttons. Poll `/api/status` once per second. MJPEG waits for new preview bytes and sends multipart frames until disconnect. Reject invalid JSON, unknown paths, and actions outside the allowlist with JSON errors and suitable HTTP status.

- [ ] **Step 4: Run web contract tests**

Run: `.venv/bin/python -m pytest tests/test_web.py -q`  
Expected: all pass without opening a camera.

- [ ] **Step 5: Commit**

```bash
git add src/stereo_calibrator/web.py tests/test_web.py
git commit -m "feat: add browser calibration console"
```

### Task 4: CLI, Git Deployment, and RK3588 Verification

**Files:**
- Modify: `src/stereo_calibrator/cli.py`
- Modify: `README.md`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `HeadlessCalibrationEngine` and `create_server`.
- Produces CLI arguments `--web`, `--device`, `--host`, and `--port`.

- [ ] **Step 1: Write failing CLI web dry-run test**

```python
def test_web_dry_run_prints_rk_mode(capsys):
    code = main(["--web", "--device", "/dev/video0", "--square-mm", "20", "--dry-run"])
    assert code == 0
    assert "3840x1080@30" in capsys.readouterr().out
```

- [ ] **Step 2: Run red test**

Run: `.venv/bin/python -m pytest tests/test_cli.py -q`  
Expected: argparse returns 2 because `--web` and `--device` are not defined.

- [ ] **Step 3: Implement web orchestration and documentation**

In web mode create the timestamped session, strict Linux camera, headless engine, and HTTP server. Print `http://<host>:<port>` and block in `serve_forever`; on Ctrl-C call `engine.action("stop")`, join, close the server, and return 0. Document RK3588 setup, Git deploy, URL, firewall assumption, and stop procedure.

- [ ] **Step 4: Run full local verification and commit**

Run: `.venv/bin/python -m pytest -q`  
Run: `.venv/bin/python -m compileall -q src`  
Run: `./calibrate --web --device /dev/video0 --square-mm 20 --dry-run`  
Expected: all tests pass, compile succeeds, and dry-run prints MJPEG 3840×1080@30 plus port 8765.

```bash
git add src/stereo_calibrator/cli.py README.md tests/test_cli.py
git commit -m "feat: expose RK3588 web calibration mode"
```

- [ ] **Step 5: Create RK3588 Git remote and deploy**

```bash
ssh root@192.168.100.200 'mkdir -p /root/git && git init --bare /root/git/stereo_chessboard_calibrator.git'
git remote add rk3588 root@192.168.100.200:/root/git/stereo_chessboard_calibrator.git
git push -u rk3588 main
ssh root@192.168.100.200 'git clone /root/git/stereo_chessboard_calibrator.git /root/stereo_chessboard_calibrator'
```

- [ ] **Step 6: Prepare remote environment and run offline/C++ tests**

```bash
ssh root@192.168.100.200 'cd /root/stereo_chessboard_calibrator && python3 -m venv --system-site-packages .venv && .venv/bin/python -m pip install PyYAML pytest && .venv/bin/python -m pytest -q && cmake -S examples/cpp -B examples/cpp/build && cmake --build examples/cpp/build -j2'
```

- [ ] **Step 7: Run exact-mode camera smoke test**

Run a 120-frame RK3588 script through `open_linux_camera`, assert every returned frame is 3840×1080, report measured FPS and release the device. Expected: 120/120 frames, no mismatch or read failure.

- [ ] **Step 8: Start web service and verify endpoints**

Start `./calibrate --web --device /dev/video0 --square-mm 20 --host 0.0.0.0 --port 8765` in a persistent SSH PTY. From the Mac, `curl` `/`, `/api/status`, and the first bytes of `/stream.mjpg`; post an invalid action and confirm HTTP 400. Leave the service running for browser validation at `http://192.168.100.200:8765`.

