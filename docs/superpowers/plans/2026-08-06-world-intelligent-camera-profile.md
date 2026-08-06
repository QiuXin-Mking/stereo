# World Intelligent Camera Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add automatic three-condition detection for the DECXIN `4000×1200` coded-band camera, label it `world intelligent`, and apply its `160 + 1920 + 1920` crop only after successful identification.

**Architecture:** A focused camera-profile module owns identity, coded-band validation, and profile-specific splitting. The Linux backend probes the two supported capture modes, while the headless engine classifies the first valid frame and publishes the resolved profile to the existing web status API. Existing `3840×1080` equal-width SBS behavior remains the generic path.

**Tech Stack:** Python 3.9+, OpenCV, NumPy, pytest, V4L2, existing `http.server` web UI.

## Global Constraints

- Set `world intelligent` only when device name contains `DECXIN Camera`, frame size is exactly `4000×1200`, and the first `160` pixels pass coded-band validation.
- Use coded-band thresholds `dark_ratio >= 0.35` and `dark_ratio - adjacent_dark_ratio >= 0.25`, with grayscale values below `12` counted as dark.
- Split the identified frame as left `frame[:, 160:2080]` and right `frame[:, 2080:4000]`, producing two `1920×1200` images without resizing or enhancement.
- A matching name and resolution with a failed coded-band check is an error; do not silently use equal splitting.
- Preserve the existing generic `3840×1080 -> 1920×1080 + 1920×1080` path.
- Probe `MJPG 4000×1200@30` and `MJPG 3840×1080@30`; accept only an exact mode.
- Keep the existing board configuration unchanged in this feature.

---

## File Map

- Create `src/stereo_calibrator/camera_profile.py`: profile data, three-condition classifier, coded-band validation, and profile-aware splitting.
- Create `tests/test_camera_profile.py`: positive and negative identity cases plus exact crop assertions.
- Modify `src/stereo_calibrator/camera_backend.py`: V4L2 device-name lookup and ordered exact-mode probing.
- Modify `tests/test_camera_backend.py`: device-name and mode-probe tests.
- Modify `src/stereo_calibrator/headless.py`: first-frame profile resolution, status fields, dynamic mode validation, and profile splitting.
- Modify `tests/test_headless.py`: generic compatibility, world-intelligent status, and failed-band behavior.
- Modify `src/stereo_calibrator/web.py`: display label and coded-band status.
- Modify `tests/test_web.py`: verify the new fields are rendered and bound.
- Modify `README.md`: document automatic camera recognition and the fixed crop.

### Task 1: Camera Profile Classification and Exact Splitting

**Files:**
- Create: `src/stereo_calibrator/camera_profile.py`
- Create: `tests/test_camera_profile.py`

**Interfaces:**
- Consumes: `CameraMode` from `stereo_calibrator.camera_backend`, a V4L2 device name, and the first decoded frame.
- Produces: `CameraProfile`, `has_world_intelligent_code_band(frame) -> bool`, `detect_camera_profile(device_name, frame, mode) -> CameraProfile`, and `split_profile_frame(frame, profile, swap_eyes=False) -> tuple[np.ndarray, np.ndarray]`.

- [ ] **Step 1: Write failing profile tests**

```python
import numpy as np
import pytest

from stereo_calibrator.camera_backend import CameraMode
from stereo_calibrator.camera_profile import detect_camera_profile, split_profile_frame


def world_frame():
    frame = np.full((1200, 4000, 3), 100, np.uint8)
    frame[:, :160] = 0
    frame[:, 160:2080] = 50
    frame[:, 2080:] = 100
    return frame


def test_three_matching_conditions_assign_world_intelligent():
    profile = detect_camera_profile(
        "DECXIN Camera: DECXIN Camera",
        world_frame(),
        CameraMode(4000, 1200, 30, "MJPG"),
    )
    assert profile.label == "world intelligent"
    assert profile.code_band_status == "通过（160 px）"
    assert profile.per_eye_size == (1920, 1200)


@pytest.mark.parametrize(
    ("name", "shape"),
    [("Other Camera", (1200, 4000)), ("DECXIN Camera", (1080, 3840))],
)
def test_name_or_resolution_mismatch_does_not_assign_world_label(name, shape):
    frame = np.full((*shape, 3), 100, np.uint8)
    mode = CameraMode(shape[1], shape[0], 30, "MJPG")
    assert detect_camera_profile(name, frame, mode).label == "generic stereo"


def test_matching_name_and_size_reject_missing_code_band():
    frame = np.full((1200, 4000, 3), 100, np.uint8)
    with pytest.raises(RuntimeError, match="码带识别失败"):
        detect_camera_profile("DECXIN Camera", frame, CameraMode(4000, 1200, 30, "MJPG"))


def test_world_split_removes_code_band_and_preserves_exact_pixels():
    frame = world_frame()
    profile = detect_camera_profile(
        "DECXIN Camera", frame, CameraMode(4000, 1200, 30, "MJPG")
    )
    left, right = split_profile_frame(frame, profile)
    assert left.shape == right.shape == (1200, 1920, 3)
    assert np.all(left == 50)
    assert np.all(right == 100)


def test_generic_split_and_swap_remain_compatible():
    frame = np.zeros((1080, 3840, 3), np.uint8)
    frame[:, :1920] = 1
    frame[:, 1920:] = 2
    profile = detect_camera_profile(
        "UVC Camera 1", frame, CameraMode(3840, 1080, 30, "MJPG")
    )
    left, right = split_profile_frame(frame, profile, swap_eyes=True)
    assert left.shape == right.shape == (1080, 1920, 3)
    assert np.all(left == 2)
    assert np.all(right == 1)
```

- [ ] **Step 2: Run the tests and verify RED**

Run: `.venv/bin/pytest tests/test_camera_profile.py -q`  
Expected: FAIL because `stereo_calibrator.camera_profile` does not exist.

- [ ] **Step 3: Implement the minimal profile module**

```python
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_backend import CameraMode
from .sbs import split_sbs


@dataclass(frozen=True)
class CameraProfile:
    label: str
    mode: CameraMode
    per_eye_size: tuple[int, int]
    code_band_status: str
    split_kind: str


def has_world_intelligent_code_band(frame: np.ndarray) -> bool:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    band_dark = float(np.mean(gray[:, :160] < 12))
    adjacent_dark = float(np.mean(gray[:, 160:320] < 12))
    return band_dark >= 0.35 and band_dark - adjacent_dark >= 0.25


def detect_camera_profile(device_name: str, frame: np.ndarray, mode: CameraMode) -> CameraProfile:
    is_name = "decxin camera" in device_name.casefold()
    is_size = frame.shape[:2] == (1200, 4000) and (mode.width, mode.height) == (4000, 1200)
    if is_name and is_size:
        if not has_world_intelligent_code_band(frame):
            raise RuntimeError("world intelligent 码带识别失败")
        return CameraProfile("world intelligent", mode, (1920, 1200), "通过（160 px）", "world")
    return CameraProfile("generic stereo", mode, (mode.width // 2, mode.height), "不适用", "equal")


def split_profile_frame(frame, profile, swap_eyes=False):
    if profile.split_kind == "world":
        left, right = frame[:, 160:2080].copy(), frame[:, 2080:4000].copy()
        if left.shape[:2] != (1200, 1920) or right.shape[:2] != (1200, 1920):
            raise RuntimeError("world intelligent 裁剪尺寸错误")
        return (right, left) if swap_eyes else (left, right)
    return split_sbs(frame, swap_eyes)
```

- [ ] **Step 4: Run profile and legacy SBS tests**

Run: `.venv/bin/pytest tests/test_camera_profile.py tests/test_sbs_detector.py -q`  
Expected: all tests pass.

- [ ] **Step 5: Commit the profile unit**

```bash
git add src/stereo_calibrator/camera_profile.py tests/test_camera_profile.py
git commit -m "feat: identify world intelligent camera frames"
```

### Task 2: Dynamic V4L2 Mode Probe and Headless Integration

**Files:**
- Modify: `src/stereo_calibrator/camera_backend.py`
- Modify: `tests/test_camera_backend.py`
- Modify: `src/stereo_calibrator/headless.py`
- Modify: `tests/test_headless.py`

**Interfaces:**
- Consumes: `CameraProfile`, `detect_camera_profile`, and `split_profile_frame` from Task 1.
- Produces: `linux_camera_name(device: str) -> str`, `open_first_supported_linux_camera(device, modes) -> tuple[cv2.VideoCapture, CameraMode]`, and resolved headless status keys `camera_label`, `code_band`, `mode`, and `per_eye`.

- [ ] **Step 1: Write failing backend tests**

```python
def test_mode_probe_returns_first_exact_supported_mode(monkeypatch):
    calls = []
    expected = CameraMode(3840, 1080, 30, "MJPG")
    fake = FakeCapture(3840, 1080, 30, "MJPG")

    def fake_open(_device, mode):
        calls.append(mode)
        if mode.width == 4000:
            raise RuntimeError("unsupported")
        return fake

    monkeypatch.setattr(camera_backend, "open_linux_camera", fake_open)
    capture, mode = camera_backend.open_first_supported_linux_camera(
        "/dev/video0",
        [CameraMode(4000, 1200, 30, "MJPG"), expected],
    )
    assert capture is fake
    assert mode == expected
    assert [item.width for item in calls] == [4000, 3840]


def test_linux_camera_name_reads_video_node_name(monkeypatch, tmp_path):
    name_file = tmp_path / "name"
    name_file.write_text("DECXIN Camera: DECXIN Camera\n")
    monkeypatch.setattr(camera_backend, "_video_name_path", lambda _device: name_file)
    assert camera_backend.linux_camera_name("/dev/video0") == "DECXIN Camera: DECXIN Camera"
```

- [ ] **Step 2: Run backend tests and verify RED**

Run: `.venv/bin/pytest tests/test_camera_backend.py -q`  
Expected: FAIL because the two new backend functions do not exist.

- [ ] **Step 3: Implement device-name lookup and ordered probing**

Add `linux_camera_name()` using `/sys/class/video4linux/<device basename>/name`. Add `open_first_supported_linux_camera()` that calls `open_linux_camera()` in order, returns the first exact match and selected `CameraMode`, and raises one combined error listing each rejected mode if none opens.

- [ ] **Step 4: Run backend tests and verify GREEN**

Run: `.venv/bin/pytest tests/test_camera_backend.py -q`  
Expected: all tests pass.

- [ ] **Step 5: Write failing headless integration tests**

```python
class WorldCamera(FakeCamera):
    def read(self):
        frame = np.full((1200, 4000, 3), 100, np.uint8)
        frame[:, :160] = 0
        return True, frame


def test_world_camera_resolves_status_and_cropped_preview(tmp_path, monkeypatch):
    engine = HeadlessCalibrationEngine(
        make_config(), tmp_path, 0.020, "/dev/video0",
        camera=WorldCamera(), device_name="DECXIN Camera"
    )
    monkeypatch.setattr(headless, "detect_chessboard", lambda *_args: None)
    engine.start()
    deadline = time.monotonic() + 2
    while engine.status_snapshot()["camera_label"] == "detecting" and time.monotonic() < deadline:
        time.sleep(0.01)
    engine.action("stop")
    engine.join(2)
    status = engine.status_snapshot()
    assert status["camera_label"] == "world intelligent"
    assert status["mode"] == "MJPG 4000x1200@30"
    assert status["per_eye"] == "1920x1200"
    assert status["code_band"] == "通过（160 px）"


def test_matching_world_camera_without_band_enters_error(tmp_path):
    camera = WorldCamera()
    camera.read = lambda: (True, np.full((1200, 4000, 3), 100, np.uint8))
    engine = HeadlessCalibrationEngine(
        make_config(), tmp_path, 0.020, "/dev/video0",
        camera=camera, device_name="DECXIN Camera"
    )
    engine.start()
    engine.join(2)
    assert engine.status_snapshot()["state"] == "error"
    assert "码带识别失败" in engine.status_snapshot()["error"]
```

- [ ] **Step 6: Run headless tests and verify RED**

Run: `.venv/bin/pytest tests/test_headless.py -q`  
Expected: FAIL because the constructor and status do not support profile resolution.

- [ ] **Step 7: Integrate profile resolution into the engine**

Define:

```python
SUPPORTED_RK3588_MODES = (
    CameraMode(4000, 1200, 30.0, "MJPG"),
    CameraMode(3840, 1080, 30.0, "MJPG"),
)
```

On `start()`, read the V4L2 name and probe supported modes when the camera is not injected. In `_run()`, classify the first frame once, update status from the returned profile, validate later frames against `profile.mode`, and replace `split_sbs()` with `split_profile_frame()`. Initialize status as `camera_label="detecting"`, `mode="探测中"`, `per_eye="探测中"`, and `code_band="探测中"`.

- [ ] **Step 8: Run headless and related tests**

Run: `.venv/bin/pytest tests/test_headless.py tests/test_camera_backend.py tests/test_camera_profile.py tests/test_sbs_detector.py -q`  
Expected: all tests pass.

- [ ] **Step 9: Commit backend and engine integration**

```bash
git add src/stereo_calibrator/camera_backend.py src/stereo_calibrator/headless.py tests/test_camera_backend.py tests/test_headless.py
git commit -m "feat: resolve camera profiles on rk3588"
```

### Task 3: Web Status, Documentation, and RK3588 Verification

**Files:**
- Modify: `src/stereo_calibrator/web.py`
- Modify: `tests/test_web.py`
- Modify: `src/stereo_calibrator/cli.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: headless status keys from Task 2.
- Produces: visible camera label and coded-band state in the existing web page; CLI startup text that reports automatic mode detection.

- [ ] **Step 1: Write failing web and CLI assertions**

Add `camera_label="world intelligent"` and `code_band="通过（160 px）"` to `FakeEngine.status_snapshot()`, then assert:

```python
assert 'id="cameraLabel"' in html
assert 'id="codeBand"' in html
assert "s.camera_label" in html
assert "s.code_band" in html
```

Update the CLI test to expect `RK3588 模式=自动探测` for `--web --dry-run`.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `.venv/bin/pytest tests/test_web.py tests/test_cli.py -q`  
Expected: FAIL because the page and CLI do not expose profile detection.

- [ ] **Step 3: Add the two status cards and dynamic bindings**

Add cards labeled `设备标签` and `码带检测`, bind them to `cameraLabel` and `codeBand`, and update them from `s.camera_label` and `s.code_band`. Replace the hard-coded CLI mode line with `RK3588 模式=自动探测 4000x1200/3840x1080 MJPG@30`.

- [ ] **Step 4: Document the new profile**

Add a concise README section containing the three conditions, label, exact crop ranges, two supported modes, and the requirement that runtime uses the same crop as calibration.

- [ ] **Step 5: Run the complete local verification**

Run: `.venv/bin/pytest -q`  
Run: `.venv/bin/python -m compileall -q src`  
Run: `git diff --check`  
Expected: all tests pass, compilation succeeds, and no whitespace errors are reported.

- [ ] **Step 6: Commit the UI and documentation**

```bash
git add src/stereo_calibrator/web.py src/stereo_calibrator/cli.py tests/test_web.py tests/test_cli.py README.md
git commit -m "feat: show world intelligent camera status"
```

- [ ] **Step 7: Synchronize the committed platform to RK3588**

Run from the repository root:

```bash
rsync -az --exclude sessions --exclude backups --exclude .venv --exclude diagnostics ./ root@192.168.100.200:/root/stereo_chessboard_calibrator/
```

Expected: source, tests, config, and documentation update while existing session material remains untouched.

- [ ] **Step 8: Run RK3588 automated and live verification**

Run:

```bash
ssh root@192.168.100.200 'cd /root/stereo_chessboard_calibrator && .venv/bin/pytest -q && .venv/bin/python -m compileall -q src'
ssh root@192.168.100.200 'cd /root/stereo_chessboard_calibrator && timeout 12 ./calibrate --web --device /dev/video0 --square-mm 20 --target 32 --host 127.0.0.1 --port 18766'
```

Expected status evidence before timeout:

```text
camera_label = world intelligent
mode = MJPG 4000x1200@30
per_eye = 1920x1200
code_band = 通过（160 px）
```

Capture one live frame through the engine and run `detect_chessboard(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), (8, 5))` and `detect_chessboard(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), (8, 5))`. Expected: both returned arrays contain `40` corners on the currently positioned board.

- [ ] **Step 9: Final repository verification**

Run: `git status --short`  
Expected: only the pre-existing untracked `diagnostics/` directory remains; all implementation files are committed.
