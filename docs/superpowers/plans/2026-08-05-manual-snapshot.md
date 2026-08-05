# Manual Snapshot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Web button that queues and saves the next full-resolution SBS frame and split eye images without requiring chessboard detection.

**Architecture:** `HeadlessCalibrationEngine.action("manual_capture")` increments a thread-safe request counter. The camera worker consumes at most one request per frame, writes three images under `session/manual/`, and exposes `manual_pairs` through the existing status API; the Web UI renders the count and sends the action.

**Tech Stack:** Python 3, OpenCV, standard-library HTTP server, pytest, Git/SSH.

## Global Constraints

- Manual images never enter `_accepted` and never count as solver-ready samples.
- Each request saves full SBS JPEG plus left and right PNG using a zero-padded increasing index.
- Existing automatic capture and solve behavior remains unchanged.
- Work directly in the existing empty-project repository; do not create a worktree.

---

### Task 1: Engine Manual Snapshot Queue

**Files:**
- Modify: `tests/test_headless.py`
- Modify: `src/stereo_calibrator/headless.py`

**Interfaces:**
- Consumes: `HeadlessCalibrationEngine.action(name: str) -> Dict[str, object]`, the current worker frame, and `split_sbs` output.
- Produces: `manual_capture` action, `manual_pairs: int` status field, and `manual/{index:04d}_{sbs.jpg,left.png,right.png}` files.

- [ ] **Step 1: Write the failing engine test**

Add a test that starts the engine with `FakeCamera`, calls `manual_capture`, waits for `manual_pairs == 1`, stops the worker, and asserts all three `0000` files exist while `accepted_pairs == 0`.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `.venv/bin/python -m pytest -q tests/test_headless.py -k manual_capture`

Expected: FAIL because `manual_capture` is unsupported and `manual_pairs` is absent.

- [ ] **Step 3: Implement the minimal engine behavior**

Initialize `_manual_requests = 0` and status `manual_pairs = 0`; accept `manual_capture` only while capturing, increment the request under `_lock`, and return `{"ok": True}`. After splitting each worker frame, atomically consume one request and call `_save_manual_snapshot(frame, left, right)`. That method creates `session/manual/`, writes `0000_sbs.jpg`, `0000_left.png`, and `0000_right.png`, raises `RuntimeError("写入手动采集图像失败")` on any failure, then increments status `manual_pairs` and reports `已手动保存第 N 对素材`.

- [ ] **Step 4: Run the focused test and full engine tests**

Run: `.venv/bin/python -m pytest -q tests/test_headless.py`

Expected: all tests PASS.

- [ ] **Step 5: Commit engine behavior**

Run: `git add tests/test_headless.py src/stereo_calibrator/headless.py && git commit -m "feat: capture manual stereo snapshots"`

### Task 2: Web Manual Capture Control

**Files:**
- Modify: `tests/test_web.py`
- Modify: `src/stereo_calibrator/web.py`

**Interfaces:**
- Consumes: status key `manual_pairs` and action `manual_capture` from Task 1.
- Produces: “手动素材” card with `id="manualCount"` and “手动拍摄” button calling `act('manual_capture')`.

- [ ] **Step 1: Write the failing Web tests**

Extend `FakeEngine` with `manual_pairs: 0` and `manual_capture`; assert the home page contains `手动拍摄`, `act('manual_capture')`, and `manualCount`; POST the action and assert it reaches the engine.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `.venv/bin/python -m pytest -q tests/test_web.py -k 'home_page or manual_capture'`

Expected: FAIL because the manual UI and action are missing.

- [ ] **Step 3: Implement the minimal Web UI**

Add the status card and button, and set `manualCount.textContent = `${s.manual_pairs} / ${s.target_pairs}`` inside `refresh()`.

- [ ] **Step 4: Run full local verification**

Run: `.venv/bin/python -m pytest -q && git diff --check`

Expected: all tests PASS and no whitespace errors.

- [ ] **Step 5: Commit Web behavior**

Run: `git add tests/test_web.py src/stereo_calibrator/web.py && git commit -m "feat: expose manual capture in web ui"`

### Task 3: RK3588 Deployment and Real Capture Verification

**Files:**
- Runtime output: `/root/stereo_chessboard_calibrator/sessions/<session>/manual/`

**Interfaces:**
- Consumes: Git `main` and the existing `/dev/video0` Web service.
- Produces: restarted service on RK3588 and one verified real manual snapshot set.

- [ ] **Step 1: Push and update RK3588**

Run: `git push rk3588 main`, then SSH to pull `main` in `/root/stereo_chessboard_calibrator`.

- [ ] **Step 2: Run remote tests**

Run remotely: `.venv/bin/python -m pytest -q`

Expected: all tests PASS.

- [ ] **Step 3: Restart the service and preserve the SSH tunnel**

Stop only the prior calibration process, start `./calibrate --web --device /dev/video0 --host 0.0.0.0 --port 8765 --square-mm 20`, and retain local forward `127.0.0.1:18765 -> RK3588:8765`.

- [ ] **Step 4: Trigger one real manual capture and verify artifacts**

POST `{"action":"manual_capture"}` through `http://127.0.0.1:18765/api/action`, poll until `manual_pairs` increases, and verify the three non-empty files and their dimensions on RK3588.

- [ ] **Step 5: Verify the live browser page**

Reload `http://127.0.0.1:18765/` and confirm the manual count and button are visible while automatic `accepted_pairs` remains unchanged.
