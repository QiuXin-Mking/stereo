# RK3588 Service and Codex Tunnel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `deploy/start.sh` to start or reuse the RK3588 calibration service and expose it to the Codex browser at `127.0.0.1:18765`, while removing the unsafe stop button.

**Architecture:** The shell script performs remote and local health checks before mutating state. It starts the remote service only when port 8765 is unhealthy, then creates one SSH local-forward process and verifies the forwarded HTTP endpoint.

**Tech Stack:** POSIX shell, OpenSSH, curl, Python/pytest, existing Python HTTP service.

## Global Constraints

- Local URL is exactly `http://127.0.0.1:18765/`.
- Remote target is exactly `root@192.168.100.200`, port `8765`.
- A healthy existing remote service or local tunnel must be reused.
- Runtime PID and log files live under `deploy/.runtime/` and are ignored by Git.
- The Web UI must not render a stop-service button; the backend action remains available.

---

### Task 1: Remove the stop button

**Files:**
- Modify: `tests/test_web.py`
- Modify: `src/stereo_calibrator/web.py`

**Interfaces:**
- Consumes: existing `HTML_PAGE` string.
- Produces: a controls row without `act('stop')` or the text `停止服务`.

- [ ] **Step 1: Add the failing assertions**

```python
assert "停止服务" not in html
assert "act('stop')" not in html
```

- [ ] **Step 2: Run the focused test and confirm it fails**

Run: `.venv/bin/pytest -q tests/test_web.py::test_home_page_contains_preview_and_controls`
Expected: FAIL because the stop control is present.

- [ ] **Step 3: Remove only the stop button element from `HTML_PAGE`**

```html
<!-- Delete: <button class="danger" onclick="act('stop')">停止服务</button> -->
```

- [ ] **Step 4: Run the focused test and confirm it passes**

Run: `.venv/bin/pytest -q tests/test_web.py::test_home_page_contains_preview_and_controls`
Expected: PASS.

### Task 2: Add the idempotent deployment launcher

**Files:**
- Create: `deploy/start.sh`
- Create: `tests/test_deploy_start.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: key-based SSH access to `root@192.168.100.200` and the remote project at `/root/stereo_chessboard_calibrator`.
- Produces: a background SSH local forward on `127.0.0.1:18765` and a verified remote HTTP service on `127.0.0.1:8765`.

- [ ] **Step 1: Add structural tests for the launcher**

```python
def test_deploy_start_uses_safe_fixed_endpoints():
    text = START.read_text()
    assert "root@192.168.100.200" in text
    assert "127.0.0.1:18765:127.0.0.1:8765" in text
    assert "ExitOnForwardFailure=yes" in text

def test_deploy_start_checks_health_before_starting_components():
    text = START.read_text()
    assert "/api/status" in text
    assert "deploy/.runtime" not in text
```

- [ ] **Step 2: Run the focused test and confirm it fails**

Run: `.venv/bin/pytest -q tests/test_deploy_start.py`
Expected: FAIL because `deploy/start.sh` does not exist.

- [ ] **Step 3: Implement the launcher**

The script must use `set -eu`, derive the project directory from its own path, create `deploy/.runtime`, verify SSH with `BatchMode=yes`, start the existing remote command only when `/api/status` is unavailable, reuse a healthy local URL, reject an unrelated listener on port 18765, create `ssh -f -N -L 127.0.0.1:18765:127.0.0.1:8765`, save a discoverable tunnel PID, and poll the local status endpoint before printing the URL.

- [ ] **Step 4: Ignore runtime files and make the launcher executable**

```gitignore
deploy/.runtime/
```

Run: `chmod +x deploy/start.sh`

- [ ] **Step 5: Run all tests**

Run: `.venv/bin/pytest -q`
Expected: all tests pass.

- [ ] **Step 6: Run the launcher twice against RK3588**

Run: `./deploy/start.sh && ./deploy/start.sh`
Expected: both runs succeed, the second reports reuse, and `curl http://127.0.0.1:18765/api/status` returns JSON.

- [ ] **Step 7: Commit and push**

```bash
git add .gitignore deploy/start.sh src/stereo_calibrator/web.py tests/test_web.py tests/test_deploy_start.py
git commit -m "feat: add one-command RK3588 tunnel startup"
git push origin main
```

