# Manual Guidance and Backup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive a fixed 32-shot manual stereo capture sequence, prevent extra shots, simplify the Web progress display, and copy the previous session into `backups/` before each new Web session.

**Architecture:** A pure `manual_guidance(completed, target)` function maps the completed shot count to the next requested pose. The headless engine uses this mapping and caps `manual_capture`; CLI startup calls a focused backup helper before creating a new timestamped Web session.

**Tech Stack:** Python 3, OpenCV, shutil, standard-library HTTP server, pytest, Git/SSH.

## Global Constraints

- Guidance depends only on successful manual shot count, not automatic corner detection.
- Shot 32 completes the session and shot 33 is rejected.
- Backup is a complete copy; the source session is retained and an existing backup is never overwritten.
- The Web page shows one capture progress card and no “手动素材” card.
- Work directly on `main` with no worktree, as explicitly requested.

---

### Task 1: Count-Driven Manual Guidance

**Files:**
- Modify: `tests/test_headless.py`
- Modify: `src/stereo_calibrator/headless.py`

**Interfaces:**
- Produces: `manual_guidance(completed: int, target: int = 32) -> str`.
- Updates: `HeadlessCalibrationEngine.action("manual_capture")` rejects when `manual_pairs >= target_pairs`.

- [ ] **Step 1: Write failing tests**

Add literal assertions for counts 0, 2, 10, 30, and 32, plus an engine test that sets `manual_pairs` to the target and expects `{"ok": False, "error": "本轮 32 组已采集完成"}`.

- [ ] **Step 2: Verify RED**

Run: `.venv/bin/python -m pytest -q tests/test_headless.py -k 'manual_guidance or rejects_after_target'`

Expected: FAIL because the mapping and cap do not exist.

- [ ] **Step 3: Implement minimal mapping and cap**

Implement the exact 32-shot ranges from the approved design. Initialize guidance with `manual_guidance(0, target)`; after each successful save, update `manual_pairs`, guidance, and completion reason. Stop the automatic `guidance_hint(history)` update from overwriting manual guidance.

- [ ] **Step 4: Verify GREEN**

Run: `.venv/bin/python -m pytest -q tests/test_headless.py`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

Run: `git add tests/test_headless.py src/stereo_calibrator/headless.py && git commit -m "feat: guide manual capture poses"`

### Task 2: Unified Web Progress

**Files:**
- Modify: `tests/test_web.py`
- Modify: `src/stereo_calibrator/web.py`

**Interfaces:**
- Consumes: `manual_pairs`, `target_pairs`, and `guidance` from engine status.
- Produces: one `count` card showing `${s.manual_pairs} / ${s.target_pairs}`.

- [ ] **Step 1: Write failing Web assertions**

Assert the page contains `采集进度`, does not contain `手动素材` or `manualCount`, and still contains the manual capture action.

- [ ] **Step 2: Verify RED**

Run: `.venv/bin/python -m pytest -q tests/test_web.py -k home_page`

Expected: FAIL because the separate manual card remains.

- [ ] **Step 3: Implement minimal HTML/JavaScript change**

Remove the manual card and assign `count.textContent` from `s.manual_pairs` instead of `s.accepted_pairs`.

- [ ] **Step 4: Verify GREEN and commit**

Run: `.venv/bin/python -m pytest -q tests/test_web.py`

Then commit `tests/test_web.py` and `src/stereo_calibrator/web.py` as `feat: show manual capture as primary progress`.

### Task 3: Previous Session Backup

**Files:**
- Modify: `tests/test_cli.py`
- Modify: `src/stereo_calibrator/cli.py`

**Interfaces:**
- Produces: `backup_previous_web_session(session_root: Path, backup_root: Path) -> Optional[Path]`.
- CLI default destination: `PROJECT_ROOT / "backups"`.

- [ ] **Step 1: Write failing backup tests**

Create two timestamp session fixtures, put a manual file in the newest, call the helper, and assert the newest session is copied to `backups/<timestamp>`, the source remains, and a pre-existing destination sentinel is not overwritten.

- [ ] **Step 2: Verify RED**

Run: `.venv/bin/python -m pytest -q tests/test_cli.py -k backup_previous`

Expected: FAIL because the helper does not exist.

- [ ] **Step 3: Implement minimal backup helper and startup call**

Filter direct children of `session_root` to timestamp directories containing `manual/`, select the lexicographically latest, skip when `backup_root / name` exists, otherwise use `shutil.copytree`. Call it only for non-dry-run Web startup, before creating the new session directory.

- [ ] **Step 4: Run complete local verification and commit**

Run: `.venv/bin/python -m pytest -q && git diff --check`

Expected: all tests PASS and no whitespace errors. Commit as `feat: back up previous web session`.

### Task 4: RK3588 Deployment

**Files:**
- Runtime backup: `/root/stereo_chessboard_calibrator/backups/20260805_162835/`

**Interfaces:**
- Consumes: committed `main`, `/dev/video0`, and existing local SSH forwarding.
- Produces: new active session at `0/32` and a verified copy of the prior 56-shot session.

- [ ] **Step 1: Push, pull, and run remote tests**

Push `main`; on RK run `PYTHONPATH=.remote-deps:src python3 -m pytest -q`.

- [ ] **Step 2: Restart and verify backup**

Start the Web service, then verify both source and backup contain 168 non-empty files and the API reports `manual_pairs: 0` with central-facing guidance.

- [ ] **Step 3: Verify browser UI**

Reload `http://127.0.0.1:18765/`; confirm one `0/32` progress card, no “手动素材” label, and the first central-facing instruction.
