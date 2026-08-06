# Hybrid Auto/Manual Calibration Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make low-light capture reliable with mandatory manual saves, five-frame sharpness selection, optional automatic pose-slot capture, separate saved/valid counts, and CLAHE retry during solving.

**Architecture:** All new manual and automatic captures are persisted in the existing `manual/` session directory with source metadata. Live detection drives automatic slot acceptance, while the solver reloads every persisted sample and applies raw-then-CLAHE corner detection before calibration.

**Tech Stack:** Python 3, OpenCV, NumPy, existing threaded HTTP server, pytest.

## Global Constraints

- Pattern is `8×5` inner corners.
- Manual capture always persists one sample and is never blocked by detection or quality gates.
- Manual capture selects the best of five consecutive SBS frames using the weaker eye's Laplacian sharpness.
- Automatic capture is enabled by default and can be disabled without affecting manual capture.
- Saved and detected-valid counts are distinct.
- Calibration requires at least 20 valid stereo pairs.

---

### Task 1: Five-frame mandatory manual save

**Files:**
- Modify: `tests/test_headless.py`
- Modify: `src/stereo_calibrator/headless.py`

**Interfaces:**
- Produces: `_sharpness_score(left, right) -> float`, a five-frame pending buffer, and `*_metadata.json` with `source`, `sharpness`, and detection state.

- [ ] Add a failing test that queues manual capture, supplies five frames with different sharpness, and asserts the highest weaker-eye score is persisted.
- [ ] Add a failing test that uses no detected corners and still asserts saved count and all three image files.
- [ ] Implement a five-frame buffer in `_run`; finalize exactly one capture after the fifth frame.
- [ ] Persist metadata only after all images write successfully and then increment `saved_pairs`/compatibility `manual_pairs`.
- [ ] Run `.venv/bin/pytest -q tests/test_headless.py`.

### Task 2: Automatic switch and pose slots

**Files:**
- Create: `src/stereo_calibrator/pose_slots.py`
- Create: `tests/test_pose_slots.py`
- Modify: `src/stereo_calibrator/headless.py`
- Modify: `src/stereo_calibrator/web.py`
- Modify: `tests/test_web.py`

**Interfaces:**
- Produces: `classify_pose_slot(corners, pattern, image_size, filled) -> str | None` and status fields `auto_capture_enabled`, `saved_pairs`, `detected_valid_pairs`, `pose_slots`.

- [ ] Test center, edge/corner, roll, perspective, near, and far classification with synthetic corner grids.
- [ ] Implement normalized center, area, roll, and opposing-edge ratios; choose only a slot below its quota.
- [ ] Add `auto_on` and `auto_off` actions; default to enabled.
- [ ] Route successful stable auto captures through the unified persisted sample path with `source="auto"` and the chosen slot.
- [ ] Add the auto toggle and separate saved/valid cards to the Web page and test the rendered controls/status bindings.
- [ ] Run focused pose, headless, and Web tests.

### Task 3: Unified raw/CLAHE solve filtering

**Files:**
- Modify: `src/stereo_calibrator/detector.py`
- Modify: `src/stereo_calibrator/headless.py`
- Modify: `tests/test_sbs_detector.py`
- Modify: `tests/test_headless.py`

**Interfaces:**
- Produces: `detect_chessboard_with_retry(gray, pattern) -> (corners, method)` where method is `raw`, `clahe`, or `none`.

- [ ] Test that raw success does not call enhanced detection and raw failure can succeed after CLAHE.
- [ ] Implement CLAHE retry with `clipLimit=2.0` and tile grid `8×8`.
- [ ] Load every persisted sample regardless of source, record rejected indices, and update detected-valid count before threshold evaluation.
- [ ] When fewer than 20 samples survive, return `retake` with invalid indices and missing pose-slot guidance.
- [ ] Run all tests.

### Task 4: Deploy and verify

**Files:**
- Modify on RK3588: matching files under `/root/stereo_chessboard_calibrator`.

**Interfaces:**
- Consumes: `deploy/start.sh` and the RK3588 camera.
- Produces: live service at `http://127.0.0.1:18765/`.

- [ ] Run `.venv/bin/pytest -q`, `git diff --check`, and `sh -n deploy/start.sh`.
- [ ] Sync changed source/config/test files to RK3588 and run its full test suite.
- [ ] Restart the Web service while preserving the current session directory.
- [ ] Verify API fields, manual five-frame save, auto toggle, and absence of the stop button.
- [ ] Commit and push `main`.

