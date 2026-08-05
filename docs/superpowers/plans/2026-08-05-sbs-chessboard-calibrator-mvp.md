# SBS Chessboard Calibrator MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a minimal end-to-end macOS tool that finds `UVC Camera 1`, captures high-resolution SBS chessboard pairs with automatic quality/novelty gating, solves stereo calibration, validates it, and exports OpenCV/Python/C++ artifacts.

**Architecture:** A single `./calibrate` entry orchestrates focused Python modules for SBS splitting, chessboard detection, quality gating, guided capture, calibration, and export. Runtime data is stored under timestamped sessions; all numerical work uses unscaled per-eye images while the UI uses resized previews.

**Tech Stack:** Python 3.11+, OpenCV 4.8+, NumPy 1.24+, PyYAML 6+, pytest 8+, macOS AVFoundation.

## Global Constraints

- Build only inside the new `stereo_chessboard_calibrator/` directory; do not import or copy from `stereo_calib_kit/`.
- Camera input is one SBS device named exactly `UVC Camera 1`; the default split is left half then right half.
- Detect and save at the highest successfully opened native resolution without resizing source images.
- Chessboard pattern is 9×6 inner corners; square size is entered in millimetres and converted to metres.
- Default target is 32 accepted pairs, minimum solve count is 20, and hard capture maximum is 40.
- MVP exports YAML, JSON, NPZ, compressed rectification maps, a rectification preview, and a C++ loader example.
- MVP omits HTML report, full session resume UI, fisheye solving, ROS/Kalibr, and two-device stereo.
- The parent workspace is not a Git repository, so commit steps are replaced by verification checkpoints.

---

## File Map

- `calibrate`: executable wrapper; launches the package from the repository without installation.
- `pyproject.toml`: runtime and test dependencies.
- `configs/default.yaml`: pattern, capture, quality, and validation defaults.
- `src/stereo_calibrator/models.py`: typed dataclasses shared between modules.
- `src/stereo_calibrator/sbs.py`: validates and splits SBS frames.
- `src/stereo_calibrator/detector.py`: chessboard detection and pose features.
- `src/stereo_calibrator/quality.py`: sharpness, exposure, stability, and novelty decisions.
- `src/stereo_calibrator/capture.py`: AVFoundation camera open, live UI, automatic acceptance, and session image writing.
- `src/stereo_calibrator/solver.py`: calibration, robust pair filtering, stereo rectification, and metrics.
- `src/stereo_calibrator/exporter.py`: stable YAML/JSON/NPZ/maps and preview writing.
- `src/stereo_calibrator/cli.py`: CLI arguments, prompts, session creation, pipeline orchestration.
- `boards/chessboard_A4_9x6_20mm.svg`: printable 1:1 A4 board with scale bar.
- `examples/cpp/load_calibration.cpp`: YAML and map loading with `cv::remap`.
- `examples/cpp/CMakeLists.txt`: C++ example build.
- `tests/`: offline unit and synthetic integration tests.

### Task 1: Project Skeleton, SBS Split, and Chessboard Detection

**Files:**
- Create: `calibrate`
- Create: `pyproject.toml`
- Create: `configs/default.yaml`
- Create: `src/stereo_calibrator/__init__.py`
- Create: `src/stereo_calibrator/models.py`
- Create: `src/stereo_calibrator/sbs.py`
- Create: `src/stereo_calibrator/detector.py`
- Create: `tests/test_sbs_detector.py`

**Interfaces:**
- Produces: `split_sbs(frame: np.ndarray, swap_eyes: bool = False) -> tuple[np.ndarray, np.ndarray]`.
- Produces: `detect_chessboard(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None` with shape `(N, 2)` and `float32` values.
- Produces: `pose_features(corners: np.ndarray, image_size: tuple[int, int]) -> PoseFeatures`.
- Produces: dataclasses `PoseFeatures`, `AcceptedPair`, and `CalibrationResult` for later tasks.

- [ ] **Step 1: Write failing split and detector tests**

```python
def test_split_sbs_and_swap():
    frame = np.zeros((4, 8, 3), np.uint8)
    frame[:, :4] = 10
    frame[:, 4:] = 20
    left, right = split_sbs(frame)
    assert left.shape == right.shape == (4, 4, 3)
    assert int(left.mean()) == 10 and int(right.mean()) == 20
    swapped_left, _ = split_sbs(frame, swap_eyes=True)
    assert int(swapped_left.mean()) == 20

def test_split_sbs_rejects_odd_width():
    with pytest.raises(ValueError, match="even"):
        split_sbs(np.zeros((4, 7, 3), np.uint8))

def test_pose_features_are_normalized():
    corners = np.array([[10, 10], [30, 10], [10, 30], [30, 30]], np.float32)
    feature = pose_features(corners, (100, 50))
    assert feature.center_x == pytest.approx(0.2)
    assert feature.center_y == pytest.approx(0.4)
    assert 0 < feature.area_ratio < 1
```

- [ ] **Step 2: Run tests and confirm missing-module failure**

Run: `python3 -m pytest tests/test_sbs_detector.py -q`  
Expected: import failure for `stereo_calibrator`.

- [ ] **Step 3: Implement dataclasses, strict SBS splitting, `findChessboardCornersSB` detection, and normalized feature extraction**

Use `cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)` and return `None` on failure. Compute center and bounding-box area normalized by image dimensions; estimate simple horizontal and vertical perspective skew from the four extreme chessboard corners.

- [ ] **Step 4: Add configuration and executable wrapper**

`default.yaml` must define device name, 9×6 pattern, target/min/max counts, 0.8 second stability, and validation thresholds. `calibrate` must prepend `src/` to `PYTHONPATH` and run `python3 -m stereo_calibrator.cli "$@"`.

- [ ] **Step 5: Run Task 1 tests**

Run: `python3 -m pytest tests/test_sbs_detector.py -q`  
Expected: all tests pass.

### Task 2: Automatic Quality Gate and Guided Capture

**Files:**
- Create: `src/stereo_calibrator/quality.py`
- Create: `src/stereo_calibrator/capture.py`
- Create: `tests/test_quality.py`

**Interfaces:**
- Consumes: `PoseFeatures`, `detect_chessboard`, and `split_sbs` from Task 1.
- Produces: `evaluate_pair(left_gray, right_gray, left_corners, right_corners, history, thresholds) -> QualityDecision`.
- Produces: `capture_session(config: dict, session_dir: Path, square_size_m: float, swap_eyes: bool) -> list[AcceptedPair]`.

- [ ] **Step 1: Write failing quality tests**

```python
def test_blurry_pair_is_rejected():
    gray = np.full((120, 160), 127, np.uint8)
    corners = regular_corners()
    decision = evaluate_pair(gray, gray, corners, corners, [], test_thresholds())
    assert not decision.accepted
    assert decision.reason == "图像模糊"

def test_duplicate_pose_is_rejected():
    gray = textured_image()
    corners = regular_corners()
    history = [pose_features(corners, (160, 120))]
    decision = evaluate_pair(gray, gray, corners, corners, history, test_thresholds())
    assert not decision.accepted
    assert "重复" in decision.reason
```

- [ ] **Step 2: Run tests and confirm missing-function failure**

Run: `python3 -m pytest tests/test_quality.py -q`  
Expected: import failure for `evaluate_pair`.

- [ ] **Step 3: Implement quality metrics and a single rejection reason**

Use Laplacian variance for sharpness, clipped-pixel ratios for exposure, corner bounding-box margins for completeness, recent centroid motion for stability, and weighted feature distance for novelty. Return the first failure in this order: missing corners, edge margin, exposure, blur, motion, duplicate.

- [ ] **Step 4: Implement live capture loop**

Open the camera with `cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)`. Probe indices 0–9 by reading one frame and match the device index supplied by FFmpeg name enumeration; request candidate widths/heights from the highest advertised mode. Show side-by-side resized previews with corners, acceptance count, primary rejection reason, stability progress, and a next-position hint derived from a 3×3 center grid. Save raw SBS and lossless left/right PNG only after the quality decision remains acceptable for 0.8 seconds. Handle `P`, `U`, and `Q`.

- [ ] **Step 5: Run quality tests and syntax checks**

Run: `python3 -m pytest tests/test_quality.py -q`  
Run: `python3 -m compileall -q src`  
Expected: both commands succeed.

### Task 3: Stereo Solver, Validation, and Artifact Export

**Files:**
- Create: `src/stereo_calibrator/solver.py`
- Create: `src/stereo_calibrator/exporter.py`
- Create: `tests/test_solver_exporter.py`

**Interfaces:**
- Consumes: accepted left/right PNG paths and corner metadata from Tasks 1–2.
- Produces: `solve_stereo(pairs: list[AcceptedPair], pattern: tuple[int, int], square_size_m: float, thresholds: dict) -> CalibrationResult`.
- Produces: `export_result(result: CalibrationResult, output_dir: Path, sample_pair: AcceptedPair) -> None`.

- [ ] **Step 1: Write failing synthetic solver and export tests**

```python
def test_solver_recovers_finite_stereo_matrices(synthetic_pairs):
    result = solve_stereo(synthetic_pairs, (9, 6), 0.020, permissive_thresholds())
    assert result.left_camera_matrix.shape == (3, 3)
    assert result.translation.shape in {(3,), (3, 1)}
    assert np.isfinite(result.translation).all()
    assert result.valid_pair_count >= 20

def test_exports_have_stable_files(tmp_path, solved_result, sample_pair):
    export_result(solved_result, tmp_path, sample_pair)
    assert (tmp_path / "stereo_calibration.yaml").exists()
    assert (tmp_path / "stereo_calibration.json").exists()
    assert (tmp_path / "stereo_calibration.npz").exists()
    assert (tmp_path / "rectify_maps.yml.gz").exists()
    assert (tmp_path / "rectify_maps.npz").exists()
```

- [ ] **Step 2: Run tests and confirm missing-function failure**

Run: `python3 -m pytest tests/test_solver_exporter.py -q`  
Expected: import failure for `solve_stereo`.

- [ ] **Step 3: Implement solve and bounded outlier removal**

Run `cv2.calibrateCamera` for each eye, then `cv2.stereoCalibrate` with `cv2.CALIB_FIX_INTRINSIC`, followed by `cv2.stereoRectify`. Compute per-pair reprojection and rectified vertical disparity. Remove only the worst robust outlier per iteration, never more than 20% and never below 20 pairs. Mark `passed` only when RMS and epipolar median/P95 thresholds pass and all matrices are finite.

- [ ] **Step 4: Implement deterministic exporters**

Write OpenCV matrices with `cv2.FileStorage` to YAML and compressed maps. Write JSON matrices as `{rows, cols, data}`. Write NPZ with stable lowercase keys. Generate a side-by-side rectified preview with horizontal lines every 40 preview pixels.

- [ ] **Step 5: Run solver/export tests**

Run: `python3 -m pytest tests/test_solver_exporter.py -q`  
Expected: all tests pass.

### Task 4: CLI Integration, Printable Board, and C++ Consumer

**Files:**
- Create: `src/stereo_calibrator/cli.py`
- Create: `boards/chessboard_A4_9x6_20mm.svg`
- Create: `boards/board.yaml`
- Create: `examples/cpp/load_calibration.cpp`
- Create: `examples/cpp/CMakeLists.txt`
- Create: `README.md`
- Create: `tests/test_cli.py`

**Interfaces:**
- Consumes: `capture_session`, `solve_stereo`, and `export_result`.
- Produces: `main(argv: list[str] | None = None) -> int` and the user-facing `./calibrate` workflow.

- [ ] **Step 1: Write failing CLI smoke tests**

```python
def test_cli_help(capsys):
    assert main(["--help"]) == 0
    assert "UVC Camera 1" in capsys.readouterr().out

def test_cli_rejects_nonpositive_square_size(capsys):
    assert main(["--square-mm", "0", "--dry-run"]) == 2
    assert "方格边长" in capsys.readouterr().err
```

- [ ] **Step 2: Run tests and confirm missing-function failure**

Run: `python3 -m pytest tests/test_cli.py -q`  
Expected: import failure for `main`.

- [ ] **Step 3: Implement minimal orchestration and explicit exit codes**

Support `--device-name`, `--device-index` override, `--square-mm`, `--swap-eyes`, `--target`, `--session-root`, and `--dry-run`. Exit 0 on PASS/help, 1 on RETAKE/runtime failure, and 2 on invalid arguments. Create `sessions/YYYYMMDD_HHMMSS/`, save resolved config, call capture, solve, and export only when at least 20 pairs exist.

- [ ] **Step 4: Add printable SVG and C++ loading example**

SVG must declare A4 landscape physical dimensions in millimetres, draw 10×7 alternating 20 mm squares, add a white outer margin, a 100 mm scale bar, and Chinese/English 100% printing instruction. The C++ example must validate the stored image size, load calibration and maps using `cv::FileStorage`, read one SBS image, split it, remap both eyes, and write a rectified preview.

- [ ] **Step 5: Document the five-command quick path**

README quick path: create virtual environment, install editable package, print board at 100%, run `./calibrate --square-mm <measured>`, and build/run the C++ example. Include macOS Camera permission troubleshooting and explain that calibration resolution must match runtime resolution.

- [ ] **Step 6: Run full verification**

Run: `python3 -m pytest -q`  
Run: `python3 -m compileall -q src`  
Run: `./calibrate --help`  
Run: `./calibrate --square-mm 20 --dry-run`  
Expected: tests and compile pass; help exits 0; dry-run prints resolved device/pattern/session configuration without opening the camera.

## Deferred After MVP

- PDF generation in addition to the physically sized SVG.
- HTML quality report and thumbnail browser.
- Full session resume wizard and bounded rejected-frame gallery.
- Automated FFmpeg parsing of all AVFoundation frame-rate ranges; MVP allows a verified `--device-index` override if name-to-index enumeration differs across FFmpeg builds.
- Fisheye solving and automatic camera-model recommendation.

