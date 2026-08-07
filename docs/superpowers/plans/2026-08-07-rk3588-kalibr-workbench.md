# RK3588 OpenCV / Kalibr Workbench Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** 在现有 RK3588 双目网页中增加 Kalibr Camera+IMU 模式，由同一 UVC 帧循环保存双目图像并解码左侧码带 IMU，最终在 RK3588 ARM64 最小容器中独立完成双目和相机–IMU联合标定。

**Architecture:** HeadlessCalibrationEngine 继续独占 /dev/video0，并逐帧调用独立的 KalibrRecorder。RK 宿主服务负责采集、质量检查和作业状态；隔离容器只负责 ROS bag 与 Kalibr 离线求解；Mac 只通过现有 SSH 隧道访问网页和下载产物。

**Tech Stack:** Python 3.9、OpenCV、NumPy、PyYAML、http.server、Docker 20.10、Ubuntu 20.04 ARM64、ROS Noetic 最小运行组件、Kalibr、pytest。

## Global Constraints

- 相机优先模式固定为 MJPG 4000×1200@30；world intelligent 单眼 1920×1200；左侧码带 160px。
- OpenCV 模式的 32 槽位、手动一按必保存、5 帧择优、CLAHE 重试和导出不得回归。
- Kalibr 棋盘为 8×5 内角点，方格 0.020m。
- 码带固定为 IMU_VAL0=50、IMU_VAL1=220、IMU_USIZE=8、IMU_GROUP=16、IMU_TARGET=272，比特 LSB-first，整数大端。
- 单位转换固定为加速度 `raw * (4000/32768) * 9.80665 / 1000`，陀螺仪 `raw * (1000/32768) * pi / 180`。参考 C++ 的 gx_mdps 字段名是历史误命名，数值实际为 deg/s。
- 数据至少 60s；解码率 >=90%；IMU 250–400Hz；左右图像数相同；时间戳单调。
- 默认 IMU 参数：0.02、0.002、0.002、0.0002，结果必须标记 provisional。
- Debian 11 宿主机不安装 ROS Desktop、RViz、Gazebo 或完整 ROS，不启动 roscore。
- ARM64 运行镜像目标 1–2GB，超过 2GB 必须输出镜像层和依赖统计后裁剪。
- 浏览器或 SSH 隧道断开不得终止 RK 上已经开始的录制或求解。
- 不修改或提交用户未跟踪目录 diagnostics/。

## File Structure

- src/stereo_calibrator/kalibr/imu_decoder.py：码带和时钟解码。
- src/stereo_calibrator/kalibr/recorder.py：图像、IMU、曝光和元数据落盘。
- src/stereo_calibrator/kalibr/validation.py：质量门槛。
- src/stereo_calibrator/kalibr/config_writer.py：Kalibr YAML 和 manifest。
- src/stereo_calibrator/kalibr/runtime.py：Docker 作业。
- src/stereo_calibrator/kalibr/controller.py：状态机。
- kalibr_runtime/：ARM64 镜像、制包和求解脚本。
- headless.py 仅增加模式路由和逐帧回调；web.py 仅负责 UI/API。

---

### Task 1: IMU Code-Band Decoder

**Files:**
- Create: src/stereo_calibrator/kalibr/__init__.py
- Create: src/stereo_calibrator/kalibr/imu_decoder.py
- Create: tests/test_imu_decoder.py
- Create: tests/fixtures/imu_reference_cli.cpp

**Interfaces:**
- Consumes: BGR 原始帧、frame_idx、DeviceClock。
- Produces: decode_vertical_band(frame, frame_idx, unwrapper) -> DecodedImuFrame；其中含曝光中点和 ImuSample 元组。

- [ ] **Step 1: Write failing payload and clock tests**

~~~python
def test_parse_payload_matches_reference_units():
    payload = make_payload(exp_start=1000, exp_end=2000, t_us=1500,
                           acc=(8192, -8192, 16384),
                           gyro=(3276, -3276, 0), samples=1)
    result = parse_payload(payload, 7, DeviceClock())
    assert result.image_timestamp_ns == 1_500_000
    assert result.samples[0].accel_mps2 == pytest.approx(
        (9.80665, -9.80665, 19.6133))
    assert result.samples[0].gyro_rps == pytest.approx(
        (0.17449, -0.17449, 0.0), abs=1e-4)

def test_device_clock_unwraps_uint32():
    clock = DeviceClock()
    assert clock.unwrap(0xfffffff0) == 0xfffffff0
    assert clock.unwrap(0x10) == 0x100000010
    with pytest.raises(ValueError, match="时间戳回退"):
        clock.unwrap(0x08)
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_imu_decoder.py -q

Expected: import FAIL because imu_decoder.py is absent.

- [ ] **Step 3: Implement typed parsing**

~~~python
@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    frame_idx: int
    raw_t_us: int
    gyro_rps: tuple[float, float, float]
    accel_mps2: tuple[float, float, float]

@dataclass(frozen=True)
class DecodedImuFrame:
    frame_idx: int
    exp_start_ns: int
    exp_end_ns: int
    image_timestamp_ns: int
    payload_bytes: int
    samples: Sequence[ImuSample]
    magnetic_samples: Sequence[MagSample]
~~~

Implement be_u32, be_s16, repeated old-header detection, invalid sentinel filtering, exposure midpoint, ACC_SENS=4000/32768 mg/LSB and GYR_SENS=1000/32768 deg/s/LSB. Preserve byte compatibility with hardware/imu/imu_decode.h while correcting its historical gx_mdps label before conversion to rad/s. Decode AK09940 into magnetic_samples with MAG_SENS=0.15 uT/LSB for decoder diagnostics, but never expose those samples to the Kalibr bag writer.

- [ ] **Step 4: Add a synthetic vertical-band round-trip**

~~~python
def test_vertical_band_decodes_lsb_first_groups():
    payload = make_payload(exp_start=10_000, exp_end=10_500,
                           t_us=10_250, samples=1)
    frame = encode_vertical_band(payload, height=1200, width=4000)
    result = decode_vertical_band(frame, 3, DeviceClock())
    assert result.payload_bytes >= 32
    assert result.samples[0].raw_t_us == 10_250
~~~

Implement column strategies (3,+8), (3,-8), (width-3,-8), (width-3,+8), four bright sync pixels, half-unit offset, two-green-pixel thresholds, LSB-first assembly and 384-byte bound.

- [ ] **Step 5: Compare Python bytes with the existing C++ reference**

tests/fixtures/imu_reference_cli.cpp includes the existing unified_capture/hardware/imu/imu_decode.h, reads width/height followed by raw luma bytes from stdin, calls imu_read_luma_vertical(), and prints the decoded bytes as lowercase hex. The pytest fixture compiles it with c++ -std=c++20, feeds the same synthetic band to both implementations, and asserts identical hex including old-protocol and 32-bit-wrap samples.

~~~python
def test_python_payload_bytes_equal_cpp_reference(cpp_reference, synthetic_band):
    python_hex = decode_vertical_payload_bytes(synthetic_band).hex()
    cpp_hex = cpp_reference.decode(synthetic_band)
    assert python_hex == cpp_hex
~~~

- [ ] **Step 6: Verify and commit**

Run: .venv/bin/pytest tests/test_imu_decoder.py -q

~~~bash
git add src/stereo_calibrator/kalibr tests/test_imu_decoder.py tests/fixtures/imu_reference_cli.cpp
git commit -m "feat: decode world intelligent IMU code band"
~~~

---

### Task 2: Continuous Dataset Recorder

**Files:**
- Create: src/stereo_calibrator/kalibr/recorder.py
- Create: tests/test_kalibr_recorder.py
- Modify: configs/default.yaml

**Interfaces:**
- Consumes: ingest(raw_frame, left, right, frame_idx) from the existing camera loop.
- Produces: start(), stop() -> CaptureSummary, abort(reason), snapshot(); kalibr/cam0, cam1, imu0.csv, frames.csv, capture.json and decoder_stats.json.

- [ ] **Step 1: Write failing layout and timestamp tests**

~~~python
def test_recorder_writes_paired_images_and_csv(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder(), image_stride=3)
    recorder.start(started_monotonic=10.0)
    for index in range(4):
        recorder.ingest(world_frame(), eye(), eye(), index)
    summary = recorder.stop(stopped_monotonic=71.0)
    assert summary.duration_seconds == 61.0
    assert summary.left_images == summary.right_images == 2
    assert len(list((tmp_path / "kalibr/cam0").glob("*.png"))) == 2
    assert read_csv(tmp_path / "kalibr/imu0.csv")[0][0] == "timestamp_ns"
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_recorder.py -q

Expected: import FAIL.

- [ ] **Step 3: Implement recorder**

~~~python
class KalibrRecorder:
    def start(self, started_monotonic=None) -> None:
        if self._state != "ready":
            raise RuntimeError("Kalibr 录制已经开始")
        self._started_monotonic = (
            time.monotonic() if started_monotonic is None else started_monotonic)
        self._open_dataset_files()
        self._state = "recording"

    def ingest(self, raw_frame, left, right, frame_idx) -> None:
        if self._state != "recording":
            return
        decoded = self._decoder(raw_frame, frame_idx, self._clock)
        self._write_decoded_frame(decoded, left, right)

    def stop(self, stopped_monotonic=None) -> CaptureSummary:
        stopped = time.monotonic() if stopped_monotonic is None else stopped_monotonic
        self._close_dataset_files(complete=True, error=None)
        self._state = "recorded"
        return self._capture_summary(stopped)
~~~

Open CSV files once, flush every second, write grayscale paired PNG files from the same exposure timestamp, retain all IMU samples, and use image_stride=3 so 30fps UVC becomes 10fps images without reducing IMU frequency. Reject duplicate image timestamps before writing either eye; count and omit duplicate IMU samples.

- [ ] **Step 4: Test incomplete recovery**

~~~python
def test_abort_marks_capture_incomplete(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder())
    recorder.start()
    recorder.abort("V4L2 连续采帧失败")
    meta = json.loads((tmp_path / "kalibr/capture.json").read_text())
    assert meta["complete"] is False
    assert meta["error"] == "V4L2 连续采帧失败"
~~~

- [ ] **Step 5: Add config and verify**

~~~yaml
kalibr:
  minimum_duration_seconds: 60
  recommended_duration_seconds: 90
  image_stride: 3
  minimum_decode_ratio: 0.90
  minimum_imu_rate_hz: 250
  maximum_imu_rate_hz: 400
~~~

Run: .venv/bin/pytest tests/test_kalibr_recorder.py tests/test_imu_decoder.py -q

- [ ] **Step 6: Commit**

~~~bash
git add configs/default.yaml src/stereo_calibrator/kalibr/recorder.py tests/test_kalibr_recorder.py
git commit -m "feat: record synchronized Kalibr datasets"
~~~

---

### Task 3: Quality Gates and Kalibr Configs

**Files:**
- Create: src/stereo_calibrator/kalibr/validation.py
- Create: src/stereo_calibrator/kalibr/config_writer.py
- Create: tests/test_kalibr_validation.py
- Create: tests/test_kalibr_config.py

**Interfaces:**
- Produces: validate_dataset(path, config) -> ValidationReport and write_kalibr_configs(path, report) -> KalibrConfigPaths.

- [ ] **Step 1: Write failing gate tests**

~~~python
@pytest.mark.parametrize("mutation,reason", [
    ("short", "录制时长不足 60 秒"),
    ("pair_mismatch", "左右图像数量不一致"),
    ("decode_89_percent", "码带解码成功率低于 90%"),
    ("imu_200hz", "IMU 频率不在 250–400Hz"),
    ("timestamp_regression", "时间戳不单调"),
])
def test_mandatory_failures(dataset_factory, mutation, reason):
    report = validate_dataset(dataset_factory(mutation), default_config())
    assert not report.passed
    assert reason in report.reasons
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_validation.py -q

- [ ] **Step 3: Implement deterministic validation**

Compute median frequency from positive IMU deltas. Require gyro span >=0.35 rad/s on each axis, dynamic acceleration span >=1.5 m/s² on each axis, at least 60 frames with both 8×5 boards detected, and at least five occupied cells in a 3×3 normalized board-center grid. Return exact retake reasons and numeric metrics.

- [ ] **Step 4: Write config tests**

~~~python
def test_exact_target_and_provisional_imu_yaml(tmp_path, passing_report):
    paths = write_kalibr_configs(tmp_path, passing_report)
    target = yaml.safe_load(paths.target.read_text())
    imu = yaml.safe_load(paths.imu.read_text())
    assert target["targetCols"] == 8 and target["targetRows"] == 5
    assert target["rowSpacingMeters"] == 0.020
    assert imu["rostopic"] == "/imu0"
    assert imu["gyroscope_noise_density"] == 0.002
    assert imu["provisional"] is True
~~~

- [ ] **Step 5: Implement atomic target.yaml, imu.yaml and pipeline.json**

pipeline.json records topics /cam0/image_raw, /cam1/image_raw and /imu0, two pinhole-radtan models, measured IMU rate, Kalibr commit, session ID and provisional warning. Write via temporary files followed by Path.replace().

- [ ] **Step 6: Verify and commit**

Run: .venv/bin/pytest tests/test_kalibr_validation.py tests/test_kalibr_config.py -q

~~~bash
git add src/stereo_calibrator/kalibr/validation.py src/stereo_calibrator/kalibr/config_writer.py tests/test_kalibr_validation.py tests/test_kalibr_config.py
git commit -m "feat: validate and configure Kalibr datasets"
~~~

---

### Task 4: Bag Builder, Pipeline and Result Summary

**Files:**
- Create: kalibr_runtime/scripts/build_bag.py
- Create: kalibr_runtime/scripts/run_pipeline.sh
- Create: kalibr_runtime/scripts/summarize.py
- Create: tests/test_kalibr_runtime_scripts.py

**Interfaces:**
- Consumes: mounted /data/kalibr.
- Produces: results/dataset.bag, native Kalibr YAML/PDF/logs and summary.json.

- [ ] **Step 1: Write script-contract tests**

~~~python
def test_pipeline_is_headless_and_uses_fixed_topics():
    text = Path("kalibr_runtime/scripts/run_pipeline.sh").read_text()
    assert all(x in text for x in (
        "/cam0/image_raw", "/cam1/image_raw", "/imu0",
        "kalibr_calibrate_cameras", "kalibr_calibrate_imu_camera"))
    assert "roscore" not in text

def test_summary_keeps_imu_transform(sample_outputs, tmp_path):
    result = summarize(sample_outputs, tmp_path / "summary.json")
    assert result["provisional_imu_noise"] is True
    assert len(result["T_cam0_imu"]) == 4
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_runtime_scripts.py -q

- [ ] **Step 3: Implement direct offline rosbag writes**

~~~python
with rosbag.Bag(str(output), "w") as bag:
    for stamp, cam0, cam1 in read_frames(frames_csv):
        bag.write("/cam0/image_raw", image_msg(cam0, stamp), stamp)
        bag.write("/cam1/image_raw", image_msg(cam1, stamp), stamp)
    for sample in read_imu(imu_csv):
        bag.write("/imu0", imu_msg(sample), sample.stamp)
~~~

Use mono8 sensor_msgs/Image, set every header stamp/frame_id and reject non-monotonic input.

- [ ] **Step 4: Implement the sequential no-roscore pipeline**

~~~sh
export MPLBACKEND=Agg
python3 /opt/kalibr-tools/build_bag.py --dataset /data/kalibr --output /data/kalibr/results/dataset.bag
kalibr_calibrate_cameras --bag /data/kalibr/results/dataset.bag --target /data/kalibr/target.yaml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --dont-show-report
kalibr_calibrate_imu_camera --bag /data/kalibr/results/dataset.bag --cam /data/kalibr/results/camchain.yaml --imu /data/kalibr/imu.yaml --target /data/kalibr/target.yaml --dont-show-report
python3 /opt/kalibr-tools/summarize.py --results /data/kalibr/results --output /data/kalibr/results/summary.json
~~~

Write a stage.json before every phase. Resolve generated names to stable camchain.yaml and camchain-imucam.yaml while retaining native files.

- [ ] **Step 5: Verify and commit**

Run: .venv/bin/pytest tests/test_kalibr_runtime_scripts.py -q

~~~bash
git add kalibr_runtime/scripts tests/test_kalibr_runtime_scripts.py
git commit -m "feat: build Kalibr bags and normalize results"
~~~

---

### Task 5: Minimal ARM64 Runtime Image

**Files:**
- Create: kalibr_runtime/Dockerfile.arm64
- Create: kalibr_runtime/build-arm64.sh
- Create: kalibr_runtime/README.md
- Create: tests/test_kalibr_dockerfile.py
- Modify: .gitignore

**Interfaces:**
- Produces: stereo-kalibr-rk3588:1f60227442d25e36365ef5f72cd80b9666d73467 and an optional arm64 tar.zst.

- [ ] **Step 1: Write image-policy tests**

~~~python
def test_runtime_is_multistage_ros_core_not_desktop():
    text = Path("kalibr_runtime/Dockerfile.arm64").read_text()
    assert text.count("FROM ") >= 2
    assert "ros:noetic-ros-core-focal" in text
    assert "desktop-full" not in text
    assert "rviz" not in text and "gazebo" not in text
    assert "catkin build" in text and "--install" in text
    assert "USER kalibr" in text
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_dockerfile.py -q

- [ ] **Step 3: Implement pinned multi-stage Dockerfile**

Builder installs compiler/catkin/development dependencies, clones official Kalibr, checks an exact 40-character commit, and builds Release/install with -j2. Runtime starts from ros:noetic-ros-core-focal, installs only rosbag, sensor_msgs, cv_bridge plus necessary runtime system/Python libraries, copies the catkin install space and scripts, and runs as user kalibr.

- [ ] **Step 4: Implement measurable build script**

~~~sh
docker build --platform linux/arm64 --build-arg KALIBR_COMMIT="$KALIBR_COMMIT" -f kalibr_runtime/Dockerfile.arm64 -t "$image" .
docker image inspect "$image" --format '{{.Architecture}} {{.Size}}'
docker history --no-trunc "$image"
docker run --rm --platform linux/arm64 "$image" kalibr_calibrate_cameras --help
~~~

Fail unless architecture is arm64, both Kalibr help commands exit 0, and image Size <=2147483648 bytes. Export only after validation.

- [ ] **Step 5: Run static and actual build checks**

Run: .venv/bin/pytest tests/test_kalibr_dockerfile.py -q

Run: KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467 ./kalibr_runtime/build-arm64.sh

Expected: ARM64 image <=2GB; help commands pass; no roscore process.

- [ ] **Step 6: Commit sources, not image archives**

~~~bash
git add kalibr_runtime .gitignore tests/test_kalibr_dockerfile.py
git commit -m "build: add minimal ARM64 Kalibr runtime"
~~~

---

### Task 6: Persistent Job Controller

**Files:**
- Create: src/stereo_calibrator/kalibr/runtime.py
- Create: src/stereo_calibrator/kalibr/controller.py
- Create: tests/test_kalibr_controller.py

**Interfaces:**
- Produces: action(name), ingest(raw_frame, left, right, frame_idx), snapshot(), artifacts() and artifact_path(key).

- [ ] **Step 1: Write state-machine tests**

~~~python
def test_happy_path(controller):
    assert controller.action("kalibr_start") == {"ok": True}
    assert controller.snapshot()["state"] == "recording"
    assert controller.action("kalibr_stop") == {"ok": True}
    assert controller.action("kalibr_validate") == {"ok": True}
    assert controller.snapshot()["state"] == "ready_to_solve"
    assert controller.action("kalibr_solve") == {"ok": True}

def test_repeated_solve_reuses_running_job(running_controller):
    assert running_controller.action("kalibr_solve") == {
        "ok": True, "reused": True}
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_controller.py -q

- [ ] **Step 3: Implement KalibrRuntime**

~~~python
class KalibrRuntime:
    def launch(self, dataset_dir: Path, session_id: str) -> str:
        name = f"stereo-kalibr-{session_id}"
        command = [
            "docker", "run", "-d", "--name", name,
            "--label", f"stereo.kalibr.session={session_id}",
            "--network", "none", "--memory", "6g", "--read-only",
            "-v", f"{dataset_dir.resolve()}:/data/kalibr:rw",
            self.image,
        ]
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip())
        return name

    def logs(self, container_name: str, tail: int = 100) -> str:
        completed = subprocess.run(
            ["docker", "logs", "--tail", str(tail), container_name],
            capture_output=True, text=True)
        return completed.stdout + completed.stderr
~~~

Launch a detached named container with session label, --network none, --memory 6g, read-only root, and only the session bind-mounted writable. Persist identity in kalibr/job.json and rediscover it after service restart.

- [ ] **Step 4: Implement transitions**

Allow ready→recording→recorded→validating→ready_to_solve→bagging→camera_calibrating→imu_calibrating→pass/retake/error. Poll stage.json and Docker without blocking the camera loop. Validation failures preserve the dataset and exact reasons; runtime failures preserve bag/logs and prior successful results.

- [ ] **Step 5: Test artifact allow-list**

~~~python
def test_artifact_path_cannot_escape(controller):
    assert "summary" in controller.artifacts()
    with pytest.raises(ValueError, match="非法产物"):
        controller.artifact_path("../../etc/passwd")
~~~

- [ ] **Step 6: Verify and commit**

Run: .venv/bin/pytest tests/test_kalibr_controller.py -q

~~~bash
git add src/stereo_calibrator/kalibr/runtime.py src/stereo_calibrator/kalibr/controller.py tests/test_kalibr_controller.py
git commit -m "feat: manage persistent RK Kalibr jobs"
~~~

---

### Task 7: Shared Camera Loop, API and UI

**Files:**
- Modify: src/stereo_calibrator/headless.py
- Modify: src/stereo_calibrator/web.py
- Modify: tests/test_headless.py
- Modify: tests/test_web.py

**Interfaces:**
- Adds actions mode_opencv, mode_kalibr, kalibr_start, kalibr_stop, kalibr_validate and kalibr_solve.
- Adds GET /api/artifacts and GET /artifact/{key}.

- [ ] **Step 1: Write shared-camera test**

~~~python
def test_kalibr_uses_same_camera_loop(tmp_path):
    controller = FakeKalibrController()
    engine = HeadlessCalibrationEngine(
        make_config(), tmp_path, .020, "/dev/video0",
        camera=WorldCamera(), kalibr_controller=controller)
    assert engine.action("mode_kalibr") == {"ok": True}
    assert engine.action("kalibr_start") == {"ok": True}
    engine.start()
    wait_until(lambda: controller.ingested_frames >= 1)
    engine.action("stop")
    assert controller.ingested_frames >= 1
~~~

Also test that switching mode is rejected during recording/solving, camera errors call controller.abort(), status stays JSON-safe, and every existing OpenCV action remains unchanged.

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_headless.py -q

- [ ] **Step 3: Integrate without opening a second camera**

After split_profile_frame, pass raw/left/right to controller only in Kalibr recording. Skip OpenCV corner detection during Kalibr recording to reserve CPU, but keep MJPEG. Merge controller status under workflow="kalibr" and a concrete kalibr status dictionary; do not rename old fields.

- [ ] **Step 4: Write UI/API tests**

~~~python
def test_page_has_two_modes_and_no_stop_service(running_server):
    html = urlopen(running_server[1] + "/").read().decode()
    assert "OpenCV Chessboard" in html
    assert "Kalibr Camera+IMU" in html
    assert "kalibr_start" in html and "kalibr_solve" in html
    assert "停止服务" not in html and "act('stop')" not in html
~~~

Test artifact list/download, invalid keys, and Content-Disposition.

- [ ] **Step 5: Implement UI and endpoints**

Show duration, paired frames, decode ratio, IMU Hz, validation reasons, image state, pipeline stage, latest 100 log lines and download links. Disable mode buttons while recording or solving. Downloads resolve only through artifact_path(key).

- [ ] **Step 6: Verify all regression tests and commit**

Run: .venv/bin/pytest tests/test_headless.py tests/test_web.py -q

Run: .venv/bin/pytest -q

~~~bash
git add src/stereo_calibrator/headless.py src/stereo_calibrator/web.py tests/test_headless.py tests/test_web.py
git commit -m "feat: add OpenCV and Kalibr web workflows"
~~~

---

### Task 8: Deployment and End-to-End RK Acceptance

**Files:**
- Modify: deploy/start.sh
- Create: deploy/install-kalibr-runtime.sh
- Create: tests/test_kalibr_deploy.py
- Modify: README.md
- Modify: docs/零基础使用手册.md

**Interfaces:**
- Produces: repeatable image import, direct 18765 tunnel, live result artifacts.

- [ ] **Step 1: Write deployment contract tests**

~~~python
def test_start_keeps_direct_tunnel_and_checks_runtime():
    text = Path("deploy/start.sh").read_text()
    assert "127.0.0.1:18765:127.0.0.1:8765" in text
    assert "/api/status" in text and "kalibr" in text
    assert "18766" not in text

def test_installer_never_installs_ros_on_host():
    text = Path("deploy/install-kalibr-runtime.sh").read_text()
    assert "docker load" in text
    assert "apt install ros-" not in text
    assert "desktop-full" not in text
~~~

- [ ] **Step 2: Verify failure**

Run: .venv/bin/pytest tests/test_kalibr_deploy.py tests/test_deploy_start.py -q

- [ ] **Step 3: Implement idempotent deployment**

Installer verifies arm64, >=4GB free disk, archive SHA-256, imports only if image ID is absent, then executes both Kalibr help commands. start.sh keeps the direct tunnel and reports Kalibr availability without blocking OpenCV when the image is absent.

- [ ] **Step 4: Sync only project source**

~~~bash
rsync -az --delete --exclude '.git/' --exclude '.venv/'   --exclude 'diagnostics/' --exclude 'sessions/' --exclude 'backups/'   --exclude 'deploy/.runtime/'   ./ root@192.168.100.200:/root/stereo_chessboard_calibrator/
~~~

Expected: existing sessions, backups and diagnostics remain untouched.

- [ ] **Step 5: Run local and RK automated checks**

~~~bash
.venv/bin/pytest -q
git diff --check
ssh root@192.168.100.200 'cd /root/stereo_chessboard_calibrator && .venv/bin/pytest -q'
ssh root@192.168.100.200 'docker image inspect stereo-kalibr-rk3588:* --format "{{.Architecture}} {{.Size}}"'
ssh root@192.168.100.200 'pgrep -af roscore && exit 1 || true'
~~~

Expected: tests pass; image arm64 <=2GB; no roscore.

- [ ] **Step 6: Run a 10-second live decoder smoke test**

Record the connected world intelligent camera for at least 10 seconds while moving it. Verify paired image counts, decode ratio >=90%, IMU 250–400Hz, and monotonic timestamps. Mark this dataset transport-only, not calibration input.

- [ ] **Step 7: Build and inspect a smoke ROS bag**

Run the container on the smoke dataset and rosbag info inside it. Expect exactly /cam0/image_raw, /cam1/image_raw and /imu0; equal camera counts; rates matching CSV; no host roscore.

- [ ] **Step 8: Run the physical 60–120 second session**

Ask the operator to keep the board visible while moving through left/right/up/down/near/far and smooth roll/pitch/yaw plus acceleration/deceleration. Stop only after at least 60s, five coverage cells, adequate three-axis excitation, decode >=90%, and IMU 250–400Hz. A failed gate must enter retake with exact reasons.

- [ ] **Step 9: Prove RK-local solving survives browser disconnect**

Start solve from http://127.0.0.1:18765/, close browser/tunnel after the detached container starts, wait on RK, restore tunnel, and inspect. Expect pass or a meaningful Kalibr retake; summary.json, camera YAML/PDF, IMU-camera YAML/PDF, bag statistics and logs must download.

- [ ] **Step 10: Update docs and commit**

Document both modes, why the minimal container exists, operator motion, quality messages, provisional warning, result filenames, restart recovery and the URL. Replace README screenshot after browser verification.

~~~bash
git add deploy/start.sh deploy/install-kalibr-runtime.sh tests/test_kalibr_deploy.py README.md docs/零基础使用手册.md docs/images
git commit -m "docs: add RK Kalibr deployment and operating guide"
~~~

- [ ] **Step 11: Final verification and push**

~~~bash
.venv/bin/pytest -q
git diff --check
git status --short
git log --oneline -10
git push origin main
~~~

Expected: all tests pass; only user-owned diagnostics/ may remain untracked; commits appear on origin/main.
