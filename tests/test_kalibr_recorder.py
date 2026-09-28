import json

import cv2
import numpy as np

from stereo_calibrator.kalibr.imu_decoder import DecodedImuFrame, ImuSample
from stereo_calibrator.kalibr.recorder import KalibrRecorder


class FakeDecoder:
    def __init__(self, duplicate_imu=False, samples_per_frame=1):
        self.duplicate_imu = duplicate_imu
        self.samples_per_frame = samples_per_frame

    def __call__(self, _frame, frame_idx, _clock):
        image_timestamp_ns = 1_500_000_000 + frame_idx * 100_000_000
        samples = tuple(
            ImuSample(
                timestamp_ns=(
                    1_400_000_000
                    if self.duplicate_imu
                    else 1_400_000_000 + frame_idx * 100_000_000 + index * 10_000_000
                ),
                frame_idx=frame_idx,
                raw_t_us=1_400_000 + index,
                gyro_rps=(0.1, 0.2, 0.3),
                accel_mps2=(1.0, 2.0, 9.8),
            )
            for index in range(self.samples_per_frame)
        )
        return DecodedImuFrame(
            frame_idx=frame_idx,
            exp_start_ns=image_timestamp_ns - 1_000_000,
            exp_end_ns=image_timestamp_ns + 1_000_000,
            image_timestamp_ns=image_timestamp_ns,
            payload_bytes=32,
            samples=samples,
            magnetic_samples=(),
        )


def eye(value=127):
    return np.full((24, 32, 3), value, np.uint8)


def raw_frame():
    return np.full((24, 80, 3), 127, np.uint8)


def video_frame_count(path):
    cap = cv2.VideoCapture(str(path))
    count = 0
    while True:
        ok, _frame = cap.read()
        if not ok:
            break
        count += 1
    cap.release()
    return count


def test_recorder_writes_video_and_imu_json(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder())
    recorder.start(started_monotonic=10.0)
    for index in range(4):
        recorder.ingest(raw_frame(), eye(index), eye(index + 10), index)

    summary = recorder.stop(stopped_monotonic=71.0)

    dataset = tmp_path / "kalibr"
    assert summary.duration_seconds == 61.0
    assert summary.left_frames == summary.right_frames == 4
    assert video_frame_count(dataset / "cam0.avi") == 4
    assert video_frame_count(dataset / "cam1.avi") == 4
    imu = json.loads((dataset / "imu.json").read_text())
    assert len(imu["samples"]) == 4
    assert imu["samples"][0]["gyro_rps"] == [0.1, 0.2, 0.3]
    assert not (dataset / "imu0.csv").exists()
    assert not (dataset / "frames.csv").exists()
    assert not (dataset / "cam0").exists()
    assert not (dataset / "cam1").exists()
    capture = json.loads((dataset / "capture.json").read_text())
    assert capture["complete"] is True


def test_recorder_omits_duplicate_imu_timestamp(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder(duplicate_imu=True))
    recorder.start(started_monotonic=0.0)
    recorder.ingest(raw_frame(), eye(), eye(), 0)
    recorder.ingest(raw_frame(), eye(), eye(), 1)
    recorder.stop(stopped_monotonic=1.0)

    imu = json.loads((tmp_path / "kalibr/imu.json").read_text())
    assert len(imu["samples"]) == 1
    stats = json.loads((tmp_path / "kalibr/decoder_stats.json").read_text())
    assert stats["duplicate_imu_timestamps"] == 1


def test_video_frames_are_written_regardless_of_decode_failure(tmp_path):
    class OneDecodeFailure(FakeDecoder):
        def __call__(self, frame, frame_idx, clock):
            if frame_idx == 0:
                raise ValueError("invalid code band")
            return super().__call__(frame, frame_idx, clock)

    recorder = KalibrRecorder(tmp_path, decoder=OneDecodeFailure())
    recorder.start(started_monotonic=0.0)
    recorder.ingest(raw_frame(), eye(), eye(), 0)
    recorder.ingest(raw_frame(), eye(), eye(), 1)
    recorder.stop(stopped_monotonic=2.0)

    integrity = json.loads((tmp_path / "kalibr/integrity.json").read_text())
    assert integrity["passed"] is True
    assert integrity["total_frames"] == 2
    assert integrity["decoded_frames"] == 1
    assert integrity["left_frames"] == 2
    assert integrity["right_frames"] == 2
    assert video_frame_count(tmp_path / "kalibr/cam0.avi") == 2


def test_snapshot_exposes_decimated_display_imu(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder())
    recorder.configure({"display": {"interval_s": 0, "samples": 10}})
    recorder.start(started_monotonic=0.0)
    for index in range(3):
        recorder.ingest(raw_frame(), eye(), eye(), index)

    display = recorder.snapshot()["display_imu"]
    assert len(display) == 3
    assert display[-1]["frame_idx"] == 2
    recorder.stop(stopped_monotonic=1.0)


def test_abort_marks_capture_incomplete(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder())
    recorder.start(started_monotonic=10.0)

    recorder.abort("V4L2 连续采帧失败", stopped_monotonic=12.0)

    metadata = json.loads((tmp_path / "kalibr/capture.json").read_text())
    assert metadata["complete"] is False
    assert metadata["error"] == "V4L2 连续采帧失败"
    assert recorder.snapshot()["state"] == "error"


def test_recorder_refuses_second_start(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder())
    recorder.start()

    try:
        recorder.start()
    except RuntimeError as error:
        assert "已经开始" in str(error)
    else:
        raise AssertionError("second start must fail")
    finally:
        recorder.abort("test cleanup")
