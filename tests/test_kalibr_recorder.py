import csv
import json

import numpy as np

from stereo_calibrator.kalibr.imu_decoder import DecodedImuFrame, ImuSample
from stereo_calibrator.kalibr.recorder import KalibrRecorder


class FakeDecoder:
    def __init__(self, duplicate_imu=False):
        self.duplicate_imu = duplicate_imu

    def __call__(self, _frame, frame_idx, _clock):
        image_timestamp_ns = 1_500_000_000 + frame_idx * 100_000_000
        imu_timestamp_ns = (
            1_400_000_000 if self.duplicate_imu else 1_400_000_000 + frame_idx * 100_000_000
        )
        sample = ImuSample(
            timestamp_ns=imu_timestamp_ns,
            frame_idx=frame_idx,
            raw_t_us=imu_timestamp_ns // 1000,
            gyro_rps=(0.1, 0.2, 0.3),
            accel_mps2=(1.0, 2.0, 9.8),
        )
        return DecodedImuFrame(
            frame_idx=frame_idx,
            exp_start_ns=image_timestamp_ns - 1_000_000,
            exp_end_ns=image_timestamp_ns + 1_000_000,
            image_timestamp_ns=image_timestamp_ns,
            payload_bytes=32,
            samples=(sample,),
            magnetic_samples=(),
        )


def eye(value=127):
    return np.full((24, 32, 3), value, np.uint8)


def raw_frame():
    return np.full((24, 80, 3), 127, np.uint8)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.reader(handle))


def test_recorder_writes_paired_images_and_csv(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder(), image_stride=3)
    recorder.start(started_monotonic=10.0)
    for index in range(4):
        recorder.ingest(raw_frame(), eye(index), eye(index + 10), index)

    summary = recorder.stop(stopped_monotonic=71.0)

    assert summary.duration_seconds == 61.0
    assert summary.left_images == summary.right_images == 2
    assert len(list((tmp_path / "kalibr/cam0").glob("*.png"))) == 2
    assert len(list((tmp_path / "kalibr/cam1").glob("*.png"))) == 2
    assert read_csv(tmp_path / "kalibr/imu0.csv")[0] == [
        "timestamp_ns", "omega_x", "omega_y", "omega_z",
        "alpha_x", "alpha_y", "alpha_z", "frame_idx", "t_us",
    ]
    frames = read_csv(tmp_path / "kalibr/frames.csv")
    assert len(frames) == 5
    assert frames[1][-2:] == ["cam0/1500000000.png", "cam1/1500000000.png"]
    assert frames[2][-2:] == ["", ""]
    capture = json.loads((tmp_path / "kalibr/capture.json").read_text())
    assert capture["complete"] is True


def test_recorder_omits_duplicate_imu_timestamp(tmp_path):
    recorder = KalibrRecorder(tmp_path, decoder=FakeDecoder(duplicate_imu=True))
    recorder.start(started_monotonic=0.0)
    recorder.ingest(raw_frame(), eye(), eye(), 0)
    recorder.ingest(raw_frame(), eye(), eye(), 1)
    recorder.stop(stopped_monotonic=1.0)

    assert len(read_csv(tmp_path / "kalibr/imu0.csv")) == 2
    stats = json.loads((tmp_path / "kalibr/decoder_stats.json").read_text())
    assert stats["duplicate_imu_timestamps"] == 1


def test_integrity_compares_frames_to_decoded_frames_not_raw_frames(tmp_path):
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
    assert integrity["frame_rows"] == 1


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
