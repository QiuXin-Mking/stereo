import struct

import numpy as np
import pytest

from stereo_calibrator.kalibr.imu_decoder import (
    DeviceClock,
    decode_vertical_band,
    decode_left_side_payload_bytes,
    decode_vertical_payload_bytes,
    parse_payload,
)


def _sample_group(t_us, acc=(8192, -8192, 16384), gyro=(3276, -3276, 0)):
    return struct.pack(">Ihhhhhh", t_us, *acc, *gyro)


def make_payload(
    *, exp_start=1000, exp_end=2000, t_us=1500,
    acc=(8192, -8192, 16384), gyro=(3276, -3276, 0), include_mag=False
):
    header = bytearray(16)
    header[0] = 1
    header[1] = 1
    header[8:12] = struct.pack(">I", exp_start)
    header[12:16] = struct.pack(">I", exp_end)
    payload = bytes(header) + _sample_group(t_us, acc, gyro)
    if include_mag:
        payload += (
            struct.pack(">I", t_us)
            + b"\x00\x00\x01"
            + b"\xff\xff\xff"
            + b"\x00\x00\x02"
            + bytes((0, 0, 0))
        )
    return payload


def make_multi_sample_payload(*timestamps):
    header = bytearray(16)
    header[0] = 1
    header[1] = len(timestamps)
    header[8:12] = struct.pack(">I", timestamps[0])
    header[12:16] = struct.pack(">I", timestamps[-1])
    return bytes(header) + b"".join(_sample_group(timestamp) for timestamp in timestamps)


def encode_vertical_band(payload, *, height=1200, width=4000):
    frame = np.full((height, width, 3), 255, np.uint8)
    for group_index in range(0, len(payload), 16):
        column = 3 + (group_index // 16) * 8
        frame[:, column, 1] = 255
        frame[4:8, column, 1] = 0
        chunk = payload[group_index : group_index + 16]
        for bit_index in range(len(chunk) * 8):
            value = (chunk[bit_index // 8] >> (bit_index % 8)) & 1
            row = 8 + bit_index * 8
            frame[row : row + 2, column, 1] = 100 if value else 0
        end = 8 + len(chunk) * 8 * 8
        frame[end : end + 2, column, 1] = 255
    return frame


def encode_left_side_band(payload, *, height=1200, width=4000):
    """Match the real world-intelligent layout: two bytes per image row."""
    frame = np.zeros((height, width, 3), np.uint8)
    for chunk_index in range(0, len(payload), 2):
        row = 3 + (chunk_index // 2) * 8
        frame[row, :4, 1] = 255
        chunk = payload[chunk_index : chunk_index + 2]
        for bit_index in range(len(chunk) * 8):
            value = (chunk[bit_index // 8] >> (bit_index % 8)) & 1
            column = 8 + bit_index * 8
            frame[row, column : column + 2, 1] = 100 if value else 0
        end = 8 + len(chunk) * 8 * 8
        frame[row, end : end + 2, 1] = 255
    return frame


def test_parse_payload_matches_reference_units():
    result = parse_payload(make_payload(), frame_idx=7, clock=DeviceClock())

    assert result.image_timestamp_ns == 1_500_000
    assert result.samples[0].frame_idx == 7
    assert result.samples[0].accel_mps2 == pytest.approx(
        (9.80665, -9.80665, 19.6133)
    )
    assert result.samples[0].gyro_rps == pytest.approx(
        (1.7449, -1.7449, 0.0), abs=2e-4
    )


def test_device_clock_unwraps_uint32_and_rejects_regression():
    clock = DeviceClock()

    assert clock.unwrap(0xFFFFFFF0) == 0xFFFFFFF0
    assert clock.unwrap(0x00000010) == 0x100000010
    with pytest.raises(ValueError, match="时间戳回退"):
        clock.unwrap(0x00000008)


def test_vertical_band_decodes_lsb_first_groups():
    payload = make_payload(exp_start=10_000, exp_end=10_500, t_us=10_250)
    frame = encode_vertical_band(payload)

    assert decode_vertical_payload_bytes(frame) == payload
    result = decode_vertical_band(frame, frame_idx=3, clock=DeviceClock())
    assert result.payload_bytes == 32
    assert result.samples[0].raw_t_us == 10_250


def test_left_side_band_decodes_across_rows_like_real_world_camera():
    payload = make_payload(exp_start=20_000, exp_end=20_500, t_us=20_250)
    frame = encode_left_side_band(payload)

    result = decode_vertical_band(frame, frame_idx=4, clock=DeviceClock())

    assert result.payload_bytes == 32
    assert result.image_timestamp_ns == 20_250_000
    assert result.samples[0].raw_t_us == 20_250


def test_left_side_decoder_ignores_matching_pattern_outside_code_band():
    payload = make_payload(exp_start=30_000, exp_end=30_500, t_us=30_250)
    frame = encode_left_side_band(payload)
    shifted = np.full_like(frame, 255)
    shifted[:, 300:] = frame[:, : frame.shape[1] - 300]

    assert decode_left_side_payload_bytes(shifted) == b""


def test_magnetometer_is_decoded_for_diagnostics():
    result = parse_payload(
        make_payload(include_mag=True), frame_idx=9, clock=DeviceClock()
    )

    assert len(result.magnetic_samples) == 1
    assert result.magnetic_samples[0].field_ut == pytest.approx(
        (2_526_412.8, -0.15, 5_052_825.6)
    )


def test_invalid_payload_is_rejected():
    with pytest.raises(ValueError, match="IMU payload"):
        parse_payload(b"short", frame_idx=0, clock=DeviceClock())


def test_all_zero_payload_is_rejected_as_false_positive():
    with pytest.raises(ValueError, match="IMU payload"):
        parse_payload(bytes(32), frame_idx=0, clock=DeviceClock())


def test_non_monotonic_sample_timestamps_are_rejected():
    with pytest.raises(ValueError, match="时间戳非单调"):
        parse_payload(
            make_multi_sample_payload(1_500, 1_500),
            frame_idx=0,
            clock=DeviceClock(),
        )
