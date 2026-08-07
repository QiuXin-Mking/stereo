from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


IMU_VAL0 = 50
IMU_VAL1 = 220
IMU_USIZE = 8
IMU_GROUP = 16
IMU_TARGET = 272
IMU_MAX_BYTES = 384
ACC_SENS_MG = 4000.0 / 32768.0
GYR_SENS_DPS = 1000.0 / 32768.0
MAG_SENS_UT = 0.15
UINT32_RANGE = 1 << 32
UINT32_HALF_RANGE = 1 << 31


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    frame_idx: int
    raw_t_us: int
    gyro_rps: tuple[float, float, float]
    accel_mps2: tuple[float, float, float]


@dataclass(frozen=True)
class MagSample:
    timestamp_ns: int
    frame_idx: int
    raw_t_us: int
    field_ut: tuple[float, float, float]
    temperature_c: float


@dataclass(frozen=True)
class DecodedImuFrame:
    frame_idx: int
    exp_start_ns: int
    exp_end_ns: int
    image_timestamp_ns: int
    payload_bytes: int
    samples: Sequence[ImuSample]
    magnetic_samples: Sequence[MagSample]


class DeviceClock:
    """Expand a wrapping uint32 microsecond device clock."""

    def __init__(self) -> None:
        self._latest_us: Optional[int] = None

    @property
    def latest_us(self) -> Optional[int]:
        return self._latest_us

    def unwrap(self, raw_us: int) -> int:
        raw = int(raw_us) & 0xFFFFFFFF
        if self._latest_us is None:
            self._latest_us = raw
            return raw
        last_raw = self._latest_us & 0xFFFFFFFF
        epoch = self._latest_us - last_raw
        if raw < last_raw:
            if last_raw - raw <= UINT32_HALF_RANGE:
                raise ValueError(
                    f"IMU 时间戳回退：previous={last_raw}, current={raw}"
                )
            epoch += UINT32_RANGE
        expanded = epoch + raw
        if expanded < self._latest_us:
            raise ValueError(
                f"IMU 时间戳回退：previous={self._latest_us}, current={expanded}"
            )
        self._latest_us = expanded
        return expanded

    def expand_near(self, raw_us: int, reference_us: int) -> int:
        raw = int(raw_us) & 0xFFFFFFFF
        base_epoch = reference_us - (reference_us & 0xFFFFFFFF)
        candidates = (base_epoch - UINT32_RANGE + raw, base_epoch + raw, base_epoch + UINT32_RANGE + raw)
        return min(candidates, key=lambda value: abs(value - reference_us))

    def observe(self, expanded_us: int) -> None:
        if self._latest_us is None or expanded_us >= self._latest_us:
            self._latest_us = int(expanded_us)


def _be_u32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], "big", signed=False)


def _be_s16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big", signed=True)


def _ak09940_axis(data: bytes) -> int:
    value = data[2]
    value = data[2] + (value << 8)
    value = data[1] + (value << 8)
    value = data[0] + (value << 8)
    return value - (1 << 32) if value & 0x80000000 else value


def parse_payload(
    payload: bytes, frame_idx: int, clock: Optional[DeviceClock] = None
) -> DecodedImuFrame:
    if len(payload) < IMU_GROUP:
        raise ValueError(f"IMU payload 不足 {IMU_GROUP} 字节")
    device_clock = clock or DeviceClock()
    header = payload[:IMU_GROUP]
    sample_count = header[1] & 0x0F
    if header[:8] == header[8:16]:
        sample_count = 11
    if not 1 <= sample_count <= 16:
        raise ValueError(f"IMU payload 样本数非法：{sample_count}")

    raw_exp_start = _be_u32(header, 8)
    raw_exp_end = _be_u32(header, 12)
    samples = []
    for index in range(sample_count):
        offset = (1 + index) * IMU_GROUP
        if offset + IMU_GROUP > len(payload):
            break
        group = payload[offset : offset + IMU_GROUP]
        raw_t_us = _be_u32(group, 0)
        raw_accel = tuple(_be_s16(group, value) for value in (4, 6, 8))
        raw_gyro = tuple(_be_s16(group, value) for value in (10, 12, 14))
        if raw_accel == (-1, -1, -1) or raw_gyro[0] == -32768:
            continue
        timestamp_us = device_clock.unwrap(raw_t_us)
        accel = tuple(value * ACC_SENS_MG * 9.80665 / 1000.0 for value in raw_accel)
        gyro = tuple(value * GYR_SENS_DPS * math.pi / 180.0 for value in raw_gyro)
        samples.append(
            ImuSample(
                timestamp_ns=timestamp_us * 1000,
                frame_idx=int(frame_idx),
                raw_t_us=raw_t_us,
                gyro_rps=gyro,
                accel_mps2=accel,
            )
        )

    anchor_us = (
        samples[-1].timestamp_ns // 1000
        if samples
        else (device_clock.latest_us if device_clock.latest_us is not None else raw_exp_start)
    )
    exp_start_us = device_clock.expand_near(raw_exp_start, anchor_us)
    exposure_duration = (raw_exp_end - raw_exp_start) & 0xFFFFFFFF
    if exposure_duration > UINT32_HALF_RANGE:
        raise ValueError("IMU payload 曝光时间倒退")
    exp_end_us = exp_start_us + exposure_duration

    magnetic_samples = []
    offset = (1 + sample_count) * IMU_GROUP
    while offset + IMU_GROUP <= len(payload):
        group = payload[offset : offset + IMU_GROUP]
        raw_t_us = _be_u32(group, 0)
        timestamp_us = device_clock.expand_near(raw_t_us, anchor_us)
        field = tuple(
            _ak09940_axis(group[position : position + 3]) * MAG_SENS_UT
            for position in (4, 7, 10)
        )
        magnetic_samples.append(
            MagSample(
                timestamp_ns=timestamp_us * 1000,
                frame_idx=int(frame_idx),
                raw_t_us=raw_t_us,
                field_ut=field,
                temperature_c=30.0 - int.from_bytes(group[13:14], "big", signed=True) / 1.7,
            )
        )
        offset += IMU_GROUP

    device_clock.observe(max(anchor_us, exp_end_us))
    return DecodedImuFrame(
        frame_idx=int(frame_idx),
        exp_start_ns=exp_start_us * 1000,
        exp_end_ns=exp_end_us * 1000,
        image_timestamp_ns=((exp_start_us + exp_end_us) * 1000) // 2,
        payload_bytes=len(payload),
        samples=tuple(samples),
        magnetic_samples=tuple(magnetic_samples),
    )


def _decode_luma_line(line: np.ndarray) -> bytes:
    if line.ndim != 1 or line.size < 5 or not np.all(line[:4] > IMU_VAL1):
        return b""
    below = np.flatnonzero(line < IMU_VAL1)
    if below.size == 0:
        return b""
    index = int(below[0]) + (IMU_USIZE >> 1)
    bits = []
    while index + 1 < line.size:
        value = int(line[index]) + int(line[index + 1])
        if value < (IMU_VAL0 << 1):
            bits.append(0)
        elif value < (IMU_VAL1 << 1):
            bits.append(1)
        else:
            break
        index += IMU_USIZE
    output = bytearray(len(bits) // 8)
    for byte_index in range(len(output)):
        for bit_index in range(8):
            output[byte_index] |= bits[byte_index * 8 + bit_index] << bit_index
    return bytes(output)


def decode_vertical_payload_bytes(frame: np.ndarray) -> bytes:
    if frame.ndim == 3 and frame.shape[2] >= 2:
        luma = frame[:, :, 1]
    elif frame.ndim == 2:
        luma = frame
    else:
        raise ValueError("码带帧必须是灰度图或 BGR 图")
    height, width = luma.shape
    if height > 8192 or height <= 0 or width <= 0:
        return b""
    strategies = ((3, IMU_USIZE), (3, -IMU_USIZE), (width - 3, -IMU_USIZE), (width - 3, IMU_USIZE))
    for start, step in strategies:
        decoded = bytearray()
        column = start
        while 0 <= column < width:
            group = _decode_luma_line(luma[:, column])
            if group:
                remaining = IMU_MAX_BYTES - len(decoded)
                decoded.extend(group[:remaining])
            if len(decoded) >= IMU_TARGET or len(decoded) >= IMU_MAX_BYTES:
                break
            column += step
        if len(decoded) >= IMU_GROUP:
            return bytes(decoded)
    return b""


def decode_left_side_payload_bytes(frame: np.ndarray) -> bytes:
    """Decode a left-side code band whose chunks run across image rows."""
    if frame.ndim == 3 and frame.shape[2] >= 2:
        luma = frame[:, :, 1]
    elif frame.ndim == 2:
        luma = frame
    else:
        raise ValueError("码带帧必须是灰度图或 BGR 图")
    height, width = luma.shape
    if height <= 0 or width <= 0:
        return b""
    decoded = bytearray()
    for row in range(3, height, IMU_USIZE):
        group = _decode_luma_line(luma[row, :])
        if group:
            remaining = IMU_MAX_BYTES - len(decoded)
            decoded.extend(group[:remaining])
        if len(decoded) >= IMU_TARGET or len(decoded) >= IMU_MAX_BYTES:
            break
    return bytes(decoded) if len(decoded) >= IMU_GROUP else b""


def decode_vertical_band(
    frame: np.ndarray,
    frame_idx: int,
    clock: Optional[DeviceClock] = None,
) -> DecodedImuFrame:
    payload = decode_left_side_payload_bytes(frame)
    if len(payload) < IMU_GROUP:
        payload = decode_vertical_payload_bytes(frame)
    if len(payload) < IMU_GROUP:
        raise ValueError("IMU payload 未从竖向码带中解出")
    return parse_payload(payload, frame_idx=frame_idx, clock=clock)
