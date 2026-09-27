#!/usr/bin/env python3
"""Probe a live Kilen/JHH02 frame using unified_capture's top-band decoder."""
from __future__ import annotations

import argparse
import struct
import time

import cv2
import numpy as np

GROUP = 16
VAL0, VAL1 = 50, 220
TARGET = 272


def decode_pair(a: np.ndarray, b: np.ndarray, code: int) -> bytes:
    if a.size < 5 or not np.all(a[:4] > VAL1):
        return b""
    sample = (code // 2) - 1
    while sample + 1 < a.size:
        p = np.array([a[sample], b[sample], a[sample + 1], b[sample + 1]], dtype=np.uint16)
        if int(p.sum()) < VAL1 * 4:
            break
        sample += code
    bits: list[int] = []
    while sample + 1 < a.size:
        p = np.array([a[sample], b[sample], a[sample + 1], b[sample + 1]], dtype=np.uint16)
        filtered = int(p.sum()) - int(p.max())
        if filtered < VAL0 * 3:
            bits.append(0)
        elif filtered < (int(a[:4].mean()) - 20) * 3:
            bits.append(1)
        else:
            break
        sample += code
    return bytes(sum(bits[i + j] << j for j in range(8)) for i in range(0, len(bits) - 7, 8))


def decode_top(frame: np.ndarray) -> tuple[bytes, int]:
    # unified_capture's probe operates on the first byte of each BGR pixel
    # (the blue/luma-equivalent byte in its decoded 24-bit frame), not OpenCV's
    # RGB grayscale conversion.  Preserve that exact channel for parity.
    y = frame[:, :, 0]
    probe = y[2, :16]
    code = 8 if int(np.count_nonzero(probe > VAL1)) > 8 else 4
    out = bytearray()
    for row in range(4 if code == 8 else 2, y.shape[0], code):
        chunk = decode_pair(y[row], y[row - 1], code)
        out.extend(chunk)
        if len(out) >= TARGET:
            break
    return bytes(out[:384]), code


def groups(payload: bytes) -> tuple[str, int, int, int]:
    header = payload[:GROUP]
    if len(header) < GROUP:
        return header.hex(), 0, 0, 0
    # Legacy JHH02 header: 11 ICM groups. New packed headers are also shown.
    if header[:8] == header[8:16]:
        declared = [(1, 11)]
    else:
        declared = [(((header[0] & 15) << 4) | (header[1] >> 4), header[1] & 15),
                    (header[2], header[3] >> 4),
                    (((header[3] & 15) << 4) | (header[4] >> 4), header[4] & 15),
                    (header[5], header[6] >> 4),
                    (((header[6] & 15) << 4) | (header[7] >> 4), header[7] & 15)]
    cursor, imu, nonzero = 1, 0, 0
    for typ, count in declared:
        if typ in (1, 4):
            for i in range(count):
                chunk = payload[(cursor + i) * GROUP:(cursor + i + 1) * GROUP]
                if len(chunk) == GROUP:
                    imu += 1
                    nonzero += int(any(chunk))
        cursor += count if typ in (1, 2, 4) else 0
    timestamp = int.from_bytes(payload[8:12], "big") if len(payload) >= 12 else 0
    return header.hex(), imu, nonzero, timestamp


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video4")
    ap.add_argument("--frames", type=int, default=30)
    args = ap.parse_args()
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    print("frame,bytes,code,imu_groups,nonzero_imu,header,exp_start_us")
    try:
        for index in range(args.frames):
            ok, frame = cap.read()
            if not ok:
                print(f"{index},READ_FAIL")
                continue
            payload, code = decode_top(frame)
            header, imu, nonzero, timestamp = groups(payload)
            print(f"{index},{len(payload)},{code},{imu},{nonzero},{header},{timestamp}", flush=True)
    finally:
        cap.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
