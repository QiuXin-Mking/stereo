#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu


def stamp_from_ns(timestamp_ns: int) -> rospy.Time:
    return rospy.Time(
        secs=int(timestamp_ns) // 1_000_000_000,
        nsecs=int(timestamp_ns) % 1_000_000_000,
    )


def image_message(path: Path, timestamp_ns: int, frame_id: str) -> Image:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"无法读取图像：{path}")
    message = Image()
    message.header.stamp = stamp_from_ns(timestamp_ns)
    message.header.frame_id = frame_id
    message.height, message.width = image.shape
    message.encoding = "mono8"
    message.is_bigendian = 0
    message.step = message.width
    message.data = image.tobytes()
    return message


def imu_message(row: dict[str, str]) -> Imu:
    message = Imu()
    message.header.stamp = stamp_from_ns(int(row["timestamp_ns"]))
    message.header.frame_id = "imu0"
    message.orientation_covariance[0] = -1.0
    message.angular_velocity.x = float(row["omega_x"])
    message.angular_velocity.y = float(row["omega_y"])
    message.angular_velocity.z = float(row["omega_z"])
    message.linear_acceleration.x = float(row["alpha_x"])
    message.linear_acceleration.y = float(row["alpha_y"])
    message.linear_acceleration.z = float(row["alpha_z"])
    return message


def load_events(dataset: Path):
    events = []
    with (dataset / "frames.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if not row["cam0_path"] or not row["cam1_path"]:
                continue
            timestamp_ns = int(row["image_timestamp_ns"])
            events.append((timestamp_ns, 0, "/cam0/image_raw", dataset / row["cam0_path"]))
            events.append((timestamp_ns, 1, "/cam1/image_raw", dataset / row["cam1_path"]))
    with (dataset / "imu0.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            events.append((int(row["timestamp_ns"]), 2, "/imu0", row))
    events.sort(key=lambda event: (event[0], event[1]))
    return events


def build_bag(dataset: Path, output: Path) -> dict[str, object]:
    counts = {"/cam0/image_raw": 0, "/cam1/image_raw": 0, "/imu0": 0}
    first_stamp = None
    last_stamp = None
    output.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(output), "w") as bag:
        for timestamp_ns, _order, topic, payload in load_events(dataset):
            if first_stamp is None:
                first_stamp = timestamp_ns
            if last_stamp is not None and timestamp_ns < last_stamp:
                raise ValueError("写 bag 时发现时间戳回退")
            last_stamp = timestamp_ns
            stamp = stamp_from_ns(timestamp_ns)
            if topic == "/cam0/image_raw":
                bag.write("/cam0/image_raw", image_message(payload, timestamp_ns, "cam0"), stamp)
            elif topic == "/cam1/image_raw":
                bag.write("/cam1/image_raw", image_message(payload, timestamp_ns, "cam1"), stamp)
            else:
                bag.write("/imu0", imu_message(payload), stamp)
            counts[topic] += 1
    duration = (
        (last_stamp - first_stamp) / 1_000_000_000.0
        if first_stamp is not None and last_stamp is not None
        else 0.0
    )
    if counts["/cam0/image_raw"] != counts["/cam1/image_raw"]:
        raise ValueError("ROS bag 左右图像数量不一致")
    return {"topics": counts, "duration_seconds": duration, "bag": str(output)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    info = build_bag(arguments.dataset, arguments.output)
    info_path = arguments.output.parent / "bag_info.json"
    info_path.write_text(json.dumps(info, indent=2), encoding="utf-8")
    print(json.dumps(info))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

