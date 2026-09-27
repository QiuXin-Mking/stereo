#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Any

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu


APRILGRID_DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
APRILGRID_IDS = set(range(48))
MIN_TAGS_PER_EYE = 6


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


def _tag_detector() -> Any:
    """Create the same AprilTag dictionary used by dataset validation."""
    dictionary = cv2.aruco.getPredefinedDictionary(APRILGRID_DICTIONARY)
    return cv2.aruco.ArucoDetector(dictionary)


def detect_aprilgrid_tags(path: Path, detector: Any | None = None) -> int:
    """Return the number of valid 36h11 tags visible in one eye image.

    The stored eye images are 1920x1200 and tags can be small.  A 1.5x retry
    mirrors the online validation path and avoids accepting a frame merely
    because the first detector pass missed small tags.
    """
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return 0
    detector = detector or _tag_detector()
    _corners, ids, _ = detector.detectMarkers(image)
    count = sum(1 for marker_id in (ids.reshape(-1) if ids is not None else ())
                if int(marker_id) in APRILGRID_IDS)
    if count < MIN_TAGS_PER_EYE:
        scaled = cv2.resize(image, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
        _corners, ids, _ = detector.detectMarkers(scaled)
        count = sum(1 for marker_id in (ids.reshape(-1) if ids is not None else ())
                    if int(marker_id) in APRILGRID_IDS)
    return count


def _unique_filtered_dir(dataset: Path) -> Path:
    candidate = dataset.parent / f"{dataset.name}_filtered"
    if not candidate.exists():
        return candidate
    index = 2
    while True:
        candidate = dataset.parent / f"{dataset.name}_filtered_{index}"
        if not candidate.exists():
            return candidate
        index += 1


def prepare_filtered_dataset(
    dataset: Path,
    *,
    min_tags_per_eye: int = MIN_TAGS_PER_EYE,
    filtered_dir: Path | None = None,
) -> tuple[Path, dict[str, object]]:
    """Create a traceable, read-only-source filtered dataset for Kalibr.

    Every frame in the generated ``frames.csv`` has at least six valid
    AprilGrid tags in both eyes.  The source directory is never modified.
    """
    source = Path(dataset)
    output = Path(filtered_dir) if filtered_dir is not None else _unique_filtered_dir(source)
    if output.exists():
        raise FileExistsError(f"过滤数据目录已存在：{output}")
    output.mkdir(parents=True)
    (output / "cam0").mkdir()
    (output / "cam1").mkdir()

    detector = _tag_detector()
    kept_rows: list[dict[str, str]] = []
    dropped: list[dict[str, object]] = []
    observations: list[dict[str, object]] = []
    with (source / "frames.csv").open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        required = {"frame_idx", "image_timestamp_ns", "cam0_path", "cam1_path"}
        if not required.issubset(fieldnames):
            raise ValueError("frames.csv 缺少图像引用字段")
        for row in reader:
            cam0 = source / row["cam0_path"]
            cam1 = source / row["cam1_path"]
            left_count = detect_aprilgrid_tags(cam0, detector)
            right_count = detect_aprilgrid_tags(cam1, detector)
            reasons = []
            if left_count < min_tags_per_eye:
                reasons.append(f"cam0 AprilGrid Tag 少于 {min_tags_per_eye}")
            if right_count < min_tags_per_eye:
                reasons.append(f"cam1 AprilGrid Tag 少于 {min_tags_per_eye}")
            item = {
                "frame_idx": int(row.get("frame_idx", len(kept_rows) + len(dropped))),
                "image_timestamp_ns": int(row["image_timestamp_ns"]),
                "cam0_tags": left_count,
                "cam1_tags": right_count,
            }
            observations.append(item.copy())
            if reasons:
                item["reasons"] = reasons
                dropped.append(item)
                continue
            kept_rows.append(row)
            shutil.copy2(cam0, output / row["cam0_path"])
            shutil.copy2(cam1, output / row["cam1_path"])

    # Copy the IMU and calibration metadata without changing the source.
    for name in ("imu0.csv", "target.yaml", "imu.yaml", "capture.json",
                 "decoder_stats.json", "dataset_manifest.json", "integrity.json"):
        path = source / name
        if path.is_file():
            shutil.copy2(path, output / name)
    with (output / "frames.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(row for row in kept_rows)

    report: dict[str, object] = {
        "schema_version": 1,
        "source_dataset": str(source),
        "filtered_dataset": str(output),
        "dictionary": "DICT_APRILTAG_36h11",
        "min_tags_per_eye": min_tags_per_eye,
        "source_pairs": len(kept_rows) + len(dropped),
        "kept_pairs": len(kept_rows),
        "dropped_pairs": len(dropped),
        "dropped": dropped,
        "observations": observations,
    }
    for eye in ("cam0", "cam1"):
        values = sorted(int(item[f"{eye}_tags"]) for item in observations)
        report[f"min_{eye}_tags"] = values[0] if values else 0
        report[f"p50_{eye}_tags"] = values[len(values) // 2] if values else 0
    manifest_path = output / "dataset_manifest.json"
    manifest: dict[str, object] = {}
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            manifest = {}
    manifest.update({
        "source_dataset": str(source),
        "filtered_dataset": str(output),
        "filter_report": str(output / "filter_report.json"),
        "filter_rule": "DICT_APRILTAG_36h11; both eyes >= 6 tags",
    })
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output / "filter_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return output, report


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
    filtered_dataset, filter_report = prepare_filtered_dataset(dataset)
    if int(filter_report["kept_pairs"]) == 0:
        raise ValueError("AprilGrid 预检后没有满足每眼至少 6 个 Tag 的双目观测")
    counts = {"/cam0/image_raw": 0, "/cam1/image_raw": 0, "/imu0": 0}
    first_stamp = None
    last_stamp = None
    output.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(output), "w") as bag:
        for timestamp_ns, _order, topic, payload in load_events(filtered_dataset):
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
    return {
        "topics": counts,
        "duration_seconds": duration,
        "bag": str(output),
        "filtered_dataset": str(filtered_dataset),
        "filter_report": str(filtered_dataset / "filter_report.json"),
        "filter": filter_report,
    }


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
