import json
import importlib.util
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml


SUMMARY_PATH = Path("kalibr_runtime/scripts/summarize.py").resolve()
SUMMARY_SPEC = importlib.util.spec_from_file_location("kalibr_summarize", SUMMARY_PATH)
SUMMARY_MODULE = importlib.util.module_from_spec(SUMMARY_SPEC)
assert SUMMARY_SPEC.loader is not None
SUMMARY_SPEC.loader.exec_module(SUMMARY_MODULE)
summarize = SUMMARY_MODULE.summarize


def test_pipeline_is_headless_and_uses_official_cli_names():
    text = Path("kalibr_runtime/scripts/run_pipeline.sh").read_text()

    assert all(topic in text for topic in (
        "/cam0/image_raw", "/cam1/image_raw", "/imu0"
    ))
    assert "kalibr_calibrate_cameras" in text
    assert "kalibr_calibrate_imu_camera" in text
    assert "--cams" in text
    assert "--dont-show-report" in text
    assert "roscore" not in text
    assert all(stage in text for stage in (
        "bagging", "camera_calibrating", "imu_calibrating", "pass"
    ))


def test_bag_builder_writes_three_topics_without_ros_master():
    text = Path("kalibr_runtime/scripts/build_bag.py").read_text()

    assert "rosbag.Bag" in text
    assert 'bag.write("/cam0/image_raw"' in text
    assert 'bag.write("/cam1/image_raw"' in text
    assert 'bag.write("/imu0"' in text
    assert "rospy.init_node" not in text


def test_aprilgrid_preflight_filters_bad_pairs_without_ros(tmp_path, monkeypatch):
    """The preflight is pure filesystem/OpenCV logic and is testable off-board."""
    rosbag_stub = types.ModuleType("rosbag")
    rosbag_stub.Bag = object
    rospy_stub = types.ModuleType("rospy")
    rospy_stub.Time = object
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (), {})
    sensor_msgs_msg.Imu = type("Imu", (), {})
    monkeypatch.setitem(sys.modules, "rosbag", rosbag_stub)
    monkeypatch.setitem(sys.modules, "rospy", rospy_stub)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs_msg)
    path = Path("kalibr_runtime/scripts/build_bag.py").resolve()
    spec = importlib.util.spec_from_file_location("kalibr_build_bag_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    source = tmp_path / "kalibr"
    (source / "cam0").mkdir(parents=True)
    (source / "cam1").mkdir()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    board = cv2.aruco.GridBoard((8, 6), 30, 9, dictionary)
    good = board.generateImage((640, 480), marginSize=8, borderBits=1)
    bad = np.zeros_like(good)
    for index, image in enumerate((good, bad)):
        cv2.imwrite(str(source / "cam0" / f"{index}.png"), image)
        cv2.imwrite(str(source / "cam1" / f"{index}.png"), image)
    (source / "frames.csv").write_text(
        "frame_idx,image_timestamp_ns,exp_start_ns,exp_end_ns,payload_bytes,imu_count,cam0_path,cam1_path\n"
        "0,100,0,0,0,0,cam0/0.png,cam1/0.png\n"
        "1,200,0,0,0,0,cam0/1.png,cam1/1.png\n",
        encoding="utf-8",
    )
    (source / "imu0.csv").write_text("timestamp_ns\n", encoding="utf-8")

    filtered, report = module.prepare_filtered_dataset(source)

    assert report["source_pairs"] == 2
    assert report["kept_pairs"] == 1
    assert report["dropped_pairs"] == 1
    assert (filtered / "cam0/0.png").is_file()
    assert not (filtered / "cam0/1.png").is_file()
    assert json.loads((filtered / "filter_report.json").read_text()) == report
    assert (source / "cam0/1.png").is_file()


def test_summary_normalizes_stereo_and_imu_transforms(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    camera_chain = {
        "cam0": {
            "camera_model": "pinhole",
            "intrinsics": [800.0, 801.0, 960.0, 600.0],
            "distortion_model": "radtan",
            "distortion_coeffs": [0.1, -0.2, 0.001, -0.001],
            "resolution": [1920, 1200],
        },
        "cam1": {
            "camera_model": "pinhole",
            "intrinsics": [802.0, 803.0, 961.0, 601.0],
            "distortion_model": "radtan",
            "distortion_coeffs": [0.11, -0.21, 0.002, -0.002],
            "resolution": [1920, 1200],
            "T_cn_cnm1": [
                [1.0, 0.0, 0.0, -0.095],
                [0.0, 1.0, 0.0, 0.001],
                [0.0, 0.0, 1.0, 0.002],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    }
    imu_chain = camera_chain | {
        "cam0": camera_chain["cam0"] | {
            "T_cam_imu": [
                [1.0, 0.0, 0.0, 0.01],
                [0.0, 1.0, 0.0, 0.02],
                [0.0, 0.0, 1.0, 0.03],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "timeshift_cam_imu": -0.00053,
        }
    }
    (results / "camchain.yaml").write_text(yaml.safe_dump(camera_chain))
    (results / "camchain-imucam.yaml").write_text(yaml.safe_dump(imu_chain))
    (results / "bag_info.json").write_text(json.dumps({"topics": {"/imu0": 20000}}))

    result = summarize(results, results / "summary.json")

    assert result["provisional_imu_noise"] is True
    assert result["cameras"]["cam0"]["K"] == [
        [800.0, 0.0, 960.0], [0.0, 801.0, 600.0], [0.0, 0.0, 1.0]
    ]
    assert result["stereo"]["T_cam1_cam0"][0][3] == -0.095
    assert result["stereo"]["baseline_m"] == pytest.approx(0.0950263, abs=1e-6)
    assert result["imu"]["T_cam0_imu"][2][3] == 0.03
    assert result["imu"]["time_offset_seconds"] == pytest.approx(-0.00053)
    assert json.loads((results / "summary.json").read_text()) == result
