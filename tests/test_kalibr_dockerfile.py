from pathlib import Path


def test_runtime_image_is_multistage_ros_core_not_desktop():
    text = Path("kalibr_runtime/Dockerfile.arm64").read_text()

    assert text.count("FROM ") >= 2
    assert "ros:noetic-ros-core-focal" in text
    assert "desktop-full" not in text
    assert "rviz" not in text
    assert "gazebo" not in text
    assert "catkin build" in text
    assert "--install" in text
    assert "build-essential" in text
    assert "USER kalibr" in text
    assert "KALIBR_COMMIT=1f60227442d25e36365ef5f72cd80b9666d73467" in text


def test_runtime_contains_only_offline_ros_components():
    text = Path("kalibr_runtime/Dockerfile.arm64").read_text()
    runtime = text.split("FROM ros:noetic-ros-core-focal AS runtime", 1)[1]

    assert "ros-noetic-rosbag" in text
    assert "ros-noetic-sensor-msgs" in text
    assert "ros-noetic-cv-bridge" in text
    assert "python3-opencv" in runtime
    assert "COPY --from=builder /opt/ros/noetic/lib/python3/dist-packages/cv_bridge" in runtime
    assert "ros-noetic-image-transport" not in runtime
    assert "python3-tk" not in runtime
    assert "/usr/local/bin/kalibr_calibrate_cameras" in runtime
    assert "/usr/local/bin/kalibr_calibrate_imu_camera" in runtime
    assert "run_pipeline.sh" in text
    assert "ENTRYPOINT" in text


def test_build_script_enforces_arm64_help_and_two_gibibyte_limit():
    text = Path("kalibr_runtime/build-arm64.sh").read_text()

    assert "--platform linux/arm64" in text
    assert "2147483648" in text
    assert "verify_help kalibr_calibrate_cameras" in text
    assert "verify_help kalibr_calibrate_imu_camera" in text
    assert "docker history" in text
