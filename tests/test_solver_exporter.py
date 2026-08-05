from pathlib import Path

import cv2
import numpy as np
import pytest

from stereo_calibrator.detector import pose_features
from stereo_calibrator.exporter import export_result
from stereo_calibrator.models import AcceptedPair
from stereo_calibrator.solver import solve_stereo


@pytest.fixture(scope="module")
def synthetic_pairs(tmp_path_factory):
    root = tmp_path_factory.mktemp("synthetic_stereo")
    width, height = 640, 480
    camera = np.array([[820.0, 0.0, 320.0], [0.0, 815.0, 240.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros(5)
    translation = np.array([[-0.080], [0.0], [0.0]])
    object_points = np.zeros((9 * 6, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.020
    rng = np.random.default_rng(42)
    pairs = []
    for index in range(24):
        rvec = np.array(
            [rng.uniform(-0.28, 0.28), rng.uniform(-0.35, 0.35), rng.uniform(-0.18, 0.18)]
        )
        tvec = np.array(
            [[rng.uniform(-0.08, 0.05)], [rng.uniform(-0.06, 0.04)], [rng.uniform(0.48, 0.90)]]
        )
        left, _ = cv2.projectPoints(object_points, rvec, tvec, camera, distortion)
        rotation, _ = cv2.Rodrigues(rvec)
        right_rvec, _ = cv2.Rodrigues(rotation)
        right, _ = cv2.projectPoints(object_points, right_rvec, tvec + translation, camera, distortion)
        left = left.reshape(-1, 2) + rng.normal(0, 0.10, (54, 2))
        right = right.reshape(-1, 2) + rng.normal(0, 0.10, (54, 2))
        left_path = root / f"{index:04d}_left.png"
        right_path = root / f"{index:04d}_right.png"
        image = np.full((height, width, 3), 80 + index, np.uint8)
        cv2.imwrite(str(left_path), image)
        cv2.imwrite(str(right_path), image)
        pairs.append(
            AcceptedPair(
                index=index,
                left_path=left_path,
                right_path=right_path,
                raw_path=None,
                left_corners=left.astype(np.float32),
                right_corners=right.astype(np.float32),
                image_size=(width, height),
                features=pose_features(left.astype(np.float32), (width, height)),
            )
        )
    return pairs


@pytest.fixture(scope="module")
def solved_result(synthetic_pairs):
    thresholds = {
        "maximum_mono_rms": 1.0,
        "maximum_epipolar_median": 0.7,
        "maximum_epipolar_p95": 1.5,
        "maximum_outlier_fraction": 0.2,
    }
    return solve_stereo(synthetic_pairs, (9, 6), 0.020, thresholds)


def test_solver_recovers_finite_stereo_matrices(solved_result):
    assert solved_result.left_camera_matrix.shape == (3, 3)
    assert solved_result.translation.shape == (3, 1)
    assert np.isfinite(solved_result.translation).all()
    assert abs(np.linalg.norm(solved_result.translation) - 0.080) < 0.012
    assert solved_result.valid_pair_count >= 20
    assert solved_result.passed


def test_exports_have_stable_files(tmp_path, solved_result, synthetic_pairs):
    export_result(solved_result, tmp_path, synthetic_pairs[0], square_size_m=0.020)

    expected = {
        "stereo_calibration.yaml",
        "stereo_calibration.json",
        "stereo_calibration.npz",
        "rectify_maps.yml.gz",
        "rectify_maps.npz",
        "rectification_preview.png",
        "quality_report.json",
    }
    assert expected.issubset({path.name for path in tmp_path.iterdir()})
    archive = np.load(tmp_path / "stereo_calibration.npz")
    assert archive["left_camera_matrix"].shape == (3, 3)
    assert archive["image_size"].tolist() == [640, 480]


def test_yaml_is_readable_by_opencv(tmp_path, solved_result, synthetic_pairs):
    export_result(solved_result, tmp_path, synthetic_pairs[0], square_size_m=0.020)

    storage = cv2.FileStorage(str(tmp_path / "stereo_calibration.yaml"), cv2.FILE_STORAGE_READ)
    assert storage.isOpened()
    matrix = storage.getNode("left_camera_matrix").mat()
    storage.release()
    assert matrix.shape == (3, 3)
