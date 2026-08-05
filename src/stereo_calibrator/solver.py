from __future__ import annotations

from typing import List, Mapping, Sequence, Tuple

import cv2
import numpy as np

from .models import AcceptedPair, CalibrationResult


def _object_template(pattern: Tuple[int, int], square_size_m: float) -> np.ndarray:
    columns, rows = pattern
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points *= float(square_size_m)
    return points


def _mono_calibrate(
    pairs: Sequence[AcceptedPair], object_points: np.ndarray, side: str
) -> Tuple[float, np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray]]:
    objects = [object_points for _ in pairs]
    images = [getattr(pair, f"{side}_corners").reshape(-1, 1, 2) for pair in pairs]
    rms, camera, distortion, rvecs, tvecs = cv2.calibrateCamera(
        objects, images, pairs[0].image_size, None, None
    )
    return float(rms), camera, distortion, list(rvecs), list(tvecs)


def _reprojection_rms(
    object_points: np.ndarray,
    corners: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera: np.ndarray,
    distortion: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera, distortion)
    error = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(error * error, axis=1))))


def _solve_once(
    pairs: Sequence[AcceptedPair],
    object_points: np.ndarray,
) -> Tuple[CalibrationResult, List[float]]:
    image_size = pairs[0].image_size
    mono_left, camera_left, distortion_left, left_rvecs, left_tvecs = _mono_calibrate(
        pairs, object_points, "left"
    )
    mono_right, camera_right, distortion_right, right_rvecs, right_tvecs = _mono_calibrate(
        pairs, object_points, "right"
    )
    objects = [object_points for _ in pairs]
    left_points = [pair.left_corners.reshape(-1, 1, 2) for pair in pairs]
    right_points = [pair.right_corners.reshape(-1, 1, 2) for pair in pairs]
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    (
        stereo_rms,
        camera_left,
        distortion_left,
        camera_right,
        distortion_right,
        rotation,
        translation,
        essential,
        fundamental,
    ) = cv2.stereoCalibrate(
        objects,
        left_points,
        right_points,
        camera_left,
        distortion_left,
        camera_right,
        distortion_right,
        image_size,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    (
        rectification_left,
        rectification_right,
        projection_left,
        projection_right,
        disparity_to_depth,
        _,
        _,
    ) = cv2.stereoRectify(
        camera_left,
        distortion_left,
        camera_right,
        distortion_right,
        image_size,
        rotation,
        translation,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
    )
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        camera_left,
        distortion_left,
        rectification_left,
        projection_left,
        image_size,
        cv2.CV_16SC2,
    )
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        camera_right,
        distortion_right,
        rectification_right,
        projection_right,
        image_size,
        cv2.CV_16SC2,
    )

    vertical_errors: List[float] = []
    pair_scores: List[float] = []
    for index, pair in enumerate(pairs):
        rectified_left = cv2.undistortPoints(
            pair.left_corners.reshape(-1, 1, 2),
            camera_left,
            distortion_left,
            R=rectification_left,
            P=projection_left,
        ).reshape(-1, 2)
        rectified_right = cv2.undistortPoints(
            pair.right_corners.reshape(-1, 1, 2),
            camera_right,
            distortion_right,
            R=rectification_right,
            P=projection_right,
        ).reshape(-1, 2)
        epipolar = np.abs(rectified_left[:, 1] - rectified_right[:, 1])
        vertical_errors.extend(epipolar.tolist())
        left_error = _reprojection_rms(
            object_points,
            pair.left_corners,
            left_rvecs[index],
            left_tvecs[index],
            camera_left,
            distortion_left,
        )
        right_error = _reprojection_rms(
            object_points,
            pair.right_corners,
            right_rvecs[index],
            right_tvecs[index],
            camera_right,
            distortion_right,
        )
        pair_scores.append(max(left_error, right_error, float(np.percentile(epipolar, 95))))

    result = CalibrationResult(
        image_size=image_size,
        left_camera_matrix=camera_left,
        left_distortion=distortion_left,
        right_camera_matrix=camera_right,
        right_distortion=distortion_right,
        rotation=rotation,
        translation=translation.reshape(3, 1),
        essential=essential,
        fundamental=fundamental,
        rectification_left=rectification_left,
        rectification_right=rectification_right,
        projection_left=projection_left,
        projection_right=projection_right,
        disparity_to_depth=disparity_to_depth,
        left_map1=left_map1,
        left_map2=left_map2,
        right_map1=right_map1,
        right_map2=right_map2,
        mono_rms_left=mono_left,
        mono_rms_right=mono_right,
        stereo_rms=float(stereo_rms),
        epipolar_median=float(np.median(vertical_errors)),
        epipolar_p95=float(np.percentile(vertical_errors, 95)),
        valid_pair_count=len(pairs),
        rejected_indices=[],
        passed=False,
    )
    return result, pair_scores


def _finite_result(result: CalibrationResult) -> bool:
    matrices = (
        result.left_camera_matrix,
        result.left_distortion,
        result.right_camera_matrix,
        result.right_distortion,
        result.rotation,
        result.translation,
        result.rectification_left,
        result.rectification_right,
        result.projection_left,
        result.projection_right,
        result.disparity_to_depth,
    )
    return all(np.isfinite(matrix).all() for matrix in matrices)


def solve_stereo(
    pairs: Sequence[AcceptedPair],
    pattern: Tuple[int, int],
    square_size_m: float,
    thresholds: Mapping[str, float],
) -> CalibrationResult:
    """Solve stereo calibration, discard bounded robust outliers, and validate it."""
    if square_size_m <= 0:
        raise ValueError("square_size_m must be positive")
    if len(pairs) < 20:
        raise ValueError("at least 20 accepted pairs are required")
    image_sizes = {pair.image_size for pair in pairs}
    if len(image_sizes) != 1:
        raise ValueError("all calibration pairs must have the same image size")

    object_points = _object_template(pattern, square_size_m)
    active = list(pairs)
    rejected: List[int] = []
    maximum_rejected = int(len(active) * float(thresholds["maximum_outlier_fraction"]))
    while True:
        result, pair_scores = _solve_once(active, object_points)
        scores = np.asarray(pair_scores)
        median = float(np.median(scores))
        mad = float(np.median(np.abs(scores - median)))
        robust_limit = median + max(0.5, 4.5 * 1.4826 * mad)
        worst = int(np.argmax(scores))
        if (
            scores[worst] <= robust_limit
            or len(rejected) >= maximum_rejected
            or len(active) <= 20
        ):
            break
        rejected.append(active[worst].index)
        del active[worst]

    failures: List[str] = []
    if not _finite_result(result):
        failures.append("标定矩阵包含非有限数值")
    if result.mono_rms_left > float(thresholds["maximum_mono_rms"]):
        failures.append("左眼 RMS 超限")
    if result.mono_rms_right > float(thresholds["maximum_mono_rms"]):
        failures.append("右眼 RMS 超限")
    if result.epipolar_median > float(thresholds["maximum_epipolar_median"]):
        failures.append("极线误差中位数超限")
    if result.epipolar_p95 > float(thresholds["maximum_epipolar_p95"]):
        failures.append("极线误差 P95 超限")
    if not np.isfinite(np.linalg.norm(result.translation)) or np.linalg.norm(result.translation) <= 0:
        failures.append("估计基线无效")
    result.valid_pair_count = len(active)
    result.rejected_indices = rejected
    result.failure_reasons = failures
    result.passed = not failures
    return result

