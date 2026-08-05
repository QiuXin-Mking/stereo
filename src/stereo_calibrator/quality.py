from __future__ import annotations

from typing import Iterable, Mapping, Optional

import cv2
import numpy as np

from .detector import pose_features
from .models import PoseFeatures, QualityDecision


def _sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _clipped_ratio(gray: np.ndarray) -> float:
    clipped = np.count_nonzero((gray <= 5) | (gray >= 250))
    return float(clipped) / float(gray.size)


def _edge_margin(corners: np.ndarray, width: int, height: int) -> float:
    points = np.asarray(corners).reshape(-1, 2)
    distances = np.column_stack(
        [points[:, 0], width - 1 - points[:, 0], points[:, 1], height - 1 - points[:, 1]]
    )
    return float(distances.min()) / float(min(width, height))


def _novelty(feature: PoseFeatures, history: Iterable[PoseFeatures]) -> float:
    previous = list(history)
    if not previous:
        return float("inf")
    current = feature.as_array()
    weights = np.asarray([1.0, 1.0, 2.5, 0.35, 0.25], dtype=np.float64)
    return min(float(np.linalg.norm((current - item.as_array()) * weights)) for item in previous)


def evaluate_pair(
    left_gray: np.ndarray,
    right_gray: np.ndarray,
    left_corners: Optional[np.ndarray],
    right_corners: Optional[np.ndarray],
    history: Iterable[PoseFeatures],
    thresholds: Mapping[str, float],
) -> QualityDecision:
    """Apply deterministic quality gates and return one operator-facing reason."""
    if left_corners is None or right_corners is None:
        return QualityDecision(False, "左右眼未同时完整检测到棋盘")
    if left_gray.shape != right_gray.shape:
        return QualityDecision(False, "左右眼图像尺寸不一致")

    height, width = left_gray.shape[:2]
    feature = pose_features(left_corners, (width, height))
    margin = min(
        _edge_margin(left_corners, width, height),
        _edge_margin(right_corners, width, height),
    )
    exposure = max(_clipped_ratio(left_gray), _clipped_ratio(right_gray))
    sharpness = min(_sharpness(left_gray), _sharpness(right_gray))
    novelty = _novelty(feature, history)
    metrics = {
        "edge_margin": margin,
        "clipped_ratio": exposure,
        "sharpness": sharpness,
        "novelty": novelty,
    }

    if margin < float(thresholds["minimum_edge_margin_ratio"]):
        return QualityDecision(False, "棋盘距离画面边缘太近", metrics, feature)
    if exposure > float(thresholds["maximum_clipped_ratio"]):
        return QualityDecision(False, "曝光异常", metrics, feature)
    if sharpness < float(thresholds["minimum_sharpness"]):
        return QualityDecision(False, "图像模糊", metrics, feature)
    if novelty < float(thresholds["minimum_novelty"]):
        return QualityDecision(False, "与已有样本重复", metrics, feature)
    return QualityDecision(True, "质量通过", metrics, feature)

