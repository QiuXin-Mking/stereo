from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class PoseFeatures:
    center_x: float
    center_y: float
    area_ratio: float
    angle_deg: float = 0.0
    perspective: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.asarray(
            [self.center_x, self.center_y, self.area_ratio, self.angle_deg / 90.0, self.perspective],
            dtype=np.float64,
        )


@dataclass
class QualityDecision:
    accepted: bool
    reason: str
    metrics: Dict[str, float] = field(default_factory=dict)
    features: Optional[PoseFeatures] = None


@dataclass
class AcceptedPair:
    index: int
    left_path: Path
    right_path: Path
    raw_path: Optional[Path]
    left_corners: np.ndarray
    right_corners: np.ndarray
    image_size: Tuple[int, int]
    features: PoseFeatures
    metrics: Dict[str, float] = field(default_factory=dict)


@dataclass
class CalibrationResult:
    image_size: Tuple[int, int]
    left_camera_matrix: np.ndarray
    left_distortion: np.ndarray
    right_camera_matrix: np.ndarray
    right_distortion: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    essential: np.ndarray
    fundamental: np.ndarray
    rectification_left: np.ndarray
    rectification_right: np.ndarray
    projection_left: np.ndarray
    projection_right: np.ndarray
    disparity_to_depth: np.ndarray
    left_map1: np.ndarray
    left_map2: np.ndarray
    right_map1: np.ndarray
    right_map2: np.ndarray
    mono_rms_left: float
    mono_rms_right: float
    stereo_rms: float
    epipolar_median: float
    epipolar_p95: float
    valid_pair_count: int
    rejected_indices: List[int]
    passed: bool
    failure_reasons: List[str] = field(default_factory=list)

