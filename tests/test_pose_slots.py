import cv2
import numpy as np

from stereo_calibrator.pose_slots import POSE_SLOT_QUOTAS, classify_pose_slot


def grid(center=(960, 600), size=(500, 300), angle=0.0):
    xs = np.linspace(-size[0] / 2, size[0] / 2, 8)
    ys = np.linspace(-size[1] / 2, size[1] / 2, 5)
    points = np.asarray([(x, y) for y in ys for x in xs], np.float32)
    matrix = cv2.getRotationMatrix2D((0, 0), angle, 1.0)
    points = cv2.transform(points.reshape(1, -1, 2), matrix).reshape(-1, 2)
    return points + np.asarray(center, np.float32)


def test_pose_slot_quotas_total_thirty_two():
    assert sum(POSE_SLOT_QUOTAS.values()) == 32


def test_classifies_center_and_left_position():
    assert classify_pose_slot(grid(), (8, 5), (1920, 1200), {}) == "center_front"
    assert classify_pose_slot(grid(center=(350, 600)), (8, 5), (1920, 1200), {}) == "left"


def test_classifies_roll_and_distance_before_position():
    assert classify_pose_slot(grid(angle=18), (8, 5), (1920, 1200), {}) == "roll_ccw"
    assert classify_pose_slot(grid(size=(1000, 650)), (8, 5), (1920, 1200), {}) == "near"
    assert classify_pose_slot(grid(size=(220, 130)), (8, 5), (1920, 1200), {}) == "far"


def test_skips_a_filled_slot_and_uses_next_matching_candidate():
    filled = {"center_front": POSE_SLOT_QUOTAS["center_front"]}

    assert classify_pose_slot(grid(), (8, 5), (1920, 1200), filled) is None
