"""Immutable detection results shared by the detector, gestures, events and the UI."""
from dataclasses import dataclass

import numpy as np

# COCO 17 keypoints used by YOLO pose models.
NOSE = 0
L_SHOULDER, R_SHOULDER = 5, 6
L_ELBOW, R_ELBOW = 7, 8
L_WRIST, R_WRIST = 9, 10
SKELETON = ((5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6))

Box = tuple[float, float, float, float]  # x1, y1, x2, y2 in pixels


@dataclass(frozen=True)
class Target:
    """Point to look at, normalized to the image (0..1, origin top-left)."""
    x: float
    y: float


@dataclass(frozen=True)
class Person:
    track_id: int | None
    box: Box
    confidence: float
    keypoints: np.ndarray  # (17, 2) pixels
    keypoint_conf: np.ndarray  # (17,)

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.box
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


@dataclass(frozen=True)
class DetectedObject:
    label: str
    box: Box
    confidence: float

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.box
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


@dataclass(frozen=True)
class Detections:
    width: int
    height: int
    persons: tuple[Person, ...]
    objects: tuple[DetectedObject, ...]
    inference_ms: float = 0.0


def box_iou(a: Box, b: Box) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def box_center(box: Box, width: int, height: int) -> Target:
    x1, y1, x2, y2 = box
    return Target((x1 + x2) / 2 / width, (y1 + y2) / 2 / height)


def person_target(person: Person, width: int, height: int, min_conf: float = 0.5) -> Target:
    """Face if the nose is visible, otherwise the upper part of the body box."""
    if person.keypoint_conf[NOSE] >= min_conf:
        x, y = person.keypoints[NOSE]
        return Target(float(x) / width, float(y) / height)
    x1, y1, x2, y2 = person.box
    return Target((x1 + x2) / 2 / width, (y1 + 0.15 * (y2 - y1)) / height)
