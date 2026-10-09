from pathlib import Path

import numpy as np
import pytest

from pepperapp.observations import Detections, Person

SAMPLE_IMAGE = (Path(__file__).resolve().parents[2]
                / "pepper_vlm_brain" / "scenarios" / "images" / "qwen-demo.jpg")


def make_person(points: dict[int, tuple[float, float]], *, track_id: int | None = 1,
                box=(150.0, 50.0, 350.0, 450.0), conf: float = 0.9) -> Person:
    """Person whose listed keypoints are visible (conf 0.9) and the rest hidden (conf 0)."""
    keypoints = np.zeros((17, 2))
    keypoint_conf = np.zeros(17)
    for index, (x, y) in points.items():
        keypoints[index] = (x, y)
        keypoint_conf[index] = conf
    return Person(track_id, box, 0.9, keypoints, keypoint_conf)


# Upright person facing the camera; image left = person's right side.
BASE_POSE = {0: (250, 100), 5: (280, 160), 6: (220, 160), 7: (300, 220), 8: (200, 220),
             9: (305, 280), 10: (195, 280), 11: (270, 300), 12: (230, 300)}


def pose(**changes) -> dict[int, tuple[float, float]]:
    """BASE_POSE with keypoints replaced, e.g. pose(**{"9": (300, 90)})."""
    return {**BASE_POSE, **{int(k): v for k, v in changes.items()}}


def frame_of(*persons: Person, objects=(), width: int = 640, height: int = 480) -> Detections:
    return Detections(width, height, tuple(persons), tuple(objects))


@pytest.fixture
def sample_image() -> Path:
    if not SAMPLE_IMAGE.is_file():
        pytest.skip("sample image is not in this checkout")
    return SAMPLE_IMAGE
