"""Draw boxes, skeletons and gesture labels on a frame for the live view."""
import cv2
import numpy as np

from .observations import SKELETON, Detections

PERSON_COLOR = (80, 200, 80)    # BGR
GESTURE_COLOR = (0, 140, 255)
OBJECT_COLOR = (255, 160, 40)
MIN_KP_CONF = 0.5


def annotate(bgr: np.ndarray, detections: Detections,
             gestures: dict[int, frozenset[str]]) -> np.ndarray:
    image = bgr.copy()
    for person in detections.persons:
        active = gestures.get(person.track_id, frozenset()) if person.track_id is not None else frozenset()
        color = GESTURE_COLOR if active else PERSON_COLOR
        _box(image, person.box, color, _person_label(person.track_id, person.confidence, active))
        _skeleton(image, person.keypoints, person.keypoint_conf, color)
    for obj in detections.objects:
        _box(image, obj.box, OBJECT_COLOR, f"{obj.label} {obj.confidence:.2f}")
    return image


def _person_label(track_id: int | None, confidence: float, active: frozenset[str]) -> str:
    name = f"person #{track_id}" if track_id is not None else "person"
    return f"{name} {confidence:.2f}" + (f" [{', '.join(sorted(active))}]" if active else "")


def _box(image: np.ndarray, box, color, label: str) -> None:
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(0, y1 - h - 6)
    cv2.rectangle(image, (x1, top), (x1 + w + 6, top + h + 6), color, -1)
    cv2.putText(image, label, (x1 + 3, top + h + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (20, 20, 20), 1, cv2.LINE_AA)


def _skeleton(image: np.ndarray, keypoints: np.ndarray, conf: np.ndarray, color) -> None:
    for a, b in SKELETON:
        if conf[a] >= MIN_KP_CONF and conf[b] >= MIN_KP_CONF:
            pa = tuple(int(v) for v in keypoints[a])
            pb = tuple(int(v) for v in keypoints[b])
            cv2.line(image, pa, pb, color, 2, cv2.LINE_AA)
    for (x, y), c in zip(keypoints, conf, strict=True):
        if c >= MIN_KP_CONF:
            cv2.circle(image, (int(x), int(y)), 3, color, -1, cv2.LINE_AA)


def to_jpeg(bgr: np.ndarray, quality: int = 80) -> bytes:
    ok, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buffer.tobytes()
