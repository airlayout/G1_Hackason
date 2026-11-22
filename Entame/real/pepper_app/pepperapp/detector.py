"""YOLO11n person pose (with ByteTrack IDs) and object detection on the CPU."""
import os
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np

from .observations import DetectedObject, Detections, Person, box_iou
from .weights import MODEL_DIR

# Keep Ultralytics from pip-installing at runtime and from writing to the home directory.
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
os.environ.setdefault("YOLO_CONFIG_DIR", str(MODEL_DIR.parent / "ultralytics"))


@dataclass(frozen=True)
class DetectorConfig:
    imgsz: int = 640
    confidence: float = 0.5
    object_labels: tuple[str, ...] = ("chair",)
    threads: int = 4
    device: str = "cpu"
    pose_weights: Path = MODEL_DIR / "yolo11n-pose.pt"
    detect_weights: Path = MODEL_DIR / "yolo11n.pt"
    # The pose model alone took a dog for a person (0.45 or 0.73 depending only on resizing).
    # Keep a pose person only if the detect model also sees a person there.
    confirm_persons: bool = True
    confirm_iou: float = 0.4

    def __post_init__(self):
        if self.imgsz not in (320, 480, 640):
            raise ValueError("imgsz must be 320, 480 or 640")
        if not 0.05 <= self.confidence <= 0.95:
            raise ValueError("confidence must be 0.05..0.95")
        if not 1 <= self.threads <= 16:
            raise ValueError("threads must be 1..16")


class YoloDetector:
    """Persons come from the pose model (keypoints + track IDs); other labels from the detect model."""

    def __init__(self, config: DetectorConfig):
        import torch
        from ultralytics import YOLO

        for weights in (config.pose_weights, config.detect_weights):
            if not Path(weights).is_file():
                raise FileNotFoundError(f"{weights} がありません。setup_ubuntu.sh か "
                                        "`python -m pepperapp.weights` で取得してください")
        torch.set_num_threads(config.threads)
        self.config = config
        self.pose = YOLO(str(config.pose_weights), task="pose")
        self.detect = YOLO(str(config.detect_weights), task="detect")
        names = self.detect.names
        unknown = [label for label in config.object_labels if label not in names.values()]
        if unknown:
            raise ValueError(f"YOLO が知らないラベル: {unknown}")
        wanted = set(config.object_labels) | ({"person"} if config.confirm_persons else set())
        self.class_ids = [i for i, name in names.items() if name in wanted]

    @property
    def labels(self) -> tuple[str, ...]:
        return tuple(self.detect.names.values())

    def __call__(self, bgr: np.ndarray) -> Detections:
        started = perf_counter()
        persons = self._persons(bgr)
        detected = self._objects(bgr) if self.class_ids else ()
        if self.config.confirm_persons:
            seen = [o.box for o in detected if o.label == "person"]
            persons = tuple(p for p in persons
                            if any(box_iou(p.box, box) >= self.config.confirm_iou for box in seen))
        objects = tuple(o for o in detected if o.label in self.config.object_labels)
        height, width = bgr.shape[:2]
        return Detections(width, height, persons, objects, (perf_counter() - started) * 1000)

    def _persons(self, bgr: np.ndarray) -> tuple[Person, ...]:
        c = self.config
        result = self.pose.track(bgr, persist=True, tracker="bytetrack.yaml", imgsz=c.imgsz,
                                 conf=c.confidence, device=c.device, verbose=False)[0]
        if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
            return ()
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        ids = result.boxes.id
        ids = [None] * len(boxes) if ids is None else [int(i) for i in ids.cpu().tolist()]
        xy = result.keypoints.xy.cpu().numpy()
        kp_conf = result.keypoints.conf
        kp_conf = np.ones(xy.shape[:2]) if kp_conf is None else kp_conf.cpu().numpy()
        return tuple(Person(ids[i], tuple(float(v) for v in boxes[i]), float(scores[i]),
                            xy[i].copy(), kp_conf[i].copy()) for i in range(len(boxes)))

    def _objects(self, bgr: np.ndarray) -> tuple[DetectedObject, ...]:
        c = self.config
        result = self.detect.predict(bgr, imgsz=c.imgsz, conf=c.confidence, device=c.device,
                                     classes=self.class_ids, verbose=False)[0]
        if result.boxes is None:
            return ()
        names = result.names
        return tuple(DetectedObject(names[int(cls)], tuple(float(v) for v in box), float(score))
                     for box, score, cls in zip(result.boxes.xyxy.cpu().tolist(),
                                                result.boxes.conf.cpu().tolist(),
                                                result.boxes.cls.cpu().tolist(), strict=False))
