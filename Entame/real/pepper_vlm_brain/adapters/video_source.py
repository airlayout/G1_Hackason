"""Local video as a deterministic sequence of individual images, never VLM video input."""
from dataclasses import dataclass
import math
from pathlib import Path
import cv2
from PIL import Image


class VideoError(RuntimeError):
    pass


def validate_timestamps(timestamps):
    values = [float(t) for t in timestamps]
    if len(values) > 10000:
        raise ValueError("At most 10000 selected frames per run")
    if not values or any(not math.isfinite(t) or t < 0 for t in values):
        raise ValueError("Timestamps must be finite, nonnegative and nonempty")
    if any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError("Timestamps must be strictly increasing, without duplicates")
    return values


def interval_timestamps(duration, interval):
    if not math.isfinite(interval) or interval <= 0:
        raise ValueError("Interval must be finite and >0")
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Video must have a known positive duration")
    ratio = duration / interval
    if not math.isfinite(ratio) or ratio > 10000:
        raise ValueError("At most 10000 selected frames per run")
    count = math.ceil(ratio)
    return [i * interval for i in range(count) if i * interval < duration]


@dataclass
class VideoFrame:
    image: Image.Image
    requested_timestamp: float
    actual_timestamp: float
    frame_index: int


class VideoSource:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.capture = None
        self.metadata = {}

    def open(self):
        if not self.path.is_file():
            raise VideoError(f"Video file does not exist: {self.path}")
        self.capture = cv2.VideoCapture(str(self.path))
        if not self.capture.isOpened():
            self.close()
            raise VideoError(f"Cannot open local video: {self.path}")
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        count = self.capture.get(cv2.CAP_PROP_FRAME_COUNT)
        if not math.isfinite(fps) or fps <= 0 or not math.isfinite(count) or count < 1:
            self.close()
            raise VideoError("Video has invalid FPS/frame-count metadata")
        self.metadata = {"path": str(self.path), "fps": fps, "frame_count": int(count),
                         "duration_s": count / fps,
                         "source_size": [int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                                         int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))],
                         "backend": self.capture.getBackendName(),
                         "orientation_degrees": self.capture.get(cv2.CAP_PROP_ORIENTATION_META)}
        return self

    def frame_at(self, timestamp):
        timestamp = validate_timestamps([timestamp])[0]
        if self.capture is None:
            raise VideoError("Video is not open")
        if timestamp >= self.metadata["duration_s"]:
            raise VideoError(f"Timestamp {timestamp}s is outside video duration")
        if not self.capture.set(cv2.CAP_PROP_POS_MSEC, timestamp * 1000):
            raise VideoError(f"Timestamp seek failed at {timestamp}s")
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise VideoError(f"Frame extraction failed at {timestamp}s")
        actual = self.capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
        index = round(self.capture.get(cv2.CAP_PROP_POS_FRAMES)) - 1
        # Reject misleading seeks instead of silently evaluating the wrong scene.
        if not math.isfinite(actual) or index < 0 or abs(actual - timestamp) > 1.5 / self.metadata["fps"]:
            raise VideoError(f"Seek mismatch: requested {timestamp}s, decoded {actual}s, index {index}")
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        return VideoFrame(image, timestamp, actual, index)

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.close()
