"""実機画像の点灯判定。SDK・カメラ配信・Yadaの押下設定には依存しない。"""

from dataclasses import dataclass
import threading

import numpy as np


@dataclass(frozen=True)
class LampFrame:
    image: np.ndarray
    bbox: tuple[float, float, float, float]
    captured_at: float
    frame_id: int


@dataclass(frozen=True)
class LampObservation:
    captured_at: float
    frame_id: int
    lamp: float


class ImageLampTracker:
    """Monitor the selected button in RGB even when depth estimation fails."""

    def __init__(self):
        self.lock = threading.Lock()
        self.bbox = None
        self.value = None

    def seed(self, bbox):
        with self.lock:
            self.bbox = tuple(map(float, bbox))

    def observe(self, image, detections, captured_at, frame_id):
        with self.lock:
            box = self.bbox
        if box is None:
            return
        nearby = []
        for detection in detections:
            candidate = np.asarray(detection.bbox, dtype=float)
            if candidate.shape != (4,) or not np.all(np.isfinite(candidate)) or np.any(candidate[2:] <= candidate[:2]):
                continue
            intersection = np.maximum(0, np.minimum(candidate[2:], box[2:])-np.maximum(candidate[:2], box[:2]))
            area = np.prod(intersection)
            union = np.prod(candidate[2:]-candidate[:2])+np.prod(np.asarray(box[2:])-box[:2])-area
            if union > 0 and area/union >= .3:
                nearby.append((area/union, tuple(candidate)))
        if not nearby:
            return
        box = max(nearby, key=lambda value:value[0])[1]
        try:
            value = LampObservation(float(captured_at), int(frame_id), lamp_level(image, box, ellipse_scale=1.0))
        except ValueError:
            return
        with self.lock:
            self.bbox, self.value = box, value

    def latest(self):
        with self.lock:
            return self.value


def lamp_level(image, bbox, *, ellipse_scale=1.15):
    """指定倍率の楕円内で、平均RGB値の90パーセンタイルを求める。

    BGRでも値は同じ。細い縁だけが点灯する実物ボタンを対象にする。
    """
    image = np.asarray(image)
    box = np.asarray(bbox, dtype=float)
    if (image.ndim != 3 or image.shape[2] != 3 or not np.all(np.isfinite(image))
            or box.shape != (4,) or not np.all(np.isfinite(box))):
        raise ValueError("点灯確認の画像または検出枠が不正です")
    x1, y1, x2, y2 = box
    h, w = image.shape[:2]
    if x1 < 0 or y1 < 0 or x2 > w or y2 > h or min(x2-x1, y2-y1) < 6:
        raise ValueError("点灯確認の検出枠が画像外または小さすぎます")
    cx, cy, rx, ry = (x1+x2)/2, (y1+y2)/2, (x2-x1)/2, (y2-y1)/2
    if not np.isfinite(ellipse_scale) or ellipse_scale <= 0:
        raise ValueError('Invalid ellipse scale')
    left, top = max(0, int(np.floor(cx-ellipse_scale*rx))), max(0, int(np.floor(cy-ellipse_scale*ry)))
    right, bottom = min(w, int(np.ceil(cx+ellipse_scale*rx))), min(h, int(np.ceil(cy+ellipse_scale*ry)))
    yy, xx = np.mgrid[top:bottom, left:right]
    inside = ((xx+0.5-cx)/rx)**2 + ((yy+0.5-cy)/ry)**2 <= ellipse_scale**2
    gray = image[top:bottom, left:right].astype(float).mean(axis=2)
    return float(np.percentile(gray[inside], 90))


class LampCheck:
    """退避・静止後の異なる3フレームを使う。混在結果は判定不能。"""

    def __init__(self, baseline, after, min_rise=30.0):
        if (not np.all(np.isfinite([baseline, after, min_rise]))
                or not 0 < min_rise <= 255):
            raise ValueError("点灯判定のしきい値または時刻が不正です")
        self.baseline, self.after, self.min_rise = baseline, after, min_rise
        self.frames = []
        self.last_t = after
        self.last_id = -1

    @property
    def finished(self):
        return len(self.frames) == 3

    @property
    def result(self):
        if not self.finished:
            return None
        bright = [row["rise"] >= self.min_rise for row in self.frames]
        return True if all(bright) else False if not any(bright) else None

    def observe(self, frame, now):
        if self.finished:
            raise ValueError("点灯確認は3フレームで終了しています")
        if (not np.all(np.isfinite([frame.captured_at, now]))
                or not 0 <= now-frame.captured_at <= 0.5):
            raise ValueError("点灯確認の画像が古い、または撮影時刻が不正です")
        if frame.captured_at <= self.last_t or frame.frame_id <= self.last_id:
            return False
        level = lamp_level(frame.image, frame.bbox)
        self.last_t, self.last_id = frame.captured_at, frame.frame_id
        self.frames.append({"captured_at_s": float(frame.captured_at),
                            "frame_id": int(frame.frame_id), "bbox": [float(v) for v in frame.bbox],
                            "level": level, "rise": level-self.baseline})
        return True


class PressLampMonitor:
    """Detect a rise in a recent observation captured after entering the press."""

    def __init__(self, baseline, min_rise, after):
        if not np.all(np.isfinite([baseline, min_rise, after])) or min_rise <= 0:
            raise ValueError('Invalid lamp baseline, threshold or start time')
        self.baseline, self.min_rise, self.after = float(baseline), float(min_rise), float(after)
        self.last_id, self.last_stamp = -1, after
        self.frames = []

    def observe(self, observed, now):
        stamp, frame_id, level = observed.captured_at, observed.frame_id, observed.lamp
        if frame_id <= self.last_id or stamp <= self.last_stamp:
            return False
        self.last_id = frame_id
        self.last_stamp = stamp
        if (not np.isfinite(stamp) or not 0 <= now-stamp <= .5 or level is None
                or not np.isfinite(level) or level-self.baseline < self.min_rise):
            self.frames.clear()
            return False
        self.frames.append(dict(frame_id=int(frame_id), captured_at_s=float(stamp), level=float(level),
                                rise=float(level-self.baseline)))
        return True
