"""実機画像の点灯判定。SDK・カメラ配信・Yadaの押下設定には依存しない。"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LampFrame:
    image: np.ndarray
    bbox: tuple[float, float, float, float]
    captured_at: float
    frame_id: int


def lamp_level(image, bbox):
    """ボタン枠の1.15倍の楕円内で、平均RGB値の90パーセンタイルを求める。

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
    left, top = max(0, int(np.floor(cx-1.15*rx))), max(0, int(np.floor(cy-1.15*ry)))
    right, bottom = min(w, int(np.ceil(cx+1.15*rx))), min(h, int(np.ceil(cy+1.15*ry)))
    yy, xx = np.mgrid[top:bottom, left:right]
    inside = ((xx+0.5-cx)/rx)**2 + ((yy+0.5-cy)/ry)**2 <= 1.15**2
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
