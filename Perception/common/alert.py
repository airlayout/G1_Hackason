"""人を検出したときに「出来事」を出すための部品(判定・写真の保存・枠の描画)。

real/person_alert.py(G1のPC2で動かす)が使う。PC2 の GPU 版はシステムの Python 3.8 で動かすので、3.8 で読める書き方にしておく。
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np

from .detector.types import Detection


def draw_detections(img: np.ndarray, dets: list[Detection]) -> np.ndarray:
    """検出した枠と「クラス名 確からしさ」を描いた画像を返す(元の画像は変えない)。"""
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d.bbox)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, f"{d.class_name} {d.confidence:.2f}", (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1)
    return out


class EventJudge:
    """人が min_frames フレーム続けて写ったら出来事にする。出したあと cooldown_s 秒は出さない。

    人が写り続けている間は cooldown_s 秒ごとに出し直す(いる間の様子が一定の間隔で残る)。1 フレームでも人がいなければ数え直す。
    """

    def __init__(self, min_frames: int = 3, cooldown_s: float = 10.0) -> None:
        self.min_frames = min_frames
        self.cooldown_s = cooldown_s
        self._streak = 0
        self._last_fired: float | None = None

    def update(self, has_person: bool, now: float) -> bool:
        """1フレーム分の結果を入れる。出来事を出すときだけ True。"""
        self._streak = self._streak + 1 if has_person else 0
        if self._streak < self.min_frames:
            return False
        if self._last_fired is not None and now - self._last_fired < self.cooldown_s:
            return False
        self._last_fired = now
        return True


class AlertStore:
    """出来事の写真(枠を描いたJPEG)と記録(events.jsonl)を保存する。写真は max_photos 枚を超えたら古いものから消す。

    記録(events.jsonl)は消さない(1 件 300 B ほどなので)。消した写真を指す行も残る。
    """

    def __init__(self, out_dir: Path, max_photos: int = 1000) -> None:
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.max_photos = max_photos
        self._log = self.out_dir / "events.jsonl"

    def save(self, jpeg: bytes, record: dict) -> Path:
        """JPEGのバイト列と記録を保存し、写真の場所を返す。ファイル名は時刻順に並ぶ(古いものから消すのに使う)。"""
        t = record["time"]
        name = time.strftime("%Y%m%d_%H%M%S", time.localtime(t)) + f"_{int(t * 1000) % 1000:03d}_n{record['count']}.jpg"
        path = self.out_dir / name
        path.write_bytes(jpeg)
        with self._log.open("a", encoding="utf-8") as f:
            f.write(json.dumps(dict(record, photo=name), ensure_ascii=False) + "\n")
        self.prune()
        return path

    def prune(self) -> int:
        """写真が max_photos 枚を超えていたら古いものから消し、消した枚数を返す。"""
        photos = sorted(self.out_dir.glob("*.jpg"))
        extra = photos[:max(len(photos) - self.max_photos, 0)]
        for p in extra:
            p.unlink()
        return len(extra)


def make_record(dets: list[Detection], now: float, frame_index: int) -> dict:
    """出来事の記録(G1の保存とPCへの配信で同じものを使う)。"""
    return {
        "time": now,
        "time_text": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
        "frame_index": frame_index,
        "count": len(dets),
        "detections": [{"class_name": d.class_name, "confidence": round(d.confidence, 3),
                        "bbox": [round(v, 1) for v in d.bbox]} for d in dets],
    }
