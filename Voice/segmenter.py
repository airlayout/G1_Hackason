"""短いフレームの RMS 音量から発話区間を切り出す、モデル非依存の状態機械。"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SegmentConfig:
    threshold_dbfs: float = -35.0
    frame_ms: int = 20
    start_ms: int = 100
    end_ms: int = 600
    pre_roll_ms: int = 300
    min_voiced_ms: int = 300
    max_ms: int = 15000
    sample_rate: int = 16000

    def __post_init__(self) -> None:
        if not math.isfinite(self.threshold_dbfs) or not -100 <= self.threshold_dbfs <= 0:
            raise ValueError("threshold_dbfs は -100〜0 にしてください")
        values = (self.frame_ms, self.start_ms, self.end_ms, self.pre_roll_ms, self.min_voiced_ms, self.max_ms, self.sample_rate)
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in values):
            raise ValueError("区間設定は非負の整数にしてください")
        if self.frame_ms <= 0 or self.sample_rate != 16000 or self.sample_rate * self.frame_ms % 1000:
            raise ValueError("16 kHz で整数サンプルに分割できる frame_ms が必要です")
        if min(self.start_ms, self.end_ms, self.min_voiced_ms) < self.frame_ms:
            raise ValueError("開始・終了・最短発声時間は 1 フレーム以上にしてください")
        if not self.start_ms <= self.min_voiced_ms < self.max_ms <= 30000:
            raise ValueError("start_ms <= min_voiced_ms < max_ms <= 30000 が必要です")
        if self.pre_roll_ms + self.start_ms + self.end_ms >= self.max_ms:
            raise ValueError("最大区間は開始前バッファと終了待ちを含めて十分長くしてください")


@dataclass(frozen=True)
class Segment:
    samples: np.ndarray
    ended_at: float


class EnergySegmenter:
    def __init__(self, config: SegmentConfig) -> None:
        self.config = config
        self.frame_samples = config.sample_rate * config.frame_ms // 1000
        self.reset()

    def reset(self) -> None:
        cfg = self.config
        self.history: deque[np.ndarray] = deque(maxlen=math.ceil((cfg.pre_roll_ms + cfg.start_ms) / cfg.frame_ms))
        self.pending = np.empty(0, dtype=np.float32)
        self.active: list[np.ndarray] = []
        self.loud_frames = self.voiced_frames = self.quiet_frames = 0
        self.blocked = False
        self.last_dbfs = float("-inf")

    def feed(self, samples: np.ndarray, now: float) -> list[Segment]:
        samples = np.asarray(samples, dtype=np.float32)
        if samples.ndim != 1 or not np.isfinite(samples).all():
            self.reset()
            raise ValueError("マイク波形が不正です")
        self.pending = np.concatenate((self.pending, samples))
        segments = []
        count = len(self.pending) // self.frame_samples
        for i in range(count):
            frame = self.pending[i * self.frame_samples:(i + 1) * self.frame_samples].copy()
            result = self._frame(frame, now - (count - i - 1) * self.config.frame_ms / 1000)
            if result is not None:
                segments.append(result)
        self.pending = self.pending[count * self.frame_samples:].copy()
        return segments

    def _frame(self, frame: np.ndarray, now: float) -> Segment | None:
        cfg = self.config
        rms = float(np.sqrt(np.mean(frame.astype(np.float64) ** 2)))
        self.last_dbfs = 20 * math.log10(rms) if rms else float("-inf")
        loud = self.last_dbfs >= cfg.threshold_dbfs
        self.quiet_frames = 0 if loud else self.quiet_frames + 1
        if self.blocked:
            if self.quiet_frames * cfg.frame_ms >= cfg.end_ms:
                self.blocked = False
                self.history.clear()
            return None
        if not self.active:
            self.history.append(frame)
            self.loud_frames = self.loud_frames + 1 if loud else 0
            if self.loud_frames * cfg.frame_ms >= cfg.start_ms:
                self.active = list(self.history)
                self.voiced_frames = self.loud_frames
            return None
        self.active.append(frame)
        self.voiced_frames += int(loud)
        if len(self.active) * cfg.frame_ms >= cfg.max_ms:
            # 長い連続音を分割して何度も実行せず、無音が続くまで再待受しない。
            self.active.clear()
            self.history.clear()
            self.blocked = True
            return None
        if self.quiet_frames * cfg.frame_ms < cfg.end_ms:
            return None
        result = None
        if self.voiced_frames * cfg.frame_ms >= cfg.min_voiced_ms:
            # 終了判定用の無音を除き、末尾には 1 フレームだけ残す。
            keep = max(1, len(self.active) - self.quiet_frames + 1)
            result = Segment(np.concatenate(self.active[:keep]), now)
        self.active.clear()
        self.history.clear()
        self.loud_frames = self.voiced_frames = 0
        return result
