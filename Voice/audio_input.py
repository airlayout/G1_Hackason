"""音声を EmbeddingGemma 2 用の 16 kHz・モノラル波形に整える。"""

from __future__ import annotations

from dataclasses import dataclass
from math import gcd, isfinite
from pathlib import Path

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

SAMPLE_RATE = 16_000
MAX_SECONDS = 30.0
MIN_SECONDS = 0.2


@dataclass(frozen=True)
class AudioClip:
    samples: np.ndarray
    sample_rate: int = SAMPLE_RATE

    @property
    def duration(self) -> float:
        return len(self.samples) / self.sample_rate


def prepare_audio(samples: np.ndarray, sample_rate: int) -> AudioClip:
    """ステレオを平均し、必要ならリサンプルする。無音・不正値・長すぎる音声は断る。"""
    samples = np.asarray(samples, dtype=np.float32)
    if sample_rate <= 0 or samples.ndim not in (1, 2) or samples.size == 0:
        raise ValueError("音声は空でない 1 次元波形か、フレーム×チャンネルの配列が必要です")
    if not MIN_SECONDS <= len(samples) / sample_rate <= MAX_SECONDS:
        raise ValueError(f"音声の長さは {MIN_SECONDS}〜{MAX_SECONDS} 秒にしてください")
    if not np.isfinite(samples).all():
        raise ValueError("音声に NaN または無限大が含まれています")
    if samples.ndim == 2:
        samples = samples.mean(axis=1)
    if sample_rate != SAMPLE_RATE:
        divisor = gcd(sample_rate, SAMPLE_RATE)
        samples = resample_poly(samples, SAMPLE_RATE // divisor, sample_rate // divisor)
    if float(np.sqrt(np.mean(samples.astype(np.float64) ** 2))) < 1e-4:
        raise ValueError("音声が無音、または音量が小さすぎます")
    return AudioClip(np.ascontiguousarray(samples, dtype=np.float32))


def load_audio(path: Path) -> AudioClip:
    """SoundFile が扱える WAV / FLAC 等を読む。長さを確認してからメモリに載せる。"""
    path = path.expanduser()
    info = sf.info(path)
    if not MIN_SECONDS <= info.duration <= MAX_SECONDS:
        raise ValueError(f"音声の長さは {MIN_SECONDS}〜{MAX_SECONDS} 秒にしてください")
    samples, sample_rate = sf.read(path, dtype="float32", always_2d=True)
    return prepare_audio(samples, sample_rate)


def record_audio(seconds: float, device: str | None = None) -> AudioClip:
    """操作 PC のマイクを指定秒数だけ録音する。G1 の UDP マイクとは別の入力。"""
    if not isfinite(seconds) or not MIN_SECONDS <= seconds <= MAX_SECONDS:
        raise ValueError(f"録音時間は {MIN_SECONDS}〜{MAX_SECONDS} 秒にしてください")
    try:
        import sounddevice as sd
    except (ImportError, OSError) as exc:
        raise RuntimeError("マイク入力には sounddevice と PortAudio が必要です。WAV 入力も使えます") from exc
    try:
        samples = sd.rec(
            round(seconds * SAMPLE_RATE), samplerate=SAMPLE_RATE,
            channels=1, dtype="float32", device=device, blocking=True,
        )
    except sd.PortAudioError as exc:
        raise RuntimeError(f"マイクから録音できませんでした: {exc}") from exc
    return prepare_audio(samples, SAMPLE_RATE)
