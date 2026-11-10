"""G1 のマイクの音を受けて保存し、エレベーターの到着の音（高めのチャイム）を探す（real/mic_record.py・real/chime_analyze.py が使う）。

マイクの仕様（2026-11-10 にネットで調べた。実機ではまだ確かめていない。docs/reports/REPORT_20261110.md の 8.）:
- UDP マルチキャスト 239.168.123.161:5555（有線の 192.168.123.x の中だけ。SDK は要らない）
- 16 kHz・モノラル・16 bit リトルエンディアン
- アプリかリモコンでマイクを起こさないと送られない・音が 0 のままのことがある
- ノイズキャンセリングがかかっているらしい（マイクを使っているメンバーの話。未確認）。続く音（ファンなど）が消され、
  チャイムのような音も弱められるかもしれない。まず録って確かめる

到着の音の探し方（detect_tones）: 短い区間ごとの周波数の強さ（スペクトログラム）を作り、高めの帯（既定 1〜6 kHz）で
一番強い周波数が、その周波数のふだんの強さ（録音全体の中央値）より min_db 以上強い区間が min_s 以上続いたら、音が鳴ったとする。
しきい値は、実機で録った到着の音を見て決め直す。numpy だけを使う（PC2 でも動かすため）。
"""

from __future__ import annotations

import json
import socket
import struct
import time
import wave
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

MIC_GROUP = "239.168.123.161"
MIC_PORT = 5555
SAMPLE_RATE = 16000


class MicTimeout(RuntimeError):
    """マイクのパケットが届かない。"""


def open_mic_socket(local_ip: str, group: str = MIC_GROUP, port: int = MIC_PORT, timeout_s: float = 3.0) -> socket.socket:
    """マイクのマルチキャストを受けるソケット。ポートに bind し、自分の有線の IP でグループに参加する
    （unitree_sdk2_python の PR #133 の g1_audio_mic_record_udp.py と同じ受け方）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    sock.bind(("", port))
    mreq = struct.pack("=4s4s", socket.inet_aton(group), socket.inet_aton(local_ip))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(timeout_s)
    return sock


@dataclass
class Chunk:
    t: float  # 受け取った時刻（time.time()。押した記録と並べるため）
    data: bytes


def receive(sock: socket.socket, seconds: float, on_chunk: Any = None) -> list[Chunk]:
    """seconds 秒分（16 kHz・16 bit で数えた量）を受ける。最初の 1 つが届かなければ MicTimeout。
    on_chunk(chunks) を渡すと、受けるたびに呼ぶ（画面に音の大きさを出すため）。Ctrl+C でそこまでを返す。"""
    want = int(seconds * SAMPLE_RATE) * 2
    chunks: list[Chunk] = []
    got = 0
    while got < want:
        try:
            data, _ = sock.recvfrom(65536)
        except KeyboardInterrupt:
            break
        except socket.timeout as e:
            if not chunks:
                raise MicTimeout("マイクのパケットが届かない（有線の IP・口、マイクが起きているか、"
                                 "WSL ならミラーモードで届くか）") from e
            break  # 途中で止まった。そこまでを返す
        if len(data) % 2:
            data = data[:-1]
        if not data:
            continue
        chunks.append(Chunk(time.time(), data))
        got += len(data)
        if on_chunk is not None:
            on_chunk(chunks)
    return chunks


def to_samples(chunks: list[Chunk]) -> np.ndarray:
    """受けたパケットをつないだ音（-1〜1 の float、16 kHz）。"""
    if not chunks:
        return np.zeros(0)
    pcm = np.frombuffer(b"".join(c.data for c in chunks), dtype="<i2")
    return pcm.astype(np.float64) / 32768.0


def save_wav(path: Path, samples: np.ndarray, sample_rate: int = SAMPLE_RATE) -> None:
    pcm = np.clip(np.round(np.asarray(samples) * 32768.0), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(pcm.tobytes())


def load_wav(path: Path) -> tuple[np.ndarray, int]:
    """WAV（16 bit。ステレオなら左だけ）→ (-1〜1 の float, サンプリング周波数)。"""
    with wave.open(str(path), "rb") as w:
        if w.getsampwidth() != 2:
            raise ValueError(f"16 bit の WAV だけ読める: {path}")
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64) / 32768.0
        ch, sr = w.getnchannels(), w.getframerate()
    return (x[::ch] if ch > 1 else x), sr


def level_dbfs(samples: np.ndarray) -> float:
    """音の大きさ（RMS、フルスケールを 0 dB）。無音なら -inf。"""
    if len(samples) == 0:
        return float("-inf")
    rms = float(np.sqrt(np.mean(np.square(samples))))
    return 20 * np.log10(rms) if rms > 0 else float("-inf")


def peak_hz(samples: np.ndarray, sample_rate: int = SAMPLE_RATE, band: tuple[float, float] = (500.0, 8000.0)) -> float:
    """band の中で一番強い周波数 [Hz]（画面に出す目安）。"""
    if len(samples) < 256:
        return float("nan")
    spec = np.abs(np.fft.rfft(samples * np.hanning(len(samples))))
    f = np.fft.rfftfreq(len(samples), 1.0 / sample_rate)
    m = (f >= band[0]) & (f <= band[1])
    return float(f[m][np.argmax(spec[m])]) if m.any() else float("nan")


def spectrogram(samples: np.ndarray, sample_rate: int = SAMPLE_RATE, n_fft: int = 1024,
                hop: int = 256) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(時刻 [s] (T,), 周波数 [Hz] (F,), 強さ [dB] (T, F))。窓は 1024 点（16 kHz で 64 ms）。"""
    x = np.asarray(samples, dtype=np.float64)
    if len(x) < n_fft:
        x = np.pad(x, (0, n_fft - len(x)))
    n = 1 + (len(x) - n_fft) // hop
    idx = np.arange(n_fft)[None, :] + hop * np.arange(n)[:, None]
    frames = x[idx] * np.hanning(n_fft)[None, :]
    power = np.abs(np.fft.rfft(frames, axis=1)) ** 2
    db = 10 * np.log10(power + 1e-12)
    return (np.arange(n) * hop + n_fft / 2) / sample_rate, np.fft.rfftfreq(n_fft, 1.0 / sample_rate), db


@dataclass
class Tone:
    start_s: float
    end_s: float
    peak_hz: float
    above_db: float  # その周波数のふだんの強さ（録音全体の中央値）より何 dB 強いか（区間の最大）

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


def detect_tones(samples: np.ndarray, sample_rate: int = SAMPLE_RATE, band: tuple[float, float] = (1000.0, 6000.0),
                 min_db: float = 15.0, min_s: float = 0.1, gap_s: float = 0.1) -> list[Tone]:
    """高めの帯で、ふだんより min_db 以上強い音が min_s 以上続いた区間（間が gap_s 以下なら 1 つにつなぐ）。"""
    t, f, db = spectrogram(samples, sample_rate)
    m = (f >= band[0]) & (f <= band[1])
    if not m.any() or len(t) == 0:
        return []
    above = db[:, m] - np.median(db[:, m], axis=0)[None, :]  # 周波数ごとの、ふだんからの上がり方
    best = np.argmax(above, axis=1)
    strength = above[np.arange(len(t)), best]
    on = strength >= min_db
    dt = t[1] - t[0] if len(t) > 1 else 0.016
    gap = int(round(gap_s / dt))
    off = np.flatnonzero(~on)
    # 音のある区間の間の、短い切れ目（gap 以下）を埋める
    runs = np.split(off, np.flatnonzero(np.diff(off) != 1) + 1) if len(off) else []
    for r in runs:
        if len(r) and r[0] > 0 and r[-1] < len(t) - 1 and len(r) <= gap:
            on[r] = True
    tones: list[Tone] = []
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    for i, j in zip(edges[::2], edges[1::2] - 1):
        k = i + int(np.argmax(strength[i:j + 1]))
        tone = Tone(float(t[i] - dt / 2), float(t[j] + dt / 2), float(f[m][best[k]]), float(strength[k]))
        if tone.duration_s >= min_s:
            tones.append(tone)
    return tones


def write_meta(path: Path, meta: dict[str, Any], tones: list[Tone]) -> None:
    path.write_text(json.dumps({**meta, "tones": [{**asdict(x), "duration_s": x.duration_s} for x in tones]},
                               ensure_ascii=False, indent=2, default=float))
