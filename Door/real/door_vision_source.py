"""11-05: 画像の特徴（door_vision_node.py が 10 Hz で送る）を door_run_stock.py の中で受ける。numpy だけ。

電文（UDP・同じ PC2 の中）: [b"DVF1", 番号 u32, 特徴の数 u32, 撮った時刻 f64, 網を通し終えた時刻 f64, 網の時間 ms f32]＋特徴 f32。
時刻はどちらも time.monotonic()（Linux の CLOCK_MONOTONIC はプロセスをまたいで同じ時計）。
シミュレータは 5 歩（0.1 s）に 1 回描いて間は同じ特徴を持ち回すので、受け取った最新の特徴を毎周期そのまま使う。

見張り（止めるのは呼ぶ側）: 特徴が STALE_S（0.5 s）届かない・撮ってからの古さが AGE_MAX_S を越える・特徴の数が合わない
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Optional, Tuple

import numpy as np

PORT = 52731
MAGIC = b"DVF1"
HEADER = struct.Struct("<4sIIddf")
STALE_S = 0.5  # 特徴がこれだけ届かなければ止める（シミュレータは 0.1 s ごとに作り直す）
AGE_MAX_S = 0.6  # 撮ってから方策に入るまでの古さの上限（純正の映像サービスの遅れ＋復号＋網）


def pack(seq: int, feat: np.ndarray, t_capture: float, t_done: float, encode_ms: float) -> bytes:
    f = np.asarray(feat, dtype=np.float32).reshape(-1)
    return HEADER.pack(MAGIC, seq & 0xFFFFFFFF, f.shape[0], t_capture, t_done, encode_ms) + f.tobytes()


def unpack(data: bytes) -> Optional[Tuple[int, np.ndarray, float, float, float]]:
    """(番号, 特徴, 撮った時刻, 通し終えた時刻, 網の ms)。形が違えば None"""
    if len(data) < HEADER.size:
        return None
    magic, seq, n, t_cap, t_done, ms = HEADER.unpack_from(data)
    if magic != MAGIC or len(data) != HEADER.size + 4 * n:
        return None
    return seq, np.frombuffer(data, dtype=np.float32, offset=HEADER.size).astype(np.float64), t_cap, t_done, ms


class VisionFeatSource:
    """受ける糸を 1 本持ち、最新の特徴を覚える"""

    def __init__(self, feat_dim: int, port: int = PORT, host: str = "127.0.0.1") -> None:
        self.feat_dim = int(feat_dim)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[int, np.ndarray, float, float, float, float]] = None
        self.n_rx = 0
        self.n_bad = 0
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            m = unpack(data)
            if m is None or m[1].shape[0] != self.feat_dim:
                self.n_bad += 1
                continue
            with self._lock:
                self._latest = (*m, time.monotonic())
                self.n_rx += 1

    def latest(self) -> Optional[Tuple[int, np.ndarray, float, float, float, float]]:
        """(番号, 特徴, 撮った時刻, 通し終えた時刻, 網の ms, 受け取った時刻) か None"""
        with self._lock:
            return self._latest

    def why_not(self, now: float) -> str:
        """使えない理由（空なら使ってよい）"""
        m = self.latest()
        if m is None:
            return "画像の特徴がまだ来ない（door_vision_node.py を見る）" + (f"・形の違う電文 {self.n_bad}" if self.n_bad else "")
        if now - m[5] > STALE_S:
            return f"画像の特徴が {now - m[5]:.2f} s 来ない"
        if now - m[2] > AGE_MAX_S:
            return f"画像の特徴が撮ってから {now - m[2]:.2f} s 古い"
        return ""

    def close(self) -> None:
        self._stop.set()
        self._sock.close()
