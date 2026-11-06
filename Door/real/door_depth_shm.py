"""11-06: 頭カメラ D435i の深度を、Python 3.8 の中継（pc2/depth_relay.py）からカメラ側（door_vision_node.py・3.12）へ渡す書式。numpy だけ。

PC2 の lerobot（3.12）には pyrealsense2 が入らない（PyPI の 3.12 用は glibc 2.38 が要り、PC2 は 2.31）。機体に元からある
3.8 の pyrealsense2 で受け、最新の 1 枚を tmpfs（/dev/shm）のファイルに置く。隣の一時ファイルに書いてから os.replace で
差し替えるので、読む側が書きかけの枚を読むことはない。
中身: [b"DDP1", 幅 u32, 高さ u32, 番号 u32, 受けた時刻 f64, 尺度 f64（z16 → m）, fx, fy, cx, cy f64]＋深度 z16（u16・行の順）。
時刻は time.monotonic()（Linux の CLOCK_MONOTONIC はプロセスをまたいで同じ時計）。
"""
from __future__ import annotations

import os
import struct
from typing import NamedTuple, Optional, Tuple

import numpy as np

PATH = "/dev/shm/door_depth.bin"
MAGIC = b"DDP1"
HEADER = struct.Struct("<4sIIIdddddd")


class DepthFrame(NamedTuple):
    z16: np.ndarray  # [H, W] u16
    seq: int
    t: float
    scale: float
    intrinsics: Tuple[float, float, float, float]  # fx・fy・cx・cy（画素）


def write_frame(path: str, z16: np.ndarray, seq: int, t: float, scale: float,
                intrinsics: Tuple[float, float, float, float]) -> None:
    z = np.ascontiguousarray(z16, dtype=np.uint16)
    h, w = z.shape
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(HEADER.pack(MAGIC, w, h, seq & 0xFFFFFFFF, t, scale, *intrinsics))
        f.write(z.tobytes())
    os.replace(tmp, path)


def read_frame(path: str) -> Optional[DepthFrame]:
    """最新の 1 枚。まだ無い・形が違えば None"""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except FileNotFoundError:
        return None
    if len(data) < HEADER.size:
        return None
    magic, w, h, seq, t, scale, fx, fy, cx, cy = HEADER.unpack_from(data)
    if magic != MAGIC or len(data) != HEADER.size + 2 * w * h:
        return None
    z = np.frombuffer(data, dtype=np.uint16, offset=HEADER.size).reshape(h, w)
    return DepthFrame(z, seq, t, scale, (fx, fy, cx, cy))
