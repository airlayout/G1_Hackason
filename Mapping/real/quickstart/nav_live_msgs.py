#!/usr/bin/env python3
"""`nav_live_view.py` が使う ROS 2 メッセージ（CDR）のデコードと TF 合成。

描画とサーバ（nav_live_view.py）から切り離してあるのは、
**受け取る側の正しさをここだけ見れば確かめられる**ようにするため。
CDR の読み方そのものは `measure_overlay.Cdr` を再利用する。

⚠️ ここで扱う OccupancyGrid は **origin が左下・row-major で下から上**。
PGM（`measure_overlay.read_map`）とは行の向きが逆なので取り違えないこと。
"""
from __future__ import annotations

import math
import struct
import threading
import time

import numpy as np

from measure_overlay import Cdr

# 購読するトピック。増やすときは DECODERS にも足す
TOPICS = [
    "/map",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
    "/local_costmap/published_footprint",
    "/plan",
    "/scan",
    "/tf",
    "/tf_static",
    "/amcl_pose",
]


class Msg(Cdr):
    """measure_overlay.Cdr に float32 と配列読みを足したもの。"""

    def f32(self) -> float:
        self._al(4)
        v = struct.unpack_from("<f", self.b, self.a)[0]
        self.a += 4
        return v

    def stamp(self) -> float:
        sec, nsec = self.i32(), self.u32()
        return sec + nsec * 1e-9

    def header(self) -> tuple[float, str]:
        return self.stamp(), self.st()

    def seq_i8(self) -> np.ndarray:
        n = self.u32()
        v = np.frombuffer(self.b, np.int8, n, self.a)
        self.a += n
        return v

    def seq_f32(self) -> np.ndarray:
        n = self.u32()
        v = np.frombuffer(self.b, np.float32, n, self.a)   # u32 の直後は 4 境界
        self.a += 4 * n
        return v

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        p = np.array([self.f64() for _ in range(3)])
        q = np.array([self.f64() for _ in range(4)])       # x y z w
        return p, q


def yaw_of(q: np.ndarray) -> float:
    """クォータニオン (x,y,z,w) から yaw。2D に落とすのでこれだけで足りる。"""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def dec_grid(b: bytes) -> dict:
    r = Msg(b)
    stamp, frame = r.header()
    r.i32(); r.u32()                                   # info.map_load_time
    res = r.f32()
    w, h = r.u32(), r.u32()
    p, q = r.pose()
    data = r.seq_i8()
    if data.size != w * h:
        raise ValueError(f"data {data.size} != {w}x{h}")
    return {"stamp": stamp, "frame": frame, "res": res, "w": w, "h": h,
            "ox": float(p[0]), "oy": float(p[1]), "oyaw": yaw_of(q),
            "grid": data.reshape(h, w)}


def dec_path(b: bytes) -> dict:
    r = Msg(b)
    stamp, frame = r.header()
    n = r.u32()
    pts = np.empty((n, 2))
    for i in range(n):
        r.header()
        p, _ = r.pose()
        pts[i] = p[:2]
    return {"stamp": stamp, "frame": frame, "pts": pts}


def dec_polygon(b: bytes) -> dict:
    r = Msg(b)
    stamp, frame = r.header()
    n = r.u32()
    pts = np.array([[r.f32(), r.f32(), r.f32()] for _ in range(n)]).reshape(-1, 3)
    return {"stamp": stamp, "frame": frame, "pts": pts[:, :2]}


def dec_scan(b: bytes) -> dict:
    r = Msg(b)
    stamp, frame = r.header()
    a0, a1, ainc = r.f32(), r.f32(), r.f32()
    r.f32(); r.f32()                                   # time_increment, scan_time
    rmin, rmax = r.f32(), r.f32()
    ranges = r.seq_f32()
    ang = a0 + ainc * np.arange(ranges.size)
    ok = np.isfinite(ranges) & (ranges > rmin) & (ranges < rmax)
    xy = np.stack([ranges[ok] * np.cos(ang[ok]), ranges[ok] * np.sin(ang[ok])], 1)
    return {"stamp": stamp, "frame": frame, "xy": xy, "n": int(ok.sum()),
            "span": (float(a0), float(a1))}


def dec_tf(b: bytes) -> list[tuple[str, str, np.ndarray, np.ndarray, float]]:
    r = Msg(b)
    out = []
    for _ in range(r.u32()):
        ts = r.stamp()
        parent, child = r.st(), r.st()
        t = np.array([r.f64() for _ in range(3)])
        q = np.array([r.f64() for _ in range(4)])
        out.append((parent, child, t, q, ts))
    return out


def dec_pose_cov(b: bytes) -> dict:
    r = Msg(b)
    stamp, frame = r.header()
    p, q = r.pose()
    cov = np.array([r.f64() for _ in range(36)]).reshape(6, 6)
    return {"stamp": stamp, "frame": frame, "x": float(p[0]), "y": float(p[1]),
            "yaw": yaw_of(q), "sx": float(math.sqrt(max(cov[0, 0], 0.0))),
            "sy": float(math.sqrt(max(cov[1, 1], 0.0))),
            "syaw": float(math.sqrt(max(cov[5, 5], 0.0)))}


DECODERS = {
    "/map": dec_grid,
    "/global_costmap/costmap": dec_grid,
    "/local_costmap/costmap": dec_grid,
    "/local_costmap/published_footprint": dec_polygon,
    "/plan": dec_path,
    "/scan": dec_scan,
    "/amcl_pose": dec_pose_cov,
}


# ── 状態（橋 → 描画の受け渡し） ──────────────────────────────────────────
class State:
    """最新のメッセージだけを持つ。/map は latched で 1 度しか来ないので消さない。"""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.msgs: dict[str, dict] = {}       # topic -> 復号済み
        self.recv: dict[str, float] = {}      # topic -> 受信時刻（Mac の時計）
        self.count: dict[str, int] = {}
        # ⚠️ 帯域を必ず測る。機体の AP は 2.4GHz/20MHz で実効 3 MB/s しかなく、
        #    購読を増やすと無線が飽和して**測位そのものを壊す**。点群は購読しない。
        self.nbytes: dict[str, int] = {}
        self.start = time.time()
        self.tf: dict[str, tuple] = {}        # child -> (parent, t, q, stamp, is_static)
        self.conn = "起動中"
        self.skipped: list[str] = []          # --skip で購読しないもの（帯域を空ける）
        self.advertised: list[str] = []
        self.missing: list[str] = []
        self.err = ""

    def put(self, topic: str, payload: bytes) -> None:
        if topic in ("/tf", "/tf_static"):
            static = topic == "/tf_static"
            with self.lock:
                for parent, child, t, q, ts in dec_tf(payload):
                    self.tf[child] = (parent, t, q, ts, static)
                self._mark(topic, len(payload))
            return
        m = DECODERS[topic](payload)
        with self.lock:
            self.msgs[topic] = m
            self._mark(topic, len(payload))

    def _mark(self, topic: str, nbytes: int) -> None:
        """受信の記録。**ロックを持った状態で呼ぶこと。**"""
        self.recv[topic] = time.time()
        self.count[topic] = self.count.get(topic, 0) + 1
        self.nbytes[topic] = self.nbytes.get(topic, 0) + nbytes

    def snapshot(self) -> dict:
        with self.lock:
            return {"msgs": dict(self.msgs), "recv": dict(self.recv),
                    "count": dict(self.count), "tf": dict(self.tf),
                    "nbytes": dict(self.nbytes), "start": self.start,
                    "conn": self.conn, "advertised": list(self.advertised),
                    "skipped": list(self.skipped),
                    "missing": list(self.missing), "err": self.err}


def tf_chain(tf: dict, target: str, source: str) -> tuple[np.ndarray, float] | None:
    """source 系の点を target 系へ移す (t[2], yaw)。map→odom→base_link も辿る。"""
    if source == target:
        return np.zeros(2), 0.0
    cur, acc_t, acc_yaw = source, np.zeros(2), 0.0
    for _ in range(16):
        e = tf.get(cur)
        if e is None:
            return None
        parent, t, q, _, _ = e
        yaw = yaw_of(q)
        c, s = math.cos(yaw), math.sin(yaw)
        acc_t = np.array([t[0] + c * acc_t[0] - s * acc_t[1],
                          t[1] + s * acc_t[0] + c * acc_t[1]])
        acc_yaw += yaw
        cur = parent
        if cur == target:
            return acc_t, acc_yaw
    return None

