"""record_dds_to_bag.py で録った /dog_odom（nav_msgs/Odometry、約 1000 Hz）を db3 から numpy に読む。段 1（2026-10-21）の脚の odometry の計測用。

返すもの（すべて同じ長さ）:
  recv_s  PC2 が受け取った時刻 [s]（db3 の timestamp。ライダーの記録の recv_ns と同じ時計）
  stamp_s ヘッダの時刻 [s]（機体の時計。PC2 と同じとは限らない）
  xyz     (n, 3) 位置 [m]（odom → robot_center）
  yaw     向き [rad]（巻き戻しを解いた連続の値）
  rpy     (n, 3) roll・pitch・yaw [rad]（yaw は巻き戻し前）

使い方（単体）: Navigation/.venv/bin/python Door/real/dog_odom_bag.py <セッションの場所>   → 要約を出す
"""
from __future__ import annotations

import math
import sqlite3
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ODOM_TOPIC = "/dog_odom"


@dataclass(frozen=True)
class OdomTrack:
    recv_s: np.ndarray
    stamp_s: np.ndarray
    xyz: np.ndarray
    yaw: np.ndarray
    rpy: np.ndarray


def parse_odom_cdr(buf: bytes) -> tuple:
    """CDR から (sec, nanosec, x, y, z, qx, qy, qz, qw)。Mapping/real/quickstart/pc2/dog_odom_to_tf.py と同じ読み方

    ⚠️ 整列は encapsulation ヘッダ（先頭 4 バイト）の後ろを基点に数える
    """
    e = "<" if buf[1] in (1, 3) else ">"
    sec, nanosec = struct.unpack_from(e + "iI", buf, 4)
    off = 12
    (n,) = struct.unpack_from(e + "I", buf, off)
    off += 4 + n
    off = 4 + ((off - 4 + 3) & ~3)
    (n,) = struct.unpack_from(e + "I", buf, off)
    off += 4 + n
    off = 4 + ((off - 4 + 7) & ~7)
    return (sec, nanosec) + struct.unpack_from(e + "7d", buf, off)


def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def find_db3(session: Path) -> list[Path]:
    files = sorted((session / "raw" / "rosbag2").glob("*.db3"))
    if not files:
        raise SystemExit(f"[odom] db3 が無い: {session}")
    return files


def load_odom(session: Path) -> OdomTrack:
    recv, rows = [], []
    for bag in find_db3(session):
        con = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name = ?", (ODOM_TOPIC,)).fetchone()
            if tid is None:
                raise SystemExit(f"[odom] {bag} に {ODOM_TOPIC} が無い")
            for ts, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", tid):
                recv.append(ts)
                rows.append(parse_odom_cdr(data))
        finally:
            con.close()
    a = np.array(rows, dtype=np.float64)
    rpy = quat_to_rpy(a[:, 5:9])
    return OdomTrack(recv_s=np.array(recv, np.float64) / 1e9, stamp_s=a[:, 0] + a[:, 1] / 1e9,
                     xyz=a[:, 2:5], yaw=np.unwrap(rpy[:, 2]), rpy=rpy)


def summary(track: OdomTrack) -> dict:
    t = track.recv_s - track.recv_s[0]
    d = track.xyz - track.xyz[0]
    return {
        "件": len(t), "秒": round(float(t[-1]), 2), "Hz": round((len(t) - 1) / float(t[-1]), 1),
        "受信とヘッダの時刻の差[s]": round(float(np.median(track.recv_s - track.stamp_s)), 3),
        "最後の位置の変化[mm]": [round(float(v) * 1000, 1) for v in d[-1]],
        "位置の最大の振れ xy[mm]": round(float(np.max(np.hypot(d[:, 0], d[:, 1]))) * 1000, 1),
        "向きの変化[°]": round(math.degrees(float(track.yaw[-1] - track.yaw[0])), 3),
        "向きの流れ[°/s]": round(math.degrees(float(np.polyfit(t, track.yaw, 1)[0])), 4),
        "roll・pitch の中央値[°]": [round(math.degrees(float(np.median(track.rpy[:, i]))), 2) for i in (0, 1)],
    }


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        print(Path(arg).name, summary(load_odom(Path(arg))))
