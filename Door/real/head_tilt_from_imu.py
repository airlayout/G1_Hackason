"""頭の傾きを、記録に残っている頭の IMU（ライダー内蔵、/utlidar/imu_livox_mid360・約 200 Hz）の加速度から出す（2026-10-21）。

止まっている間の加速度計は「重力の反対（上）」を指す。頭（ライダー）の座標で見た上の向きから、
頭の前後の傾き（pitch）と左右の傾き（roll）を出す。ライダーは上下逆さまに付いているので、まっすぐなら上は −z を向く。
  pitch = atan2(a_x, −a_z)、roll = atan2(−a_y, −a_z)（符号は 10-21 の画像で確かめた: 下を向くほど負）
⚠️ 歩いている間は接地の加速度が混ざって使えない（09-12 に最大 10.6°）。止まっている間（加速度の大きさが 1 g の ±3% で、
   ジャイロが小さい）だけを使う。

使い方:
  head_tilt_from_imu.py <記録の dir> [<記録の dir> ...]        記録ごとに、止まっている間の頭の傾き
  head_tilt_from_imu.py --segments <記録の dir>                  止まっている区間ごとに分けて出す（歩く前後で変わるか）
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import struct
from pathlib import Path

import numpy as np

TOPIC = "/utlidar/imu_livox_mid360"
STILL_G_TOL = 0.03  # 加速度の大きさが 1 g からこれ以内
STILL_GYRO = 0.05  # [rad/s] ジャイロの大きさがこれ以下
STILL_MIN_S = 1.5  # 止まっている区間の最短


def parse_imu(buf: bytes) -> tuple[float, np.ndarray, np.ndarray]:
    """sensor_msgs/Imu の CDR から (時刻 [s], ジャイロ [3], 加速度 [3])。整列はカプセル化ヘッダの後ろが基点"""
    sec, nsec = struct.unpack_from("<iI", buf, 4)
    (n,) = struct.unpack_from("<I", buf, 12)
    off = 16 + n
    off = 4 + ((off - 4 + 7) & ~7)
    off += 8 * (4 + 9)  # orientation と その共分散
    gyro = np.array(struct.unpack_from("<3d", buf, off))
    off += 8 * (3 + 9)
    acc = np.array(struct.unpack_from("<3d", buf, off))
    return sec + nsec * 1e-9, gyro, acc


def find_db3(d: Path) -> list[Path]:
    files = sorted(d.glob("**/*.db3"))
    if not files:
        raise SystemExit(f"[head] db3 が無い: {d}")
    return files


def load(d: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ts, rs, gs, as_ = [], [], [], []
    for f in find_db3(d):
        con = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        try:
            row = con.execute("SELECT id FROM topics WHERE name = ?", (TOPIC,)).fetchone()
            if row is None:
                continue
            for recv, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (row[0],)):
                t, g, a = parse_imu(data)
                ts.append(t)
                rs.append(recv / 1e9)
                gs.append(g)
                as_.append(a)
        finally:
            con.close()
    if not ts:
        raise SystemExit(f"[head] {TOPIC} が無い: {d}")
    return np.array(ts), np.array(rs), np.array(gs), np.array(as_)


def tilt_deg(acc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """加速度（g）→ (pitch, roll) [°]"""
    ax, ay, az = acc[:, 0], acc[:, 1], acc[:, 2]
    return np.degrees(np.arctan2(ax, -az)), np.degrees(np.arctan2(-ay, -az))


def still_mask(gyro: np.ndarray, acc: np.ndarray, t: np.ndarray) -> np.ndarray:
    """止まっている所。0.5 s の窓で加速度の大きさとジャイロを見て、短い区間は落とす"""
    mag = np.linalg.norm(acc, axis=1)
    gm = np.linalg.norm(gyro, axis=1)
    k = max(1, int(0.5 / max(1e-3, np.median(np.diff(t)))))
    ker = np.ones(k) / k
    mag_s = np.convolve(mag, ker, mode="same")
    gm_s = np.convolve(gm, ker, mode="same")
    ok = (np.abs(mag_s - 1.0) < STILL_G_TOL) & (gm_s < STILL_GYRO)
    out = np.zeros_like(ok)
    i = 0
    while i < len(ok):
        if not ok[i]:
            i += 1
            continue
        j = i
        while j < len(ok) and ok[j]:
            j += 1
        if t[j - 1] - t[i] >= STILL_MIN_S:
            out[i:j] = True
        i = j
    return out


def segments(mask: np.ndarray) -> list[tuple[int, int]]:
    segs, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", type=Path, nargs="+")
    ap.add_argument("--segments", action="store_true")
    a = ap.parse_args()
    for d in a.dirs:
        t, recv, gyro, acc = load(d)
        m = still_mask(gyro, acc, t)
        pitch, roll = tilt_deg(acc)
        name = d.name if d.name not in ("bag", "rosbag2", "raw") else str(d)
        if not m.any():
            print(f"{name}: 止まっている所が無い（{len(t)} 件・{t[-1] - t[0]:.0f} s）")
            continue
        print(f"{name}: 頭の前後の傾き {np.median(pitch[m]):+6.2f}°・左右 {np.median(roll[m]):+5.2f}°"
              f"（止まっている {m.sum() / len(m) * 100:.0f}%・{t[-1] - t[0]:.0f} s・加速度の大きさ {np.median(np.linalg.norm(acc[m], axis=1)):.3f} g）")
        if a.segments:
            for i, j in segments(m):
                print(f"    {recv[i] - recv[0]:6.1f}〜{recv[j - 1] - recv[0]:6.1f} s  前後 {np.median(pitch[i:j]):+6.2f}°"
                      f"（揺れ σ {np.std(pitch[i:j]):.2f}°）・左右 {np.median(roll[i:j]):+5.2f}°")


if __name__ == "__main__":
    main()
