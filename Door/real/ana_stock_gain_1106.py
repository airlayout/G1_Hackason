"""11-06: 純正の歩行が方策の速度の指令にどれだけ応えるかを、指令の大きさごとに測る（一様な倍率か・不感帯か）。

DOOR と PROBE の周期で、送った指令（vel_want: [歩く, vx, vy, wz]）と /dog_odom の微分（体の座標・0.5 s の幅で平滑）を、
遅れ（0〜1.2 s で相関が最大の所）をずらして比べる。⚠️ /dog_odom は距離を横 7〜10%・前後 4〜6% 短く出す（10-21）。

使い方（OMEN）: python ana_stock_gain_1106.py <door_stock_*.npz> ...
"""
from __future__ import annotations

import sys
from typing import List, Tuple

import numpy as np

DOOR, PROBE = 3, 7  # door_run_stock.PHASES の番号
BINS = {0: (0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5), 1: (0.0, 0.05, 0.1, 0.2, 0.3, 0.5), 2: (0.0, 0.1, 0.2, 0.35, 0.5, 1.0)}
NAMES = ("前後 vx", "左右 vy", "旋回 wz")
DT = 0.02


def body_vel(odom: np.ndarray, t: np.ndarray, win: float = 0.5) -> np.ndarray:
    """odom（世界の x・y・向き）→ 体の座標の速度 [N, 3]（中心差分を win 秒で）"""
    k = max(1, int(round(win / 2 / DT)))
    v = np.full((len(t), 3), np.nan)
    for i in range(k, len(t) - k):
        dt = t[i + k] - t[i - k]
        if dt <= 0:
            continue
        dx, dy = odom[i + k, :2] - odom[i - k, :2]
        dyaw = (odom[i + k, 2] - odom[i - k, 2] + np.pi) % (2 * np.pi) - np.pi
        c, s = np.cos(odom[i, 2]), np.sin(odom[i, 2])
        v[i] = (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt
    return v


def collect(paths: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    cmds, vels = [], []
    for path in paths:
        z = np.load(path, allow_pickle=True)
        t, ph, want, odom = z["t"], z["phase"].astype(int), z["vel_want"], z["odom"]
        m = np.isin(ph, (DOOR, PROBE)) & np.isfinite(odom).all(1) & np.isfinite(want).all(1)
        if m.sum() < 50:
            continue
        v = body_vel(odom, t)
        cmds.append(np.where(want[:, :1] > 0.5, want[:, 1:4], 0.0)[m])
        vels.append(v[m])
        print(f"[gain] {path.split('/')[-1]}: DOOR・PROBE {m.sum() * DT:.1f} 秒", flush=True)
    return np.concatenate(cmds), np.concatenate(vels)


def best_lag(c: np.ndarray, v: np.ndarray) -> int:
    ok = np.isfinite(v)
    best, arg = -1e9, 0
    for lag in range(0, int(1.2 / DT)):
        a, b = c[: len(c) - lag], v[lag:]
        m = ok[lag:]
        if m.sum() < 50 or np.std(a[m]) < 1e-6:
            continue
        r = np.corrcoef(a[m], b[m])[0, 1]
        if r > best:
            best, arg = r, lag
    return arg


def main(paths: List[str]) -> int:
    c, v = collect(paths)
    for ax in range(3):
        lag = best_lag(c[:, ax], v[:, ax])
        a, b = c[: len(c) - lag, ax], v[lag:, ax]
        m = np.isfinite(b)
        a, b = a[m], b[m]
        nz = np.abs(a) > 1e-3
        slope = float(np.sum(a[nz] * b[nz]) / np.sum(a[nz] ** 2)) if nz.any() else float("nan")
        print(f"\n[gain] {NAMES[ax]}: 遅れ {lag * DT:.2f} s・原点を通る傾き（進んだ／指令） {slope:.2f}", flush=True)
        edges = BINS[ax]
        for lo, hi in zip(edges[:-1], edges[1:]):
            for sgn in (1, -1):
                sel = (sgn * a > lo) & (sgn * a <= hi)
                if sel.sum() < 25:
                    continue
                print(f"   指令 {'+' if sgn > 0 else '−'}{lo:.2f}〜{hi:.2f}: {sel.sum() * DT:5.1f} 秒・指令の平均 {np.mean(a[sel]):+.3f}・"
                      f"進んだ平均 {np.mean(b[sel]):+.3f}（{100 * np.mean(b[sel]) / np.mean(a[sel]):4.0f}%）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
