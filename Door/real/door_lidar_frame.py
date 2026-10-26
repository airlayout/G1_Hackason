"""段 0（2026-10-17）: 頭のライダーの 1 回の測定ごとに、扉の枠の位置と向き（機体から見て）を出し、その揺れを測る。機体は止まっている前提。

door_lidar_angle.py と同じ前処理（上下逆さまを戻し、全部の回を重ねて床で水平にする）のあと、回ごとに:
  壁   高さ 0.3〜1.25 m・前 0.4〜2.5 m の点に、上から見た直線を RANSAC で当てる → 壁までの距離・壁に対する機体の向き
  枠   壁から手前に 1.5〜6 cm 出ている点（10-16 の 0° の記録で、開口の両側の枠が 3.5 cm 出ていた）を、
       壁に沿って |u| 0.35〜0.8 m で左右に分け、中央値の真ん中を開口の中心、差を開口の幅とする
正解（巻き尺）は無いので、出せるのは「回ごとの揺れ」と「記録ごとの平均」だけ（片寄りは分からない）

使い方: Navigation/.venv/bin/python Door/real/door_lidar_frame.py <crop.npz> [<crop.npz> ...]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_angle as dla  # noqa: E402

PROUD_MIN, PROUD_MAX = 0.015, 0.06  # [m] 壁から手前に出ている枠
JAMB_U_MIN, JAMB_U_MAX = 0.35, 0.80  # [m] 開口の中心から枠までの距離の探す範囲
MIN_JAMB_POINTS = 8


def fit_line_2d(xy: np.ndarray, tol: float = 0.02, iters: int = 200, seed: int = 0) -> tuple[np.ndarray, float]:
    """上から見た直線 n·p = c（n は水平の単位ベクトル、機体の方を向く）を RANSAC で当て、内側の点で主成分に直す"""
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for _ in range(iters):
        a, b = xy[rng.choice(len(xy), 2, replace=False)]
        d = b - a
        if np.linalg.norm(d) < 0.2:
            continue
        n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        inl = np.abs((xy - a) @ n) < tol
        if inl.sum() > best_n:
            best, best_n = inl, inl.sum()
    q = xy[best]
    m = q.mean(0)
    direction = np.linalg.svd(q - m, full_matrices=False)[2][0]
    n = np.array([-direction[1], direction[0]])
    if n[0] > 0:  # 機体（原点）の方を向かせる
        n = -n
    return n, float(n @ m)


WALL_NEAR_M = 0.08  # 基準の壁からこの距離までの点を、その回の壁の候補にする
OPENING_HALF_M = 0.55  # 開口の中心からこれより外（両側の柱）だけで壁を当てる。開いた扉の板を壁と取り違えないため


def reference_wall(lev: list[np.ndarray], n_first: int = 30) -> tuple[np.ndarray, float]:
    """記録の始め（扉が閉じている）の回を重ねて、基準の壁を当てる。実機でも、扉が閉じた状態で測り始める"""
    q = np.concatenate(lev[:n_first])
    band = q[(q[:, 2] > 0.3) & (q[:, 2] < 1.25) & (q[:, 0] > 0.4) & (q[:, 0] < 2.5) & (np.abs(q[:, 1]) < 2.0)]
    return fit_line_2d(band[:, :2])


def frame_per_scan(q: np.ndarray, ref: tuple[np.ndarray, float]) -> dict:
    n0, c0 = ref
    t0 = np.array([-n0[1], n0[0]])
    if t0[1] < 0:
        t0 = -t0
    band = q[(q[:, 2] > 0.3) & (q[:, 2] < 1.25)]
    v0, u0 = band[:, :2] @ n0 - c0, band[:, :2] @ t0
    pillars = band[(np.abs(v0) < WALL_NEAR_M) & (np.abs(u0) > OPENING_HALF_M) & (np.abs(u0) < 2.0)]
    if len(pillars) < 50:
        return {}
    n, c = fit_line_2d(pillars[:, :2])  # 両側の柱の面（扉が開いても動かない）
    out = {"dist": -c, "yaw_deg": math.degrees(math.atan2(-n[1], -n[0]))}
    # 枠: 記録の始めの閉じた扉の面（基準）から手前に出ている点。機体は止まっているので基準のままで測れる
    proud = (v0 > PROUD_MIN) & (v0 < PROUD_MAX)
    left = proud & (u0 > JAMB_U_MIN) & (u0 < JAMB_U_MAX)
    right = proud & (u0 < -JAMB_U_MIN) & (u0 > -JAMB_U_MAX)
    if left.sum() >= MIN_JAMB_POINTS and right.sum() >= MIN_JAMB_POINTS:
        ul, ur = float(np.median(u0[left])), float(np.median(u0[right]))
        out.update(center=(ul + ur) / 2, width=ul - ur)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("crops", type=Path, nargs="+")
    a = ap.parse_args()
    for path in a.crops:
        scans, _ = dla.load_scans(path)
        R, h = dla.level(scans)
        lev = [s @ R.T + np.array([0.0, 0.0, h]) for s in scans]
        ref = reference_wall(lev)
        rows = [frame_per_scan(q, ref) for q in lev]
        dist = np.array([r.get("dist", np.nan) for r in rows])
        yaw = np.array([r.get("yaw_deg", np.nan) for r in rows])
        cen = np.array([r.get("center", np.nan) for r in rows])
        wid = np.array([r.get("width", np.nan) for r in rows])
        print(f"== {path.stem}: {len(scans)} 回")
        for name, x, unit, k in (("壁までの距離", dist, "m", 1000), ("壁に対する向き", yaw, "°", 1),
                                 ("開口の中心（左右）", cen, "m", 1000), ("開口の幅（枠の間）", wid, "m", 1000)):
            ok = np.isfinite(x)
            if not ok.any():
                print(f"  {name}: 取れない")
                continue
            unit_s = "mm" if k == 1000 else unit
            print(f"  {name}: 平均 {np.nanmean(x):+.4f} {unit}・回ごとの揺れ σ {np.nanstd(x) * k:.2f} {unit_s}"
                  f"・取れた回 {ok.mean() * 100:.0f}%・最小〜最大 {np.nanmin(x):+.4f}〜{np.nanmax(x):+.4f}")


if __name__ == "__main__":
    main()
