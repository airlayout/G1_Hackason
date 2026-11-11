"""11-11（すき間の罰の X を決める）: シムの評価の記録で、手の部品（指先・手のひら・手首・前腕の中ほど・肘、左右）と枠・壁の上から見た
距離を出し、X ごとに「罰に掛かる回」の割合を出す。numpy だけ（OMEN の CPU）。gap_x_from_real_1111.py と同じ模型の点・同じ距離の取り方。

枠・壁: 開口の縁（y = ±(幅/2 − 0.002)）から COVER_W は枠（前の面 --frame-x）、その外は壁（--wall-x）。2 つの箱（上から見た長方形）への
距離（箱の中は負）。縁の内でも枠の横の面に近ければ数える（付け根の近くを押す手も入る）。

  python gap_x_sim_1111.py <records/...> [--frame-x -0.05 --wall-x -0.03] [--cover-w 0.04]
"""
from __future__ import annotations

import argparse
import glob
import os
from typing import List

import numpy as np

import g1_arm_fk as fk

XS_CM = (2, 3, 4, 5, 6, 7, 8, 9, 10)
DEPTH = 0.30  # 箱の奥行き（前の面から奥へ）。距離の計算に使うだけ


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_dist(px: np.ndarray, py: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    """上から見た長方形 [x0,x1]×[y0,y1] までの符号つき距離（中は負）"""
    dx = np.maximum(x0 - px, px - x1)
    dy = np.maximum(y0 - py, py - y1)
    out = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    inside = np.minimum(np.maximum(dx, dy), 0)
    return out + inside


def gap_to_walls(P: np.ndarray, edge: float, frame_x: float, wall_x: float, cover_w: float) -> np.ndarray:
    """P [..., 3]（扉の座標）→ 両側の枠・壁までの距離の最小"""
    px, py = P[..., 0], P[..., 1]
    ds = []
    for s in (1.0, -1.0):
        y = s * py
        ds.append(box_dist(px, y, frame_x, frame_x + DEPTH, edge, edge + cover_w))
        ds.append(box_dist(px, y, wall_x, wall_x + DEPTH, edge + cover_w, edge + 5.0))
    return np.min(np.stack(ds), axis=0)


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("records")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--frame-x", type=float, default=-0.05)
    ap.add_argument("--wall-x", type=float, default=-0.03)
    ap.add_argument("--cover-w", type=float, default=0.04)
    a = ap.parse_args(argv)
    chain = fk.load_chain(a.urdf)
    st = np.load(os.path.join(a.records, "static.npz"), allow_pickle=True)
    names = [str(n) for n in st["dof_names"]]
    order = [names.index(n) for n in fk.JOINT_ORDER]
    files = sorted(glob.glob(os.path.join(a.records, "states_*.npz")))
    keys = ("root_pos", "root_quat_xyzw", "dof_pos", "episode_length", "hand_lever_force", "door_dof_pos")
    Z = {k: np.concatenate([np.load(f)[k] for f in files]) for k in keys}
    T, N = Z["episode_length"].shape
    pre, whole, ok = [], [], []
    for e in range(N):
        ep = Z["episode_length"][:, e]
        end = np.nonzero(np.diff(ep) < 0)[0]
        Lk = int(end[0] + 1) if len(end) else T
        f = np.linalg.norm(Z["hand_lever_force"][:Lk, e], axis=-1)
        kc = np.flatnonzero(f > 1.0)
        k_hi = int(kc[0]) if len(kc) else Lk - 1
        edge = float(st["door_width"][e]) / 2 - 0.002
        g = []
        for k in range(Lk):
            q = Z["dof_pos"][k, e][order]
            R = quat_xyzw_to_matrix(Z["root_quat_xyzw"][k, e])
            p = chain.points(q)
            pts = []
            for side in ("L", "R"):
                pw = {n: Z["root_pos"][k, e] + R @ p[side + n] for n in ("指先", "手", "手首", "肘")}
                pts += [pw["指先"], pw["手"], pw["手首"], (pw["手首"] + pw["肘"]) / 2, pw["肘"]]
            g.append(gap_to_walls(np.array(pts), edge, a.frame_x, a.wall_x, a.cover_w).min())
        g = np.array(g)
        pre.append(g[: k_hi + 1].min())
        whole.append(g.min())
        ok.append(np.degrees(Z["door_dof_pos"][:Lk, e, 0]).max() >= 3.0)
    pre, whole, ok = np.array(pre), np.array(whole), np.array(ok)
    print(f"[gap_x_sim] {a.records} 台 {N}・成功 {ok.sum()}・枠 x {a.frame_x}・壁 x {a.wall_x}・枠の幅 {a.cover_w}")
    print(f"  一番近い所（触れるまで）[cm]: 10% {100 * np.percentile(pre, 10):.1f}・中央 {100 * np.median(pre):.1f}・90% {100 * np.percentile(pre, 90):.1f}")
    print(f"  一番近い所（回の終わりまで）[cm]: 10% {100 * np.percentile(whole, 10):.1f}・中央 {100 * np.median(whole):.1f}")
    print("  X [cm]\t触れるまでに入る回\tその回の成功\t入らない回の成功\t回の終わりまでに入る回")
    for x in XS_CM:
        m = pre < x / 100
        mw = whole < x / 100
        print(f"  {x}\t{100 * m.mean():.0f}%\t{100 * ok[m].mean() if m.any() else float('nan'):.0f}%\t"
              f"{100 * ok[~m].mean() if (~m).any() else float('nan'):.0f}%\t{100 * mw.mean():.0f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
