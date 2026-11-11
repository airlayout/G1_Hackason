"""11-10: 深度から閉じた扉の開口の左右の縁を拾い、ライダーの正解の「左」を回ごとに確かめる（答え合わせの計器。方策には入れない）。

実物の Room 4 の扉は、枠か壁が板より 1.5〜2 cm 手前に出ている（作業ログ 16 章）。深度の点に板の面を当て、板の面にある点が
扉と平行な向きにどこまで続くかで開口の縁を決める（板より 1〜3.5 cm 手前の点が縁の外にあることも確かめる）。
縁の真ん中は、カメラから見た開口の中心の横の位置になる。カメラの pelvis からの横の取付は、巻き尺で確かめた回の地点（--calib）で
「深度のカメラの横 − 正解の pelvis の横」として出し、ほかの回（--test）で正解の pelvis の横と向きから深度のカメラの横を見込み、
測った値と比べる。差が正解の「左」の誤り（深度の測り方は向きに依らないとみなす）。

使い方（OMEN の real_ana_1106/）:
  python ana_depth_edges_1110.py --calib <vision>:<truth>:t0:t1 --test <vision>:<truth>:t0:t1 [--test ...]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ana_depth_yolo_1110 as ady  # noqa: E402

SLAB_TOL_M = 0.006  # 板の面とみなす距離
FRONT_M = (0.010, 0.035)  # 枠・壁とみなす板より手前の距離
CAM_X_M = 0.0576  # カメラの pelvis からの前（シムの取付。向きで横に回り込む分だけに使う）
MIN_WIDTH_M, MAX_WIDTH_M = 0.80, 1.10


def edges(depth: np.ndarray, rng: np.random.Generator) -> Optional[Tuple[float, float]]:
    """1 枚の深度 → (カメラから見た開口の中心の左 [m], 開口の幅 [m])。拾えなければ None"""
    P = ady.backproject(depth.astype(np.float32))
    n = c = None
    rest = P
    for _ in range(3):  # 板の面（縦・法線の y が正）を、細かい線で当てる
        r = ady.fit_plane(rest, SLAB_TOL_M, rng)
        if r is None:
            return None
        if r[0][1] > 0.2:
            n, c = r[0], r[1]
            break
        rest = rest[~r[2]]
    if n is None:
        return None
    s = P @ n + c  # カメラの側が正
    v = np.array([0.0, 1.0, 0.0]) - n[1] * n  # 面の中の下向き
    v /= np.linalg.norm(v)
    u = np.cross(n, v)  # 面の中の横
    if u[0] > 0:
        u = -u  # 左（光学の x の逆）を正に
    pu, pv = P @ u, P @ v
    slab = np.abs(s) < SLAB_TOL_M
    if slab.sum() < 200:
        return None
    v_lo, v_hi = np.percentile(pv[slab], [5, 60])  # 板の上の方（床との境と、近くの床を避ける）
    band = (pv > v_lo) & (pv < v_hi)
    su = pu[slab & band]
    fu = pu[(s > FRONT_M[0]) & (s < FRONT_M[1]) & band]
    if len(su) < 100 or len(fu) < 30:
        return None
    left, right = np.percentile(su, 99), np.percentile(su, 1)
    if not ((fu > left).sum() > 10 and (fu < right).sum() > 10):  # 両側に枠・壁が写っていなければ縁ではない（画面の端）
        return None
    w = float(left - right)
    if not MIN_WIDTH_M <= w <= MAX_WIDTH_M:
        return None
    return float((left + right) / 2.0), w


def measure(spec: str, every: int, rng: np.random.Generator):
    """vision:truth:t0:t1 → (枚ごとの [カメラから見た中心の左, 幅, 正解の pelvis の横（扉の座標）, 正解の向き])"""
    vis, truth, t0, t1 = spec.split(":")
    zv = np.load(vis, allow_pickle=True)
    zt = np.load(truth)
    t, pose = zv["t_capture"], zt["pose"]
    js = np.nonzero((t >= float(t0)) & (t <= float(t1)) & np.isfinite(pose[:, 0]))[0][::every]
    depth = zv["depth"]
    rows: List[Tuple[float, float, float, float]] = []
    for j in js:
        e = edges(depth[j], rng)
        if e is not None:
            rows.append((e[0], e[1], float(pose[j, 1]), float(pose[j, 2])))
    return np.array(rows), len(js)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--test", action="append", default=[])
    ap.add_argument("--every", type=int, default=10)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(0)
    r, n = measure(a.calib, a.every, rng)
    if not len(r):
        raise SystemExit("[edges] 取付を出す地点で縁が拾えない")
    # 扉の座標でのカメラの横 = −（カメラから見た中心の左）。取付の横 = カメラの横 − pelvis の横 − 前の取付が向きで回り込む分
    cam_y = -r[:, 0]
    off = np.median(cam_y - r[:, 2] - CAM_X_M * np.sin(r[:, 3]))
    print(f"[edges] 取付: {len(r)}/{n} 枚で縁を拾えた・幅 {np.median(r[:, 1]):.3f} m（巻き尺 0.94）・カメラから見た中心の左 "
          f"{np.median(r[:, 0]):+.3f} m・正解の pelvis の横 {np.median(r[:, 2]):+.3f} m → カメラは pelvis の左 {off:+.3f} m", flush=True)
    for spec in a.test:
        r, n = measure(spec, a.every, rng)
        if not len(r):
            print(f"[edges] {spec}: 縁が拾えない（{n} 枚）", flush=True)
            continue
        want = -(r[:, 2] + CAM_X_M * np.sin(r[:, 3]) + off * np.cos(r[:, 3]))  # 正解から見込むカメラから見た中心の左
        d = r[:, 0] - want
        print(f"[edges] {spec.split(':')[0].split('/')[-1]} t {spec.split(':')[2]}〜{spec.split(':')[3]}: {len(r)}/{n} 枚・幅 {np.median(r[:, 1]):.3f} m・"
              f"向き {math.degrees(np.median(r[:, 3])):+.1f}°・カメラから見た中心の左 深度 {np.median(r[:, 0]):+.3f} m／正解から見込む {np.median(want):+.3f} m"
              f" → 深度 − 正解 {100 * np.median(d):+.1f} cm（揺れ {100 * np.std(d):.1f} cm）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
