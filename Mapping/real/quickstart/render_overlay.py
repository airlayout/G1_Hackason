#!/usr/bin/env python3
"""重畳を**目で見る**。事前地図の上にライブ点群を、当たり/外れで色分けして描く。

## なぜ要るのか

`measure_overlay.py` は「70.1 %」としか言わない。**どこが外れているのか**は数字から
分からない。2026-09-12 に「静止 70.1 % / 歩行終端 53.0 % / 歩行後の静止 64.1 %」という
並びが出たとき、それが**部屋のどの部分の食い違いなのか**を見る手段が無かった。

`render_reloc_eval.py` は同じ絵を描くが、再定位の評価用 JSON に縛られていて流用できない。

## 何を描くか

パネルごとに:
  - 灰: 事前地図の占有セル
  - 緑: 占有セルに乗った点 / 赤: 外れた点
  - 青: そのときの `map -> base_link`（機体の位置）

## 使い方

    Navigation/.venv/bin/python quickstart/render_overlay.py \\
        runs/<S>/map/old/nav_map.yaml out.png \\
        runs/still_A/bag:all:歩行前の静止 \\
        runs/stage_B/bag:first5:歩行の最初 \\
        runs/stage_B/bag:last5:歩行の終端

パネルの指定は `<bag>:<切り取り>:<見出し>`。切り取りは `all` / `firstN` / `lastN`。
見出しは省略できる（フォルダ名になる）。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from jp_font import japanese_font
from measure_overlay import (read_map, read_bag, RANGE_MIN, RANGE_MAX,
                             WALL_Z_BELOW_SENSOR, WALL_Z_ABOVE_SENSOR)

MAX_POINTS_PER_PANEL = 120_000      # 描画が重くなるだけなので間引く
MARGIN_M = 1.5                      # 描画範囲は点群の外接矩形にこれだけ足す


def parse_panel(spec: str) -> tuple[Path, str, str]:
    """`<bag>:<切り取り>:<見出し>` を解く。見出しは省略可。"""
    parts = spec.split(":")
    bag = Path(parts[0])
    sl = parts[1] if len(parts) > 1 and parts[1] else "all"
    label = parts[2] if len(parts) > 2 and parts[2] else bag.parent.name
    return bag, sl, label


def scan_slice(scans: list, sl: str) -> list:
    if sl == "all":
        return scans
    if sl.startswith("first"):
        return scans[: int(sl[5:])]
    if sl.startswith("last"):
        return scans[-int(sl[4:]):]
    raise SystemExit("切り取りは all / firstN / lastN のどれか（受け取ったのは {}）".format(sl))


def panel_points(bag: Path, sl: str, occ, res, ox, oy, wall_z=None):
    """map 系に起こした壁帯の点と、占有セルに乗ったかの真偽、機体の軌跡を返す。"""
    tfs, scans, sensor_tf = read_bag(bag)
    if sensor_tf is None:
        s_t, s_R = np.zeros(3), np.eye(3)
    else:
        s_t, s_q = sensor_tf
        s_R = Rotation.from_quat(s_q).as_matrix()

    tt = tfs[:, 0]
    slerp = Slerp(tt, Rotation.from_quat(tfs[:, 4:8]))
    h, w = occ.shape

    pts, poses = [], []
    for ts, p in scan_slice(scans, sl):
        if not (tt[0] <= ts <= tt[-1]):
            continue
        p = p[np.isfinite(p).all(1)]
        d = np.linalg.norm(p, axis=1)
        p = p[(d > RANGE_MIN) & (d < RANGE_MAX)]
        t = np.array([np.interp(ts, tt, tfs[:, 1 + i]) for i in range(3)])
        R = slerp(ts).as_matrix()
        q = (R @ (s_R @ p.T + s_t[:, None])).T + t
        if wall_z is not None:
            z_lo, z_hi = wall_z
        else:
            sensor_z = float((R @ s_t + t)[2])
            z_lo = sensor_z + WALL_Z_BELOW_SENSOR
            z_hi = sensor_z + WALL_Z_ABOVE_SENSOR
        q = q[(q[:, 2] > z_lo) & (q[:, 2] < z_hi)]
        pts.append(q[:, :2])
        poses.append(t[:2])
    if not pts:
        raise SystemExit("{} に描ける点が無い".format(bag))

    xy = np.vstack(pts)
    if len(xy) > MAX_POINTS_PER_PANEL:
        idx = np.random.default_rng(0).choice(len(xy), MAX_POINTS_PER_PANEL, replace=False)
        xy = xy[idx]

    ix = np.floor((xy[:, 0] - ox) / res).astype(int)
    iy = np.floor((xy[:, 1] - oy) / res).astype(int)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    hit = np.zeros(len(xy), bool)
    hit[inside] = occ[h - 1 - iy[inside], ix[inside]]
    # 地図の外に落ちた点は「外れ」として扱う（measure_overlay は分母から外すので割合は別物）
    pct = 100.0 * hit[inside].sum() / max(inside.sum(), 1)
    return xy, hit, np.array(poses), pct


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("panels", nargs="+", help="<bag>:<切り取り>:<見出し>")
    ap.add_argument("--wall-z", nargs=2, type=float, metavar=("MIN", "MAX"), default=None,
                    help="壁とみなす高さ帯を map 系で固定する（既定はセンサ基準 -1.2..+0.6 m＝"
                         "床すれすれから。床の反射が外れとして数えられる）")
    ap.add_argument("--title", default=None)
    ap.add_argument("--dpi", type=int, default=130)
    a = ap.parse_args()

    japanese_font()
    occ, res, ox, oy = read_map(a.map_yaml)
    h, w = occ.shape
    oc_y, oc_x = np.nonzero(occ)
    map_x = ox + oc_x * res
    map_y = oy + (h - 1 - oc_y) * res

    panels = [parse_panel(s) for s in a.panels]
    data = []
    for bag, sl, label in panels:
        xy, hit, poses, pct = panel_points(bag, sl, occ, res, ox, oy, a.wall_z)
        data.append((label, xy, hit, poses, pct))
        print("{:<28} {:>6.1f} %  点 {:,}".format(label, pct, len(xy)))

    # 全パネルで同じ範囲にする（場所の差を目で比べられるように）
    allxy = np.vstack([d[1] for d in data])
    x0, x1 = allxy[:, 0].min() - MARGIN_M, allxy[:, 0].max() + MARGIN_M
    y0, y1 = allxy[:, 1].min() - MARGIN_M, allxy[:, 1].max() + MARGIN_M

    n = len(data)
    fig, axes = plt.subplots(1, n, figsize=(5.4 * n, 5.6), squeeze=False)
    for ax, (label, xy, hit, poses, pct) in zip(axes[0], data):
        ax.scatter(map_x, map_y, s=1.2, c="#c9ccd1", marker="s", linewidths=0)
        ax.scatter(xy[~hit, 0], xy[~hit, 1], s=0.35, c="#d9443f", linewidths=0)
        ax.scatter(xy[hit, 0], xy[hit, 1], s=0.35, c="#2f9e44", linewidths=0)
        ax.plot(poses[:, 0], poses[:, 1], "-", c="#1c7ed6", lw=2.0)
        ax.plot(poses[0, 0], poses[0, 1], "o", c="#1c7ed6", ms=6)
        ax.set_title("{}\n占有セルに乗った割合 {:.1f} %".format(label, pct), fontsize=11)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=8)
    if a.title:
        fig.suptitle(a.title, fontsize=13)
    fig.text(0.5, 0.01, "灰 = 事前地図の占有セル / 緑 = 乗った点 / 赤 = 外れた点 / 青 = 機体",
             ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.035, 1, 0.97 if a.title else 1.0))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=a.dpi)
    print("-> {}".format(a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
