#!/usr/bin/env python3
"""`inflation_radius` を動かすと経路がどう変わるかを動画にする。

`tune_inflation.py` が出す数字（最狭部 ≒ min(inflation_radius, 最大ボトルネック)）を
目で見る形にする。⚠️ **これは事前地図だけの予測である。**
実際の `global_costmap` には `/scan` のライブ障害物層も乗るので、Isaac Sim で
同じ測定をやり直して確かめる必要がある。

左: 地図全体の上に、その設定で NavFn が選ぶ経路。狭い所を通る部分を赤で塗る。
右上: 1 セルの罰金が距離に対してどう付くか（設定を動かすと形が変わる）。
右下: 掃引の結果（最狭部と経路長）。いまの値に印を置く。

    .venv/bin/python Mapping/real/quickstart/render_inflation_sweep.py \\
        --out ../docs/作業ログ/video/2026-09-08_sim_inflation_sweep.mp4
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.colors import LinearSegmentedColormap, Normalize
from PIL import Image
from scipy import ndimage

R = Path("Mapping/real/runs/20260906T135940_UiS_room_v3")
plt.rcParams["font.family"] = ["Hiragino Sans", "DejaVu Sans"]
plt.rcParams["font.monospace"] = ["Menlo", "Hiragino Sans", "DejaVu Sans Mono"]
INK, INK2, INK3 = "#111620", "#46505f", "#6b7686"
LINE, SURFACE = "#dce1e9", "#ffffff"
GOOD, BAD, ACCENT = "#14714a", "#a8202c", "#0f6fc4"
HEIGHT_CMAP = LinearSegmentedColormap.from_list(
    "height", ["#e4e8ef", "#aab3c1", "#6b7686", "#3b4453", "#1c2430"])
COST_NEUTRAL, COST_FACTOR = 50.0, 0.8
LETHAL, INSCRIBED, INFLATED_MAX = 254.0, 253.0, 252.0
ROBOT_RADIUS = 0.25
SCALING = 5.0                # ⚠️ 触らない（1.0〜5.0 で結果が同じ。10.0 で悪化）
TIGHT_M = 0.50               # 「狭い」の基準
FPS = 4                      # 1 設定を 1 コマ以上見せたいのでゆっくり
HOLD = 6                     # 1 設定を何コマ見せるか
NB = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
      (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142))


def load_map():
    meta: dict[str, str] = {}
    for line in (R / "map/nav_map_clean.yaml").read_text().splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    img = np.asarray(Image.open(R / "map/nav_map_clean.pgm"), dtype=np.float64)
    occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
    return occ, ndimage.distance_transform_edt(~occ) * res, res, ox, oy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--scene", default=str(R / "sim/octomap_sim.npz"),
                    help="障害物の高さで色を付けるための npz（走行の動画と同じもの）")
    ap.add_argument("--from-wp", type=int, default=7)
    ap.add_argument("--to-wp", type=int, default=2)
    ap.add_argument("--values", type=float, nargs="*",
                    default=[0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70,
                             0.80, 1.00])
    args = ap.parse_args()

    occ, clear, res, ox, oy = load_map()
    h, w = occ.shape

    def cell(x: float, y: float) -> tuple[int, int]:
        return (h - 1 - int((y - oy) / res), int((x - ox) / res))

    def xy(c) -> tuple[float, float]:
        return (ox + c[1] * res, oy + (h - 1 - c[0]) * res)

    def cost_field(infl_r: float) -> np.ndarray:
        c = np.zeros_like(clear)
        c[occ] = LETHAL
        m = (~occ) & (clear <= ROBOT_RADIUS)
        c[m] = INSCRIBED
        m2 = (~occ) & (clear > ROBOT_RADIUS) & (clear <= infl_r)
        c[m2] = INFLATED_MAX * np.exp(-SCALING * (clear[m2] - ROBOT_RADIUS))
        return c

    def plan(a, b, cost) -> list:
        trav = COST_NEUTRAL + COST_FACTOR * cost
        dist = np.full(clear.shape, math.inf)
        prev: dict = {}
        dist[a] = 0.0
        pq = [(0.0, a)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == b:
                break
            for dr, dc, k in NB:
                v = (u[0] + dr, u[1] + dc)
                if not (0 <= v[0] < h and 0 <= v[1] < w) or occ[v]:
                    continue
                nd = d + k * trav[v]
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        path, c = [], b
        while c != a:
            path.append(c)
            c = prev[c]
        path.append(a)
        return path[::-1]

    def bottleneck(a, b, lo=0.10, hi=3.0) -> float:
        best = 0.0
        for _ in range(24):
            mid = (lo + hi) / 2
            lab, _ = ndimage.label(
                clear >= mid, structure=ndimage.generate_binary_structure(2, 1))
            if lab[a] and lab[a] == lab[b]:
                best, lo = mid, mid
            else:
                hi = mid
        return best

    wps = json.loads((R / "measure_20260908/waypoints.json").read_text())
    wa, wb = wps[args.from_wp], wps[args.to_wp]
    A, B = cell(wa["x"], wa["y"]), cell(wb["x"], wb["y"])
    bn = bottleneck(A, B)
    print(f"[route] wp{args.from_wp} → wp{args.to_wp}  "
          f"最大ボトルネック幅 {bn:.2f} m")

    # 各設定の経路をあらかじめ全部計算する（Dijkstra は 1 回 2〜3 秒）
    runs = []
    for ir in args.values:
        p = plan(A, B, cost_field(ir))
        cs = np.array([clear[c] for c in p])
        L = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(p, p[1:])) * res
        pts = np.array([xy(c) for c in p])
        runs.append({"ir": ir, "path": pts, "clear": cs, "len": L,
                     "min": float(cs.min()),
                     "tight": res * float((cs < TIGHT_M).sum())})
        print(f"  inflation_radius {ir:.2f} → 経路 {L:5.2f} m / "
              f"最狭部 {cs.min():.2f} m / 狭い所 {runs[-1]['tight']:.1f} m")

    # 障害物の高さ（走行の動画と同じ絵にする）
    d = np.load(args.scene)
    sc_occ, lvl, s_origin, s_cell = (d["occupied"], d["level"], d["origin"],
                                     float(d["cell"]))
    extent = (float(s_origin[0]), float(s_origin[0]) + sc_occ.shape[1] * s_cell,
              float(s_origin[1]), float(s_origin[1]) + sc_occ.shape[0] * s_cell)
    rows, cols = np.nonzero(sc_occ)
    box = (float(s_origin[0]) + cols.min() * s_cell,
           float(s_origin[0]) + (cols.max() + 1) * s_cell,
           float(s_origin[1]) + rows.min() * s_cell,
           float(s_origin[1]) + (rows.max() + 1) * s_cell)

    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0], height_ratios=[1, 1],
                          left=0.035, right=0.945, top=0.862, bottom=0.075,
                          wspace=0.16, hspace=0.34)
    ax = fig.add_subplot(gs[:, 0])
    ax_cost = fig.add_subplot(gs[0, 1])
    ax_sweep = fig.add_subplot(gs[1, 1])

    # 地図の表示範囲を軸の縦横比に合わせる（走行の動画と同じ約束）
    pos = ax.get_position()
    fw, fh = fig.get_size_inches()
    aspect = (pos.width * fw) / (pos.height * fh)
    x0, x1, y0, y1 = box[0] - 0.5, box[1] + 0.5, box[2] - 0.5, box[3] + 0.5
    bw, bh = x1 - x0, y1 - y0
    if bw / bh < aspect:
        bw = bh * aspect
    else:
        bh = bw / aspect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    xlim = (cx - bw / 2, cx + bw / 2)
    ylim = (cy - bh / 2, cy + bh / 2)

    fig.text(0.035, 0.955,
             "inflation_radius を動かすと Nav2 の経路がどう変わるか",
             fontsize=19, color=INK, weight="bold", va="center")
    fig.text(0.035, 0.917,
             f"wp{args.from_wp} → wp{args.to_wp}（直線 "
             f"{math.hypot(wa['x'] - wb['x'], wa['y'] - wb['y']):.2f} m）。"
             f"この区間で通れる一番広い道の幅は {bn:.2f} m。"
             f"⚠️ 事前地図だけの予測（ライブ障害物層は入っていない）",
             fontsize=12.5, color=INK2, va="center")

    writer = FFMpegWriter(fps=FPS, bitrate=3600,
                          metadata={"title": "inflation_radius の掃引"})
    dd = np.linspace(0.0, 1.2, 240)

    with writer.saving(fig, args.out, dpi=120):
        for k, run in enumerate(runs):
            ir = run["ir"]
            for _ in range(HOLD):
                # ── 左: 地図と経路 ──────────────────
                ax.clear()
                ax.set_facecolor("#f6f8fa")
                hh = np.ma.masked_where(~sc_occ, lvl)
                ax.imshow(hh, origin="lower", extent=extent, cmap=HEIGHT_CMAP,
                          norm=Normalize(0.0, 2.0), interpolation="nearest")
                ax.set_xlim(*xlim)
                ax.set_ylim(*ylim)
                ax.set_aspect("equal")
                ax.tick_params(labelsize=9, colors=INK3)
                for sp in ax.spines.values():
                    sp.set_color(LINE)
                ax.set_xlabel("map x [m]", fontsize=10, color=INK3)
                ax.set_ylabel("map y [m]", fontsize=10, color=INK3)

                p, cs = run["path"], run["clear"]
                ax.plot(p[:, 0], p[:, 1], color=ACCENT, lw=3.0, zorder=6,
                        label=f"経路（{run['len']:.2f} m）")
                tight = cs < TIGHT_M
                if tight.any():
                    ax.scatter(p[tight, 0], p[tight, 1], s=26, color=BAD,
                               zorder=7, edgecolors="none",
                               label=f"余裕 {TIGHT_M} m 未満（{run['tight']:.1f} m）")
                # 最狭部を指す
                imin = int(np.argmin(cs))
                ax.plot([p[imin, 0]], [p[imin, 1]], marker="o", ms=13,
                        mfc="none", mec=BAD, mew=2.4, zorder=9)
                ax.annotate(f"最狭部 {run['min']:.2f} m",
                            (p[imin, 0], p[imin, 1]),
                            textcoords="offset points", xytext=(14, 12),
                            fontsize=12, color=BAD, weight="bold", zorder=10,
                            bbox=dict(fc="white", ec=BAD, lw=1.0, alpha=0.92,
                                      pad=2.5))
                for wp, name, col in ((wa, f"wp{args.from_wp}", INK2),
                                      (wb, f"wp{args.to_wp}", GOOD)):
                    ax.plot([wp["x"]], [wp["y"]], marker="*", ms=20, color=col,
                            markeredgecolor="white", markeredgewidth=1.2,
                            zorder=8)
                    ax.annotate(name, (wp["x"], wp["y"]),
                                textcoords="offset points", xytext=(10, -16),
                                fontsize=11.5, color=col, weight="bold",
                                zorder=8)
                ax.legend(loc="upper left", fontsize=10.5, framealpha=0.92)
                ok = run["tight"] == 0.0
                ax.set_title(
                    f"inflation_radius = {ir:.2f} m"
                    f"    経路 {run['len']:.2f} m"
                    f"    最狭部 {run['min']:.2f} m"
                    + ("    ← 狭い所を通らない" if ok
                       else f"    ← 狭い所を {run['tight']:.1f} m 通る"),
                    fontsize=13.5, color=GOOD if ok else BAD, pad=10,
                    loc="left")

                # ── 右上: 罰金の形 ─────────────────
                ax_cost.clear()
                ax_cost.set_facecolor(SURFACE)
                cc = np.where(
                    dd <= ROBOT_RADIUS, INSCRIBED,
                    np.where(dd <= ir,
                             INFLATED_MAX * np.exp(-SCALING *
                                                   (dd - ROBOT_RADIUS)), 0.0))
                ax_cost.fill_between(dd, cc, color=ACCENT, alpha=0.16)
                ax_cost.plot(dd, cc, color=ACCENT, lw=2.4)
                ax_cost.axvline(ir, color=BAD, lw=1.6, ls="--")
                ax_cost.text(ir + 0.02, 235, f"inflation_radius {ir:.2f}",
                             fontsize=11, color=BAD, weight="bold")
                ax_cost.axvline(bn, color=GOOD, lw=1.4, ls=":")
                ax_cost.text(bn + 0.02, 150,
                             f"通れる一番広い道 {bn:.2f}", fontsize=10.5,
                             color=GOOD)
                ax_cost.set_xlim(0, 1.2)
                ax_cost.set_ylim(0, 265)
                ax_cost.set_title(
                    "① 1 セルの罰金。破線より右は 0 ＝ それ以上離れる動機が無い",
                    fontsize=11.5, color=INK, loc="left", pad=6)
                ax_cost.set_xlabel("障害物からの距離 [m]", fontsize=9.5,
                                   color=INK3)
                ax_cost.tick_params(labelsize=9.5, colors=INK3)
                for sp in ax_cost.spines.values():
                    sp.set_color(LINE)

                # ── 右下: 掃引の結果 ───────────────
                ax_sweep.clear()
                ax_sweep.set_facecolor(SURFACE)
                irs = [r["ir"] for r in runs]
                mins = [r["min"] for r in runs]
                lens = [r["len"] for r in runs]
                ax_sweep.plot(irs, mins, "-o", color=ACCENT, lw=2.2, ms=6,
                              label="選ばれた経路の最狭部 [m]")
                ax_sweep.axhline(bn, color=GOOD, lw=1.4, ls=":")
                ax_sweep.text(irs[0], bn + 0.015,
                              f"上限 {bn:.2f} m（部屋の側の限界）",
                              fontsize=10, color=GOOD)
                ax_sweep.plot([ir], [run["min"]], "o", ms=13, mfc="none",
                              mec=BAD, mew=2.4, zorder=6)
                ax_sweep.set_ylim(0.30, bn + 0.10)
                ax_sweep.set_xlabel("inflation_radius [m]", fontsize=9.5,
                                    color=INK3)
                ax_sweep.set_ylabel("最狭部 [m]", fontsize=9.5, color=ACCENT)
                ax_sweep.tick_params(labelsize=9.5, colors=INK3)
                for sp in ax_sweep.spines.values():
                    sp.set_color(LINE)
                ax2 = ax_sweep.twinx()
                ax2.plot(irs, lens, "--s", color=INK3, lw=1.6, ms=4.5,
                         label="経路長 [m]")
                ax2.set_ylabel("経路長 [m]", fontsize=9.5, color=INK3)
                ax2.tick_params(labelsize=9.5, colors=INK3)
                ax_sweep.set_title(
                    "② 最狭部 ≒ min(inflation_radius, 上限)。届いた後は長くなるだけ",
                    fontsize=11.5, color=INK, loc="left", pad=6)
                lines1, lab1 = ax_sweep.get_legend_handles_labels()
                lines2, lab2 = ax2.get_legend_handles_labels()
                ax_sweep.legend(lines1 + lines2, lab1 + lab2, loc="lower right",
                                fontsize=10, framealpha=0.9)
                writer.grab_frame()
                ax2.remove()

    print(f"[OK] -> {args.out}  （{len(runs)} 設定 / "
          f"{len(runs) * HOLD / FPS:.0f} 秒）")


if __name__ == "__main__":
    main()
