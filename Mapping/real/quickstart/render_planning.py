#!/usr/bin/env python3
"""check_planning.py --record の記録から、段 B の 8 回を動画にする。

左: 機体の周りのコストマップと、選ばれたゴール・引けた経路。
右: 8 回ぶんの計測値が 1 行ずつ埋まっていく表と、2 つの合否。

「経路が引けた」だけでなく **機体セルのコストが 0 である**ことが見えるように、
機体の下のセル（static / global / local）を毎回数字で出す。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Circle, FancyArrow

plt.rcParams["font.family"] = ["Hiragino Sans", "DejaVu Sans"]
# ⚠️ macOS に日本語の等幅フォントは無い。数字だけ等幅にし、日本語は
# Hiragino へフォールバックさせる（桁揃えは列位置で作る）。
plt.rcParams["font.monospace"] = ["Menlo", "Hiragino Sans", "DejaVu Sans Mono"]

INK, INK2, INK3 = "#111620", "#46505f", "#6b7686"
LINE = "#dce1e9"
ACCENT, GOOD, BAD, WARN = "#0f6fc4", "#14714a", "#a8202c", "#b1500f"
SURFACE = "#ffffff"

VIEW_HALF_X, VIEW_HALF_Y = 9.0, 6.0     # 表示する範囲（機体中心から ±m）
HOLD_FRAMES = 14                        # 経路を引き終えたあと止める枚数
DRAW_FRAMES = 12                        # 経路を描いていく枚数
FPS = 12


def costmap_cmap():
    """Nav2 のコストマップの色分け。0=自由 / 1-98=膨張 / 99=内接 / 100=致死 / -1=未知。"""
    colors = ["#ffffff",            # 0 自由
              "#dbe9f7", "#b9d5ef", "#93bde4", "#6aa2d8",   # 膨張（薄→濃）
              "#f0b429",            # 99 内接
              "#a8202c"]            # 100 致死
    bounds = [0, 1, 25, 50, 75, 99, 100, 101]
    return ListedColormap(colors), BoundaryNorm(bounds, len(colors))


def grid_extent(info: dict) -> tuple[float, float, float, float]:
    ox, oy = info["origin"]
    return (ox, ox + info["width"] * info["resolution"],
            oy, oy + info["height"] * info["resolution"])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True, help="planning.json のあるディレクトリ")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rec = Path(args.rec)
    meta = json.loads((rec / "planning.json").read_text())
    grids = np.load(rec / "planning_grids.npz")
    frames = meta["frames"]
    n = len(frames)

    static_info = meta["static_info"]
    static = grids["static"].reshape(static_info["height"], static_info["width"])

    cmap, norm = costmap_cmap()

    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=SURFACE)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.45, 1.0], left=0.035, right=0.985,
                          top=0.90, bottom=0.06, wspace=0.10)
    ax = fig.add_subplot(gs[0, 0])
    axt = fig.add_subplot(gs[0, 1]); axt.axis("off")

    fig.text(0.035, 0.955, "段 B：Nav2 が経路を引けるか（掃除済み地図 nav_map_clean・Isaac Sim）",
             fontsize=19, color=INK, weight="bold", va="center")
    fig.text(0.035, 0.918,
             "1 回ごとに姿勢を読み直し、機体の前方 3〜6 m のゴールへ ComputePathToPose を投げる",
             fontsize=12.5, color=INK2, va="center")

    writer = FFMpegWriter(fps=FPS, bitrate=3200,
                          metadata={"title": "段 B 経路計画 8 回"})
    rows_done: list[str] = []

    with writer.saving(fig, args.out, dpi=120):
        for idx, f in enumerate(frames):
            rx, ry, ryaw = f["robot"]
            info = f.get("global_costmap_info") or static_info
            key = f"costmap_{f['try']:02d}"
            g = (grids[key].reshape(info["height"], info["width"])
                 if key in grids.files else static)
            path = np.asarray(f["path"], dtype=float) if f["path"] else np.empty((0, 2))
            goal = f["goal"]

            for step in range(DRAW_FRAMES + HOLD_FRAMES):
                ax.clear()
                ax.set_facecolor("#f6f8fa")
                # コストマップ（-1 は白扱いにして自由と区別しないほうが読みやすい）
                shown = np.where(g < 0, 0, g)
                ax.imshow(shown, origin="lower", extent=grid_extent(info),
                          cmap=cmap, norm=norm, interpolation="nearest")
                # 事前地図の占有だけを黒く重ねる（どれが壁・机かを示す）
                occ = np.ma.masked_where(static < 90, static)
                ax.imshow(occ, origin="lower", extent=grid_extent(static_info),
                          cmap=ListedColormap(["#2b3340"]), vmin=90, vmax=100,
                          interpolation="nearest", alpha=0.9)

                ax.set_xlim(rx - VIEW_HALF_X, rx + VIEW_HALF_X)
                ax.set_ylim(ry - VIEW_HALF_Y, ry + VIEW_HALF_Y)
                ax.set_aspect("equal")
                ax.tick_params(labelsize=9, colors=INK3)
                for s in ax.spines.values():
                    s.set_color(LINE)
                ax.set_xlabel("map x [m]", fontsize=10, color=INK3)
                ax.set_ylabel("map y [m]", fontsize=10, color=INK3)

                # 経路（少しずつ引かれる）
                if len(path):
                    k = min(len(path), max(2, int(len(path) * (step + 1) / DRAW_FRAMES)))
                    ax.plot(path[:k, 0], path[:k, 1], color=GOOD, lw=3.4,
                            solid_capstyle="round", zorder=5)

                # ゴール
                if goal:
                    gx, gy, dist, bearing, gv = goal
                    ax.plot([gx], [gy], marker="*", ms=22, color=WARN,
                            markeredgecolor="white", markeredgewidth=1.2, zorder=6)
                    ax.annotate(f"ゴール {dist:.0f} m / {bearing:+.0f}°",
                                (gx, gy), textcoords="offset points", xytext=(12, 10),
                                fontsize=11, color=WARN, weight="bold", zorder=7)

                # 機体（robot_radius の円と向き）
                ax.add_patch(Circle((rx, ry), meta["robot_radius"], fill=False,
                                    ec=ACCENT, lw=2.2, zorder=8))
                ax.add_patch(FancyArrow(rx, ry, 0.9 * np.cos(ryaw), 0.9 * np.sin(ryaw),
                                        width=0.06, head_width=0.26, head_length=0.28,
                                        color=ACCENT, zorder=9))
                ax.plot([rx], [ry], "o", ms=5, color=ACCENT, zorder=10)

                # 凡例（コストマップの色が何を意味するか）
                leg = [("#ffffff", "自由 0"), ("#93bde4", "膨張"),
                       ("#f0b429", "内接 99"), ("#a8202c", "致死 100"),
                       ("#2b3340", "事前地図の占有")]
                for li, (col, lab) in enumerate(leg):
                    ax.add_patch(plt.Rectangle(
                        (0.012 + li * 0.145, 0.026), 0.022, 0.030,
                        transform=ax.transAxes, facecolor=col,
                        edgecolor="#8d97a5", lw=0.8, zorder=20))
                    ax.text(0.040 + li * 0.145, 0.041, lab, transform=ax.transAxes,
                            fontsize=10, color=INK, va="center", zorder=20,
                            bbox=dict(fc="white", ec="none", alpha=0.75, pad=1.0))

                cell = (f"機体の下のセル  static {f['cost_static']} / "
                        f"global {f['cost_global']} / local {f['cost_local']}")
                ax.set_title(f"{f['try']} 回目 / {n}    機体 ({rx:+.2f}, {ry:+.2f})    {cell}",
                             fontsize=13.5, color=INK, pad=10, loc="left")

                # ── 右の表 ─────────────────────────────
                axt.clear(); axt.axis("off")
                axt.set_xlim(0, 1); axt.set_ylim(0, 1)
                COLS = [(0.00, "回", "left"), (0.11, "機体の下のセル", "left"),
                        (0.46, "ゴール", "left"), (0.68, "結果", "left"),
                        (0.83, "経路長", "left")]
                axt.text(0.0, 0.975, "計測値", fontsize=13.5, color=INK3, weight="bold")
                for x, label, ha in COLS:
                    axt.text(x, 0.935, label, fontsize=11.5, color=INK3, ha=ha)
                axt.plot([0, 1], [0.921, 0.921], color=LINE, lw=1)

                d = (np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])).sum()
                     if len(path) > 1 else 0.0)
                cur = {
                    "try": f["try"],
                    "cell": f"{f['cost_static']} / {f['cost_global']} / {f['cost_local']}",
                    "goal": f"{goal[2]:.0f} m / {goal[3]:+.0f}°" if goal else "—",
                    "ok": f["ok"],
                    "len": f"{d:.2f} m",
                }
                rows = rows_done + [cur]
                for i, r in enumerate(rows):
                    y = 0.888 - i * 0.045
                    is_cur = (i == len(rows) - 1)
                    c = INK if is_cur else INK2
                    wt = "bold" if is_cur else "normal"
                    if is_cur:
                        axt.add_patch(plt.Rectangle((-0.01, y - 0.012), 1.02, 0.036,
                                                    color="#e7f1fb", zorder=0))
                    axt.text(COLS[0][0], y, str(r["try"]), fontsize=12, color=c,
                             weight=wt, family="monospace")
                    axt.text(COLS[1][0], y, r["cell"], fontsize=12,
                             color=GOOD if r["cell"].startswith("0 / 0 / 0") else c,
                             weight="bold" if r["cell"].startswith("0 / 0 / 0") else wt,
                             family="monospace")
                    axt.text(COLS[2][0], y, r["goal"], fontsize=12, color=c,
                             weight=wt, family="monospace")
                    axt.text(COLS[3][0], y, "成功" if r["ok"] else "失敗", fontsize=12,
                             color=GOOD if r["ok"] else BAD, weight="bold")
                    axt.text(COLS[4][0], y, r["len"], fontsize=12, color=c,
                             weight=wt, family="monospace")

                # 合否（ここまでの集計）
                done = frames[:idx + 1]
                n_ok = sum(1 for x in done if x["ok"])
                n_cell = sum(1 for x in done
                             if x["cost_global"] is not None and x["cost_global"] < 99)
                y0 = 0.888 - len(rows) * 0.045 - 0.045
                axt.plot([0, 1], [y0 + 0.035, y0 + 0.035], color=LINE, lw=1.4)
                axt.text(0.0, y0 - 0.012, "合否（段 B）", fontsize=13.5, color=INK3,
                         weight="bold")
                for j, (label, val, tot) in enumerate([
                        ("3〜6 m 先へ経路が引ける", n_ok, len(done)),
                        ("機体セルが inscribed(99) 未満", n_cell, len(done))]):
                    ok = val == tot
                    yy = y0 - 0.068 - j * 0.052
                    axt.text(0.0, yy, "PASS" if ok else "FAIL", fontsize=13,
                             color=GOOD if ok else BAD, weight="bold",
                             family="monospace")
                    axt.text(0.13, yy, label, fontsize=12.5, color=INK)
                    axt.text(0.99, yy, f"{val}/{tot}", fontsize=13, ha="right",
                             color=GOOD if ok else BAD, weight="bold",
                             family="monospace")

                axt.text(0.0, 0.03,
                         "旧地図では経路 2/30 → 13/30、機体セルのコスト中央値 99.0。\n"
                         "経路長 ÷ 直線距離も中央 13.9 倍 → 1.00 倍になった。",
                         fontsize=11.5, color=INK3, va="bottom")

                writer.grab_frame()

            rows_done.append(cur)

    print(f"[OK] -> {args.out}")


if __name__ == "__main__":
    main()
