#!/usr/bin/env python3
"""同じスキャンを複数の姿勢で地図に重ね、**目で選べる形**に並べる。

## なぜ要るか

重畳率や完全一致率は、地図の込み具合に引きずられる。`nav_map_run` は既知セルの
24.3 % が占有なので、**でたらめな姿勢でも 42 % 前後の重畳が出る**。
数字だけ見て閾値を決めると、この下駄を踏む。

だから**人が見て決められる図**を出す。スキャンが壁の線に乗っているかどうかは、
数字より目のほうが確実である。

## 使い方

    Navigation/.venv/bin/python render_pose_compare.py nav_map_run.yaml \\
        --case scanP.yaml regP AMCL 5.978 1.252 -139.66 \\
        --case scanP.yaml regP 探索 0.220 0.010 15.40 \\
        --out /tmp/compare.png
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42
import matplotlib.pyplot as plt
from jp_font import japanese_font
from check_scan_overlay import first_document, load_scan, read_pgm, score


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("--case", action="append", nargs=6, required=True,
                        metavar=("SCAN", "PLACE", "LABEL", "X", "Y", "YAW_DEG"))
    parser.add_argument("--out", default="pose_compare.png")
    parser.add_argument("--span", type=float, default=11.0, help="各パネルの半幅 [m]")
    parser.add_argument("--tolerances", type=int, nargs="+", default=[0, 1, 2, 3],
                        help="重畳を出すときの許容セル数")
    parser.add_argument("--dpi", type=int, default=135)
    arguments = parser.parse_args()

    japanese_font()
    map_path = Path(arguments.map_yaml)
    meta = yaml.safe_load(map_path.read_text())
    grid = read_pgm(map_path.parent / meta["image"])
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = grid.shape
    occupied = grid < 100
    extent = (origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution)
    chance = 100.0 * occupied.sum() / (occupied.sum() + (grid > 210).sum())

    places = []
    for case in arguments.case:
        if case[1] not in places:
            places.append(case[1])
    labels = []
    for case in arguments.case:
        if case[2] not in labels:
            labels.append(case[2])

    figure, axes_grid = plt.subplots(len(labels), len(places),
                                     figsize=(5.6 * len(places), 5.4 * len(labels)),
                                     squeeze=False)
    for case in arguments.case:
        scan_path, place, label = case[0], case[1], case[2]
        x, y, yaw = float(case[3]), float(case[4]), math.radians(float(case[5]))
        axes = axes_grid[labels.index(label)][places.index(place)]
        angles, ranges, _ = load_scan(Path(scan_path))
        local_x, local_y = ranges * np.cos(angles), ranges * np.sin(angles)
        map_x = x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        map_y = y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        columns = np.round((map_x - origin_x) / resolution).astype(int)
        rows = np.round((height - 1) - (map_y - origin_y) / resolution).astype(int)

        axes.imshow(np.flipud(grid), cmap="gray", origin="lower", extent=extent,
                    vmin=0, vmax=255, interpolation="nearest")
        axes.plot(map_x, map_y, ".", ms=2.4, color="tab:red", zorder=4)
        axes.plot([x], [y], "o", ms=9, color="tab:blue", zorder=5)
        axes.annotate("", xy=(x + 1.1 * math.cos(yaw), y + 1.1 * math.sin(yaw)),
                      xytext=(x, y), zorder=5,
                      arrowprops=dict(arrowstyle="-|>", color="tab:blue", lw=2.2,
                                      mutation_scale=16))
        axes.set_xlim(x - arguments.span, x + arguments.span)
        axes.set_ylim(y - arguments.span, y + arguments.span)
        text = "  ".join(
            f"±{t}セル {score(occupied, columns, rows, t):.0f}%"
            for t in arguments.tolerances)
        axes.set_title(f"{place} / {label}\n({x:.2f}, {y:.2f}, {math.degrees(yaw):+.1f}°)\n{text}",
                       fontsize=10)
        axes.tick_params(labelsize=8)

    figure.suptitle(
        f"同じスキャンを姿勢ちがいで重ねた図 — **赤い点が黒い線に乗っているか**で判断する"
        f"（この地図はでたらめな姿勢でも ±0セルで約 {chance:.0f} % 当たる）",
        fontsize=12)
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(arguments.out, dpi=arguments.dpi)
    print(f"書いた: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
