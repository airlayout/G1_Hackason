#!/usr/bin/env python3
"""nav_map を軌跡つきの PNG にする。地図を作り直したときに「何が消えたか」を見る。

## なぜ要るか

`check_map_clearance.py` は「軌跡の 32.9% が 0.30 m 未満」までは言えるが、
**壁に穴が空いたのか、経路脇の散らかりが消えたのか**は数字から分からない。
占有セル数が増えていても、壁を消して床の粒を増やしていれば同じ数字になる。

## 何を描くか

- 左: 比較元の地図（`--baseline`）
- 中: 対象の地図
- 右: 差分。**赤 = 消えたセル / 緑 = 増えたセル**

軌跡は占有セルまでの距離で色を変える（赤 < robot_radius ≤ 青）。
`--crop x y w h`（世界座標[m]）で拡大できる。

## 使い方

    ../../G1_Hackason/.venv/bin/python quickstart/render_nav_map.py \\
        runs/<id>/map/nav_map_try40 runs/<id>/mola_floor0/traj.txt \\
        --baseline runs/<id>/map/old/nav_map --out /tmp/compare.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_map_clearance import (DEFAULT_ROBOT_RADIUS, distance_to_occupied,  # noqa: E402
                                 load_map, read_trajectory)

# 占有=黒 / 空き=白 / 未知=灰
SHADE_OCCUPIED, SHADE_FREE, SHADE_UNKNOWN = 0.0, 1.0, 0.75
DPI = 150


def as_image(grid) -> np.ndarray:
    shade = np.full(grid.shape, SHADE_UNKNOWN)
    shade[grid.free] = SHADE_FREE
    shade[grid.occupied] = SHADE_OCCUPIED
    return shade


def extent(grid) -> "tuple[float, float, float, float]":
    rows, cols = grid.shape
    return (grid.origin[0], grid.origin[0] + cols * grid.resolution,
            grid.origin[1], grid.origin[1] + rows * grid.resolution)


def draw_trajectory(ax, grid, xy: np.ndarray, robot_radius: float) -> None:
    field = distance_to_occupied(grid)
    col, row, inside = grid.to_cells(xy)
    xy, col, row = xy[inside], col[inside], row[inside]
    tight = field[row, col] < robot_radius
    ax.plot(xy[~tight, 0], xy[~tight, 1], ".", ms=1.2, color="#1560d0",
            label="clearance ≥ {:.2f} m".format(robot_radius))
    ax.plot(xy[tight, 0], xy[tight, 1], ".", ms=1.6, color="#e01b24",
            label="clearance < {:.2f} m".format(robot_radius))


def align(grid, reference) -> np.ndarray:
    """`reference` の格子の上に `grid` の占有を載せ直す（原点も大きさも違うため）。"""
    rows, cols = reference.shape
    out = np.zeros((rows, cols), dtype=bool)
    ry, rx = np.nonzero(grid.occupied)
    x = (rx + 0.5) * grid.resolution + grid.origin[0]
    y = (ry + 0.5) * grid.resolution + grid.origin[1]
    c = np.floor((x - reference.origin[0]) / reference.resolution).astype(int)
    r = np.floor((y - reference.origin[1]) / reference.resolution).astype(int)
    ok = (c >= 0) & (c < cols) & (r >= 0) & (r < rows)
    out[r[ok], c[ok]] = True
    return out


def draw_difference(ax, grid, baseline) -> None:
    """比較元の格子の上で、消えたセルを赤・増えたセルを緑に塗る。"""
    current = align(grid, baseline)
    rgb = np.ones(baseline.shape + (3,))
    rgb[baseline.occupied & current] = (0.15, 0.15, 0.15)
    rgb[baseline.occupied & ~current] = (0.88, 0.11, 0.14)     # 消えた
    rgb[~baseline.occupied & current] = (0.15, 0.65, 0.25)     # 増えた
    ax.imshow(rgb, origin="lower", extent=extent(baseline), interpolation="nearest")
    lost = int((baseline.occupied & ~current).sum())
    gained = int((~baseline.occupied & current).sum())
    ax.set_title("diff:  red = lost {:,}  /  green = gained {:,}".format(lost, gained))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("map", type=Path)
    p.add_argument("traj", type=Path)
    p.add_argument("--baseline", type=Path)
    p.add_argument("--out", type=Path, default=Path("/tmp/nav_map.png"))
    p.add_argument("--robot-radius", type=float, default=DEFAULT_ROBOT_RADIUS)
    p.add_argument("--crop", type=float, nargs=4, metavar=("x", "y", "w", "h"))
    args = p.parse_args()

    xy = read_trajectory(args.traj)
    grid = load_map(args.map)
    grids = [(args.baseline.name, load_map(args.baseline))] if args.baseline else []
    grids.append((args.map.name, grid))

    panels = len(grids) + (1 if args.baseline else 0)
    fig, axes = plt.subplots(1, panels, figsize=(7.5 * panels, 7.0), squeeze=False)
    for ax, (name, g) in zip(axes[0], grids):
        ax.imshow(as_image(g), origin="lower", extent=extent(g),
                  cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        draw_trajectory(ax, g, xy, args.robot_radius)
        ax.set_title("{}   occupied {:,}".format(name, int(g.occupied.sum())))
    if args.baseline:
        draw_difference(axes[0][-1], grid, grids[0][1])

    for ax in axes[0]:
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        if args.crop:
            x, y, w, h = args.crop
            ax.set_xlim(x, x + w); ax.set_ylim(y, y + h)
    axes[0][0].legend(loc="lower right", markerscale=8, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=DPI)
    print("[OK] {}".format(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
