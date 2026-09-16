#!/usr/bin/env python3
"""段 4 の登録結果を地図の上に描く。

数字の表だけでは「どちら向きに食い違っているか」が分からない。
地点ごとの推定を線で結べば、測位の誤差が**どの方向に出ているか**が見える。

    Navigation/.venv/bin/python render_registration.py tag_registry.json \\
        --map nav_map_run.yaml --out /tmp/registration.png
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42
import matplotlib.pyplot as plt
from jp_font import japanese_font

SESSION_COLORS = ["tab:blue", "tab:green", "tab:purple", "tab:brown", "tab:cyan"]


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as handle:
        if handle.readline().strip() != b"P5":
            raise SystemExit(f"P5 の PGM ではない: {path}")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        handle.readline()
        data = np.frombuffer(handle.read(), dtype=np.uint8)
    return data[:width * height].reshape(height, width)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("registry")
    parser.add_argument("--map", required=True, help="nav_map_run.yaml")
    parser.add_argument("--out", default="registration.png")
    parser.add_argument("--pad", type=float, default=1.2, help="描く範囲の余白 [m]")
    parser.add_argument("--whole-map", action="store_true",
                        help="タグの周りではなく地図全体を描く")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--size", type=float, nargs=2, default=[12.5, 10.5],
                        help="図の大きさ [inch]")
    arguments = parser.parse_args()

    japanese_font()
    registry = json.loads(Path(arguments.registry).read_text())
    map_path = Path(arguments.map)
    meta = yaml.safe_load(map_path.read_text())
    grid = read_pgm(map_path.parent / meta["image"])
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = grid.shape
    extent = (origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution)

    figure, axes = plt.subplots(figsize=tuple(arguments.size))
    # 矢印や格子の大きさを、描く範囲に合わせる
    if arguments.whole_map:
        # 占有セルの分位で切る（遠くに散った観測を落とす。render_map_grid.py と同じ手）
        rows_occupied, columns_occupied = np.nonzero(grid < 100)
        column_low, column_high = np.percentile(columns_occupied, [0.7, 99.3])
        row_low, row_high = np.percentile(rows_occupied, [0.7, 99.3])
        view = (origin_x + column_low * resolution - 2.0,
                origin_x + column_high * resolution + 2.0,
                origin_y + (height - row_high) * resolution - 2.0,
                origin_y + (height - row_low) * resolution + 2.0)
    else:
        view = None
    axes.imshow(np.flipud(grid), cmap="gray", origin="lower", extent=extent,
                vmin=0, vmax=255, interpolation="nearest")

    points = []
    for index, session in enumerate(registry["sessions"]):
        color = SESSION_COLORS[index % len(SESSION_COLORS)]
        x, y = session["robot_position"][0], session["robot_position"][1]
        yaw = np.radians(session["robot_yaw_deg"])
        points.append((x, y))
        axes.plot([x], [y], "o", ms=9, color=color, zorder=5)
        arrow_length = (view[1] - view[0]) * 0.035 if view else 0.55
        axes.annotate("", xy=(x + arrow_length * np.cos(yaw),
                              y + arrow_length * np.sin(yaw)),
                      xytext=(x, y), zorder=5,
                      arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8,
                                      mutation_scale=14))
        if not view:
            axes.annotate(f"{session['label']}  {session['robot_yaw_deg']:+.0f}°", (x, y),
                          textcoords="offset points", xytext=(10, -14), fontsize=9,
                          color=color, zorder=6)
        for tag_id, position in session.get("tags", {}).items():
            points.append((position[0], position[1]))
            axes.plot([position[0]], [position[1]], "x", ms=8, mew=1.8, color=color,
                      zorder=6)
            merged = registry["tags"][tag_id]["position"]
            axes.plot([position[0], merged[0]], [position[1], merged[1]],
                      "-", lw=0.9, color=color, alpha=0.65, zorder=4)

    lines = []
    for tag_id, entry in sorted(registry["tags"].items(), key=lambda kv: int(kv[0])):
        estimates = np.array([session["tags"][tag_id]
                              for session in registry["sessions"]
                              if tag_id in session.get("tags", {})])
        worst = 0.0
        if len(estimates) > 1:
            worst = float(np.linalg.norm(
                estimates[:, :2] - estimates[:, :2].mean(axis=0), axis=1).max() * 1000)
        lines.append(f"ID{tag_id}: ({entry['position'][0]:.3f}, {entry['position'][1]:.3f}, "
                     f"z {entry['position'][2]:+.3f})  地点間 {worst:.0f} mm")
        x, y = entry["position"][0], entry["position"][1]
        points.append((x, y))
        axes.plot([x], [y], "*", ms=20, color="tab:red", mec="black", mew=0.6, zorder=7)
        if not view:
            axes.annotate(f"ID{tag_id}\n({x:.2f}, {y:.2f})", (x, y),
                          textcoords="offset points", xytext=(12, 10), fontsize=10,
                          color="tab:red", weight="bold", zorder=8)

    array = np.array(points)
    if view:
        axes.set_xlim(view[0], view[1])
        axes.set_ylim(view[2], view[3])
        # 全体図ではタグが小さすぎるので、どこに在るかを枠で示す
        margin = 1.6
        low_x, high_x = array[:, 0].min() - margin, array[:, 0].max() + margin
        low_y, high_y = array[:, 1].min() - margin, array[:, 1].max() + margin
        axes.plot([low_x, high_x, high_x, low_x, low_x],
                  [low_y, low_y, high_y, high_y, low_y],
                  "-", color="tab:red", lw=1.8, zorder=9)
        axes.annotate("タグと 3 地点", ((low_x + high_x) / 2, high_y),
                      textcoords="offset points", xytext=(0, 8), ha="center",
                      fontsize=11, color="tab:red", weight="bold", zorder=9)
    else:
        axes.set_xlim(array[:, 0].min() - arguments.pad, array[:, 0].max() + arguments.pad)
        axes.set_ylim(array[:, 1].min() - arguments.pad, array[:, 1].max() + arguments.pad)
    grid_steps = ((5.0, 0.2), (10.0, 0.5)) if view else ((1.0, 0.18), (5.0, 0.5))
    for step, alpha in grid_steps:
        for value in np.arange(np.floor(axes.get_xlim()[0]), axes.get_xlim()[1], step):
            axes.axvline(value, color="tab:blue", lw=0.4, alpha=alpha)
        for value in np.arange(np.floor(axes.get_ylim()[0]), axes.get_ylim()[1], step):
            axes.axhline(value, color="tab:blue", lw=0.4, alpha=alpha)

    axes.plot([], [], "*", ms=16, color="tab:red", mec="black", label="登録したタグ（3 地点の平均）")
    axes.plot([], [], "x", ms=8, color="0.35", label="地点ごとの推定（線は平均との差）")
    axes.plot([], [], "o", ms=9, color="0.35", label="ロボット（矢印は向き）")
    if len(registry["tags"]) >= 2:
        keys = sorted(registry["tags"], key=int)
        first = np.array(registry["tags"][keys[0]]["position"])
        second = np.array(registry["tags"][keys[1]]["position"])
        lines.append(f"タグ間距離 {np.linalg.norm(first - second):.3f} m")
    axes.text(0.015, 0.015, "\n".join(lines), transform=axes.transAxes, fontsize=9,
              va="bottom", ha="left",
              bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="0.6", alpha=0.93))
    axes.legend(loc="upper left", fontsize=9, framealpha=0.92)
    axes.set_xlabel("map x [m]")
    axes.set_ylabel("map y [m]")
    axes.set_title("段 4 タグの地図登録 — 細い格子 1 m / 太い格子 5 m", fontsize=12)
    figure.tight_layout()
    figure.savefig(arguments.out, dpi=arguments.dpi)
    print(f"書いた: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
