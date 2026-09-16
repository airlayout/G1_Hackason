#!/usr/bin/env python3
"""いまの状態を 1 枚にまとめる —— 地図・タグ・ロボット・スキャンを重ねた図。

「合っているか」を人が目で確かめるための図。数字ではなく**配置が現実と合うか**を見る。

    Navigation/.venv/bin/python render_state.py nav_map_run.yaml \\
        --registry tag_registry.json \\
        --scan scan.yaml --pose タグから -0.214 -0.357 38.84 \\
        --pose 探索から -0.171 -0.328 38.50 --out /tmp/state.png
"""

from __future__ import annotations

import argparse
import math
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
from check_scan_overlay import load_scan, read_pgm

POSE_COLORS = ["tab:blue", "tab:green", "tab:orange"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("--registry", default=None)
    parser.add_argument("--scan", default=None)
    parser.add_argument("--pose", action="append", nargs=4, default=[],
                        metavar=("LABEL", "X", "Y", "YAW_DEG"))
    parser.add_argument("--span", type=float, default=4.0, help="描く範囲の半幅 [m]")
    parser.add_argument("--out", default="state.png")
    parser.add_argument("--dpi", type=int, default=150)
    arguments = parser.parse_args()

    japanese_font()
    map_path = Path(arguments.map_yaml)
    meta = yaml.safe_load(map_path.read_text())
    grid = read_pgm(map_path.parent / meta["image"])
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = grid.shape
    extent = (origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution)

    figure, axes = plt.subplots(figsize=(12.5, 11.0))
    axes.imshow(np.flipud(grid), cmap="gray", origin="lower", extent=extent,
                vmin=0, vmax=255, interpolation="nearest")

    focus = []
    if arguments.scan and arguments.pose:
        label, x, y, yaw_deg = arguments.pose[0]
        x, y, yaw = float(x), float(y), math.radians(float(yaw_deg))
        angles, ranges, _ = load_scan(Path(arguments.scan))
        local_x, local_y = ranges * np.cos(angles), ranges * np.sin(angles)
        axes.plot(x + local_x * math.cos(yaw) - local_y * math.sin(yaw),
                  y + local_x * math.sin(yaw) + local_y * math.cos(yaw),
                  ".", ms=3.2, color="tab:red", alpha=0.85, zorder=3,
                  label="いま LiDAR が見ているもの")

    if arguments.registry:
        import json
        registry = json.loads(Path(arguments.registry).read_text())
        for tag_id, entry in sorted(registry["tags"].items(), key=lambda kv: int(kv[0])):
            tx, ty = entry["position"][0], entry["position"][1]
            focus.append((tx, ty))
            axes.plot([tx], [ty], "*", ms=26, color="gold", mec="black", mew=1.2, zorder=8)
            axes.annotate(f"タグ ID{tag_id}\n({tx:.2f}, {ty:.2f})", (tx, ty),
                          textcoords="offset points", xytext=(14, 12), fontsize=11,
                          weight="bold", color="black", zorder=9,
                          bbox=dict(boxstyle="round,pad=0.3", fc="gold", ec="black",
                                    alpha=0.85))
        for session in registry.get("sessions", []):
            sx, sy = session["robot_position"][0], session["robot_position"][1]
            axes.plot([sx], [sy], "o", ms=7, mfc="none", mec="0.45", mew=1.5, zorder=4)
            axes.annotate(session["label"], (sx, sy), textcoords="offset points",
                          xytext=(7, -12), fontsize=8, color="0.45", zorder=4)

    for index, (label, x, y, yaw_deg) in enumerate(arguments.pose):
        x, y, yaw_deg = float(x), float(y), float(yaw_deg)
        focus.append((x, y))
        color = POSE_COLORS[index % len(POSE_COLORS)]
        axes.plot([x], [y], "o", ms=15, color=color, zorder=7,
                  label=f"{label}  ({x:.2f}, {y:.2f}, {yaw_deg:+.1f}°)")
        axes.annotate("", xy=(x + 0.9 * math.cos(math.radians(yaw_deg)),
                              y + 0.9 * math.sin(math.radians(yaw_deg))),
                      xytext=(x, y), zorder=7,
                      arrowprops=dict(arrowstyle="-|>", color=color, lw=3.0,
                                      mutation_scale=22))

    if len(arguments.pose) >= 2:
        first = np.array([float(arguments.pose[0][1]), float(arguments.pose[0][2])])
        second = np.array([float(arguments.pose[1][1]), float(arguments.pose[1][2])])
        axes.annotate(f"2 つの答えの差 {np.linalg.norm(first - second)*1000:.0f} mm",
                      (first + second) / 2, textcoords="offset points", xytext=(0, -34),
                      ha="center", fontsize=11, weight="bold", color="black", zorder=9,
                      bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="0.4"))

    center = np.array(focus).mean(axis=0) if focus else np.array([0.0, 0.0])
    axes.set_xlim(center[0] - arguments.span, center[0] + arguments.span)
    axes.set_ylim(center[1] - arguments.span, center[1] + arguments.span)
    for step, alpha in ((0.5, 0.15), (1.0, 0.45)):
        for value in np.arange(math.floor(axes.get_xlim()[0]), axes.get_xlim()[1], step):
            axes.axvline(value, color="tab:blue", lw=0.4, alpha=alpha)
        for value in np.arange(math.floor(axes.get_ylim()[0]), axes.get_ylim()[1], step):
            axes.axhline(value, color="tab:blue", lw=0.4, alpha=alpha)
    axes.plot([], [], "*", ms=18, color="gold", mec="black", label="登録したタグ")
    axes.plot([], [], "o", ms=7, mfc="none", mec="0.45", label="登録に使った 3 地点")
    axes.legend(loc="upper left", fontsize=10, framealpha=0.94)
    axes.set_xlabel("map x [m]   （細い格子 0.5 m / 太い格子 1 m）")
    axes.set_ylabel("map y [m]")
    axes.set_title("いまの状態 — タグの登録位置と、ロボットの推定位置", fontsize=14)
    figure.tight_layout()
    figure.savefig(arguments.out, dpi=arguments.dpi)
    print(f"書いた: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
