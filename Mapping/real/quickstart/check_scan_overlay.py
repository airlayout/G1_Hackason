#!/usr/bin/env python3
"""1 枚のスキャンを、与えられた姿勢で地図に重ねて「合っているか」を測る。

## なぜ要るか

測位の自己申告（AMCL の共分散、MOLA の `pose_quality`）は**外していても高く出る**。
2026-09-13 には誤った姿勢の `pose_quality` が正しい姿勢より高かった。
共分散も、静止中に再標本化を繰り返せば粒子が潰れて 1e-06 まで落ちる。
**幾何で裏を取る以外にない。**

## ずらし検査

重畳率の絶対値だけでは「ずれている」のか「そもそも地図と中身が違う」のかが分からない。
姿勢を ±N セル ずらして採点し、**最良が (0,0) かどうか**を見る。

- 最良が (0,0) で、ずらすと下がる → **合っている**
- 最良が別の位置 → その方向にずれている（量も出る）
- どこも似た値 → 地図と中身が合っていない。重畳では判定できない

## 使い方

    ssh g1w 'bash /tmp/grab_scan.sh'      # /scan と /amcl_pose を 1 個ずつ落とす
    scp g1w:/tmp/scan.yaml g1w:/tmp/pose.yaml .
    Navigation/.venv/bin/python check_scan_overlay.py nav_map_run.yaml scan.yaml \\
        --pose-yaml pose.yaml --out /tmp/overlay.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

OCCUPIED_BELOW = 100          # PGM で黒いほど占有
DEFAULT_TOLERANCE_CELLS = 1   # 壁の帯。0 だと床の反射を「外れ」と数える
DEFAULT_SHIFT_CELLS = 5


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


def first_document(path: Path) -> dict:
    """`ros2 topic echo` は `--once` でも末尾に `---` を付ける。最初の 1 個を取る。"""

    for document in yaml.safe_load_all(path.read_text()):
        if isinstance(document, dict):
            return document
    raise SystemExit(f"YAML が読めない: {path}")


def load_scan(path: Path) -> tuple:
    """`ros2 topic echo /scan` の YAML から (角度, 距離) を取る。"""

    message = first_document(path)
    ranges = np.array([float("inf") if value is None else float(value)
                       for value in message["ranges"]], dtype=float)
    angles = (float(message["angle_min"])
              + np.arange(len(ranges)) * float(message["angle_increment"]))
    valid = np.isfinite(ranges) & (ranges > float(message["range_min"])) \
        & (ranges < float(message["range_max"]))
    return angles[valid], ranges[valid], message.get("header", {}).get("frame_id", "?")


def load_pose(path: Path) -> tuple:
    """`ros2 topic echo /amcl_pose` などの YAML から (x, y, yaw) を取る。"""

    message = first_document(path)
    pose = message.get("pose", message)
    if "pose" in pose:
        pose = pose["pose"]
    position = pose["position"]
    orientation = pose["orientation"]
    yaw = math.atan2(2.0 * (orientation["w"] * orientation["z"]
                            + orientation["x"] * orientation["y"]),
                     1.0 - 2.0 * (orientation["y"] ** 2 + orientation["z"] ** 2))
    return float(position["x"]), float(position["y"]), yaw


def score(grid_occupied: np.ndarray, columns: np.ndarray, rows: np.ndarray,
          tolerance: int) -> float:
    """占有セル（許容 ±tolerance セル）に乗った点の割合 [%]。"""

    height, width = grid_occupied.shape
    inside = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    if not np.any(inside):
        return 0.0
    columns, rows = columns[inside], rows[inside]
    hit = np.zeros(len(columns), dtype=bool)
    for offset_x in range(-tolerance, tolerance + 1):
        for offset_y in range(-tolerance, tolerance + 1):
            cx = np.clip(columns + offset_x, 0, width - 1)
            ry = np.clip(rows + offset_y, 0, height - 1)
            hit |= grid_occupied[ry, cx]
    return 100.0 * hit.mean()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("scan_yaml")
    parser.add_argument("--pose-yaml", default=None)
    parser.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--tolerance", type=int, default=DEFAULT_TOLERANCE_CELLS)
    parser.add_argument("--shift", type=int, default=DEFAULT_SHIFT_CELLS)
    parser.add_argument("--out", default=None, help="重ねた図を PNG で出す")
    arguments = parser.parse_args()

    map_path = Path(arguments.map_yaml)
    meta = yaml.safe_load(map_path.read_text())
    grid = read_pgm(map_path.parent / meta["image"])
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = grid.shape
    occupied = grid < OCCUPIED_BELOW

    if arguments.pose_yaml:
        x, y, yaw = load_pose(Path(arguments.pose_yaml))
    elif arguments.pose:
        x, y, yaw = arguments.pose[0], arguments.pose[1], math.radians(arguments.pose[2])
    else:
        parser.error("--pose-yaml か --pose のどちらかが要る")

    angles, ranges, frame = load_scan(Path(arguments.scan_yaml))
    print(f"姿勢 (x {x:+.3f}, y {y:+.3f}, yaw {math.degrees(yaw):+.2f} deg) / "
          f"スキャン {len(ranges)} 点（frame {frame}）")
    if frame not in ("base_link", "base_footprint"):
        print(f"⚠️ スキャンの frame が {frame}。base_link 前提で計算している")

    local_x = ranges * np.cos(angles)
    local_y = ranges * np.sin(angles)
    map_x = x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
    map_y = y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
    columns = np.round((map_x - origin_x) / resolution).astype(int)
    # PGM の行 0 は y が大きい側
    rows = np.round((height - 1) - (map_y - origin_y) / resolution).astype(int)

    best = score(occupied, columns, rows, arguments.tolerance)
    band_cm = arguments.tolerance * resolution * 100.0
    print(f"\n重畳（許容 ±{arguments.tolerance} セル = {band_cm:.0f} cm）")
    print(f"  そのままの姿勢: **{best:.1f} %**")

    print(f"\n=== ずらし検査（±{arguments.shift} セル = ±{arguments.shift*resolution:.1f} m）===")
    table = np.zeros((2 * arguments.shift + 1, 2 * arguments.shift + 1))
    for index_y, shift_y in enumerate(range(-arguments.shift, arguments.shift + 1)):
        for index_x, shift_x in enumerate(range(-arguments.shift, arguments.shift + 1)):
            table[index_y, index_x] = score(occupied, columns + shift_x,
                                            rows + shift_y, arguments.tolerance)
    center = arguments.shift
    peak = np.unravel_index(int(np.argmax(table)), table.shape)
    peak_shift_x = peak[1] - center
    peak_shift_y = peak[0] - center
    print(f"  最良は ({peak_shift_x:+d}, {peak_shift_y:+d}) セル "
          f"= ({peak_shift_x*resolution:+.2f}, {-peak_shift_y*resolution:+.2f}) m "
          f"で {table[peak]:.1f} %")
    spread = table.max() - table.min()
    if peak_shift_x == 0 and peak_shift_y == 0:
        print("  ✅ 最良が中心。この姿勢で合っている")
    else:
        print(f"  ⛔ 中心から {math.hypot(peak_shift_x, peak_shift_y)*resolution:.2f} m ずれている")
    if spread < 5.0:
        print(f"  ⚠️ 全体の幅が {spread:.1f} pt しかない。"
              "重畳ではこの場所の姿勢を決められない（地図が平坦）")

    if arguments.out:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from jp_font import japanese_font
        japanese_font()
        figure, axes = plt.subplots(figsize=(11, 9))
        extent = (origin_x, origin_x + width * resolution,
                  origin_y, origin_y + height * resolution)
        axes.imshow(np.flipud(grid), cmap="gray", origin="lower", extent=extent,
                    vmin=0, vmax=255, interpolation="nearest")
        axes.plot(map_x, map_y, ".", ms=1.6, color="tab:red", label="スキャン")
        axes.plot([x], [y], "o", ms=8, color="tab:blue", label="ロボット")
        axes.arrow(x, y, 1.0 * math.cos(yaw), 1.0 * math.sin(yaw), head_width=0.3,
                   fc="tab:blue", ec="tab:blue")
        pad = 12.0
        axes.set_xlim(x - pad, x + pad)
        axes.set_ylim(y - pad, y + pad)
        axes.set_title(f"重畳 {best:.1f} %  （許容 ±{arguments.tolerance} セル）")
        axes.legend(loc="upper right", fontsize=9)
        figure.tight_layout()
        figure.savefig(arguments.out, dpi=120)
        print(f"\n書いた: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
