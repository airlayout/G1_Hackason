#!/usr/bin/env python3
"""占有格子地図を、座標を読み取れる図にする。

## なぜ要るか

「ロボットはいまどこにいるか」を人に聞くとき、`nav_map_run.pgm` をそのまま見せても
座標が読めない。AMCL の `--init` も MOLA の `--init` も世界座標を要求するので、
**格子と目盛りを入れて、地図の上で指させる**ようにする。

余白（未観測セル）は切り落とす。`nav_map_run` は右側 40 m ほどが空で、
そのまま描くと部屋が潰れる。

## 使い方

    Navigation/.venv/bin/python render_map_grid.py nav_map_run.yaml --out /tmp/map.png
    ... --mark 0.703 12.966 -57.5 --label "前回の初期姿勢"
"""

from __future__ import annotations

import argparse
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

# 占有とみなす画素値（黒側）。切り出しはこれの分布で決める。
OCCUPIED_BELOW = 100
# ⚠️ 「未観測でないセル」で切ると効かない。nav_map_run には**遠くに散った観測セル**が
# あり、箱が 80 m まで伸びる（2026-09-16 に踏んだ）。分位で外れ値を落とす。
CROP_PERCENTILE = 0.7


def read_pgm(path: Path) -> np.ndarray:
    """P5 形式の PGM を読む（`#` のコメント行に対応）。"""

    with path.open("rb") as handle:
        magic = handle.readline().strip()
        if magic != b"P5":
            raise SystemExit(f"P5 の PGM ではない: {path} ({magic!r})")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        handle.readline()  # maxval
        data = np.frombuffer(handle.read(), dtype=np.uint8)
    return data[:width * height].reshape(height, width)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("--out", default="map_grid.png")
    parser.add_argument("--mark", type=float, nargs=3, action="append",
                        metavar=("X", "Y", "YAW_DEG"), help="印を打つ姿勢。複数可")
    parser.add_argument("--label", action="append", default=[], help="--mark ごとの名前")
    parser.add_argument("--margin", type=float, default=2.0, help="切り出しの余白 [m]")
    parser.add_argument("--dpi", type=int, default=120)
    arguments = parser.parse_args()

    japanese_font()
    yaml_path = Path(arguments.map_yaml)
    meta = yaml.safe_load(yaml_path.read_text())
    grid = read_pgm(yaml_path.parent / meta["image"])
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = grid.shape

    # 占有セルの分布で切り出す（散った外れ値を分位で落とす）
    rows, columns = np.nonzero(grid < OCCUPIED_BELOW)
    if len(rows) == 0:
        raise SystemExit("占有セルが無い")
    low, high = CROP_PERCENTILE, 100.0 - CROP_PERCENTILE
    column_low, column_high = np.percentile(columns, [low, high])
    row_low, row_high = np.percentile(rows, [low, high])
    left = origin_x + column_low * resolution - arguments.margin
    right = origin_x + (column_high + 1) * resolution + arguments.margin
    # PGM の行 0 は地図の上端＝ y が大きい側
    bottom = origin_y + (height - row_high - 1) * resolution - arguments.margin
    top = origin_y + (height - row_low) * resolution + arguments.margin

    extent = (origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution)
    span_x, span_y = right - left, top - bottom
    figure, axes = plt.subplots(figsize=(13.0, 13.0 * span_y / span_x))
    axes.imshow(np.flipud(grid), cmap="gray", origin="lower", extent=extent,
                vmin=0, vmax=255, interpolation="nearest")
    axes.set_xlim(left, right)
    axes.set_ylim(bottom, top)

    for step, color, alpha in ((1.0, "tab:blue", 0.18), (5.0, "tab:blue", 0.55)):
        for x in np.arange(np.ceil(left / step) * step, right, step):
            axes.axvline(x, color=color, lw=0.4, alpha=alpha)
        for y in np.arange(np.ceil(bottom / step) * step, top, step):
            axes.axhline(y, color=color, lw=0.4, alpha=alpha)
    axes.set_xticks(np.arange(np.ceil(left / 5.0) * 5.0, right, 5.0))
    axes.set_yticks(np.arange(np.ceil(bottom / 5.0) * 5.0, top, 5.0))
    axes.set_xticks(np.arange(np.ceil(left), right, 1.0), minor=True)
    axes.set_yticks(np.arange(np.ceil(bottom), top, 1.0), minor=True)
    axes.tick_params(labelsize=9)

    for index, mark in enumerate(arguments.mark or []):
        x, y, yaw = mark
        label = arguments.label[index] if index < len(arguments.label) else f"印 {index}"
        axes.plot([x], [y], marker="o", ms=9, mfc="none", mec="tab:red", mew=2.0)
        length = 1.2
        axes.arrow(x, y, length * np.cos(np.radians(yaw)), length * np.sin(np.radians(yaw)),
                   head_width=0.35, head_length=0.35, fc="tab:red", ec="tab:red", lw=1.5)
        axes.annotate(f"{label}\n({x:.2f}, {y:.2f}, {yaw:.1f}°)", (x, y),
                      textcoords="offset points", xytext=(12, 12), fontsize=9,
                      color="tab:red")

    axes.set_xlabel("map x [m]   （yaw 0° は +x 方向、90° は +y 方向）")
    axes.set_ylabel("map y [m]")
    axes.set_title(f"{yaml_path.stem} — 細い格子 1 m / 太い格子 5 m", fontsize=12)
    figure.tight_layout()
    figure.savefig(arguments.out, dpi=arguments.dpi)
    print(f"書いた: {arguments.out}")
    print(f"切り出した範囲: x {left:.1f}〜{right:.1f} m / y {bottom:.1f}〜{top:.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
