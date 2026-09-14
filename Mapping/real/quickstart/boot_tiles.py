#!/usr/bin/env python3
"""**再定位の探索を部屋ごとに切る**ためのタイル中心を出す。

## なぜ要るのか（2026-09-14 実測）

`mola_relocalization` の SE(2) 尤度は **「当たった点の数」で正規化されていない**。
地図の外では対応がつく点が少ない（一致率 48%）が、その少数がよく合えば平均は高く出る。
結果、**ROI を広げると探索が ROI の縁の「地図の外」を 1 位にする**:

| ROI | 09-13 採用姿勢との差 | 一致率 | 判定 |
|---|---|---|---|
| ±2〜4 m | 0.560 m | 100% | 採用 |
| **±5 m** | **6.99 m / 180 deg** | **48%** | 自信なし |

⇒ **1 回の探索は地図の中に収める。**広く探したいときは**小さい ROI を敷き詰め**、
タイルをまたいだ比較は尤度ではなく**帯の残差と一致率**で行う（そちらは正規化されている）。

## 絞り方

全面を敷き詰めると 112 枚になるが、**占有セルが十分あるタイルだけ**に絞れば 36 枚で済む
（`--min-occ 200`。この地図は 69.9 x 42.4 m で、大半が空白）。

使い方:

    Navigation/.venv/bin/python quickstart/boot_tiles.py <nav_map.yaml> \\
        --roi 3.0 --step 5.0 --min-occ 200
    # 1 行 1 タイルで "x y 占有セル数" を出す（占有の多い順）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import read_map  # noqa: E402


def tiles(map_yaml: Path, roi: float, step: float, min_occ: int):
    occ, res, ox, oy = read_map(map_yaml)
    h, _ = occ.shape
    ys, xs = np.nonzero(occ)
    # ⚠️ read_map は PGM を反転せずに返す（overlay_at_pose.py と同じ規約）
    wx = ox + xs * res
    wy = oy + (h - 1 - ys) * res
    out = []
    for a in np.arange(wx.min() + roi, wx.max(), step):
        for b in np.arange(wy.min() + roi, wy.max(), step):
            n = int(((np.abs(wx - a) <= roi) & (np.abs(wy - b) <= roi)).sum())
            if n >= min_occ:
                out.append((float(a), float(b), n))
    out.sort(key=lambda t: -t[2])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--roi", type=float, default=3.0)
    ap.add_argument("--step", type=float, default=5.0)
    ap.add_argument("--min-occ", type=int, default=200,
                    help="タイルに要る占有セル数。少ないと空白を探して時間を捨てる")
    a = ap.parse_args()
    t = tiles(a.map_yaml, a.roi, a.step, a.min_occ)
    if not t:
        print("タイルが 1 枚も立たない（--min-occ を下げるか地図を確かめること）",
              file=sys.stderr)
        return 1
    for x, y, n in t:
        print("{:.3f} {:.3f} {}".format(x, y, n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
