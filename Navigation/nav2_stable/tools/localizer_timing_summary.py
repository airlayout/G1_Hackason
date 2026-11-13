#!/usr/bin/env python3
"""`map_localizer.py --timing-csv` の記録を集計し、照合を何 Hz まで回せるかを出す。

    python3 localizer_timing_summary.py <timing.csv>

⚠️ ここで出す「上限」は**照合だけに CPU を使えた場合**の値。実運用では Nav2・内蔵SLAM と
CPU を取り合い、さらに TF の配信（20Hz）も同じスレッドで回るので、上限いっぱいには設定しない。
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np


def main() -> None:
    if len(sys.argv) != 2:
        print("使い方: localizer_timing_summary.py <timing.csv>", file=sys.stderr)
        sys.exit(2)
    rows = list(csv.DictReader(Path(sys.argv[1]).open(encoding="utf-8")))
    if not rows:
        print("[timing] 記録が0件。点群・TF・地図が揃っていたかログを確認すること")
        sys.exit(1)
    n_pts = np.array([float(r["n_pts"]) for r in rows])
    stamps = np.array([float(r["cloud_stamp"]) for r in rows])
    span = stamps.max() - stamps.min()
    print(f"[timing] 照合 {len(rows)} 回（bag 上の {span:.1f} 秒ぶん、実効 "
          f"{(len(rows) - 1) / span if span > 0 else float('nan'):.2f} Hz）")
    print(f"[timing] 点数: 中央値 {np.median(n_pts):.0f} / 最小 {n_pts.min():.0f} / 最大 {n_pts.max():.0f}")
    print(f"[timing] {'':10s} {'中央値':>8s} {'p95':>8s} {'最大':>8s}  [ms]")
    for key, label in (("prep_ms", "点群の変換"), ("match_ms", "照合"), ("total_ms", "合計")):
        v = np.array([float(r[key]) for r in rows])
        print(f"[timing] {label:10s} {np.median(v):8.1f} {np.percentile(v, 95):8.1f} {v.max():8.1f}")
    total = np.array([float(r["total_ms"]) for r in rows])
    p95 = float(np.percentile(total, 95))
    print(f"[timing] p95 から見た上限: {1000.0 / p95:.1f} Hz（照合だけに CPU を使えた場合）")


if __name__ == "__main__":
    main()
