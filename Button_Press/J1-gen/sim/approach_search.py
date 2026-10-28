"""ボトルが体の正面寄り（右 10 cm より内側）でも押せるように、指の向きと IK の初期値の候補を探す（計画だけ）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/approach_search.py --workers 8 --via 68.3 -11.1 -40.9 -43.9 0 -4.1 0

今は指を押す向き（前方）に合わせるので、ボトルが体の正面寄りだと、手前の姿勢で腕・手首が胴体に近づきすぎる
（torso_link ↔ wrist が 1 cm 未満）。指を左・上へ傾ける（エレベーターの finger_tilts と同じ。手先は押す向きにまっすぐ動く）、
IK の初期値を増やす、のどちらが効くかを、体の正面寄りのセルで数える。
"""

from __future__ import annotations

import argparse
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reach_map as rm  # noqa: E402

Cell = tuple[float, float, float, float, float]


def job(args: tuple[int, tuple[float, float], list[Cell]]) -> tuple[int, list[int]]:
    i, tilt, cells = args
    return i, [rm.run_cell(c, tilts=[tilt])["plan_ok"] for c in cells]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--via", type=float, nargs=7)
    ap.add_argument("--lefts", type=float, nargs="+", default=[0, 15, 30, 45, 60, 75])
    ap.add_argument("--ups", type=float, nargs="+", default=[0, 15, 30, -15])
    ap.add_argument("--edges", type=float, nargs="+", default=[round(0.20 + 0.04 * i, 2) for i in range(6)])
    ap.add_argument("--gaps", type=float, nargs="+", default=[0.0, 0.06])
    ap.add_argument("--ys", type=float, nargs="+", default=[-0.20, -0.15, -0.10, -0.05, 0.0, 0.05])
    ap.add_argument("--tops", type=float, nargs="+", default=[0.0])
    ap.add_argument("--slopes", type=float, nargs="+", default=[0.0])
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    cells = [(e, g, y, t, sl) for sl in args.slopes for t in args.tops for e in args.edges for g in args.gaps for y in args.ys]
    tilts = [(l, u) for l in args.lefts for u in args.ups]
    t0 = time.time()
    print(f"[approach] セル {len(cells)} × 指の向き {len(tilts)} 通り")
    with ProcessPoolExecutor(args.workers, initializer=rm.init_worker,
                             initargs=("right", "real", None, None, args.via)) as ex:
        res = dict(ex.map(job, [(i, t, cells) for i, t in enumerate(tilts)], chunksize=1))
    M = np.array([res[i] for i in range(len(tilts))], dtype=bool)
    print(f"[approach] 完了（{time.time() - t0:.0f} 秒）")
    base = M[tilts.index((0, 0))] if (0, 0) in tilts else None
    if base is not None:
        print(f"今（指を前方に合わせる）: 通る {base.sum()}/{len(cells)}")
    order = np.argsort(-M.sum(axis=1))
    print("1 つの向きで多く通るもの（上位 6）:")
    for i in order[:6]:
        print(f"  {M[i].sum():3d} 通り  左 {tilts[i][0]:g}° 上 {tilts[i][1]:g}°")
    cover = np.zeros(len(cells), dtype=bool)
    for k in range(4):
        gain = (M & ~cover).sum(axis=1)
        j = int(np.argmax(gain))
        if gain[j] == 0:
            break
        cover |= M[j]
        print(f"使い分け {k + 1} 個目: 左 {tilts[j][0]:g}° 上 {tilts[j][1]:g}° → 通る合計 {cover.sum()}/{len(cells)}（+{gain[j]}）")
    print("左右 y ごとの、通る縁の数（セルのうち通った数 / 全部）: 今 → 向きを使い分けたとき")
    for y in args.ys:
        idx = [k for k, c in enumerate(cells) if c[2] == y]
        b = int(base[idx].sum()) if base is not None else 0
        print(f"  y {y:+.2f}: {b:2d} → {int(cover[idx].sum()):2d} / {len(idx)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
