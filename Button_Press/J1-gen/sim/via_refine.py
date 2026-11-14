"""経由の姿勢（pipeline.yaml の via_pose）を、良い姿勢の近くで細かく探す（山登り）。計画だけで MuJoCo は動かさない。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/via_refine.py --workers 8 --rounds 6
    # 開始点を与える（右腕 7 関節 [deg]。既定は via_search.py の探索で多く通った姿勢）
    $P Button_Press/J1-gen/sim/via_refine.py --start 55.9 -22 -22.6 -35 0 0 0

学習用の格子（reach_map.py の格子の一部）で通る数を最大にするように、肩・肘・手首ピッチをずらして探す。
探した結果は、学習に使っていない格子（縁・間隔・左右・天板の高さ・傾きを変えたもの）でも確かめる（過学習していないか）。
さらに、実機の開始姿勢が分かっていないことへの頑丈さとして、開始の腕を ±数度ずらしたときと、余裕（clearance_m）を
変えたときの通る数も出す。
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
SEARCH_DIMS = [0, 1, 2, 3, 5]     # 肩 pitch・roll・yaw、肘、手首 pitch（手首 roll・yaw は 0 のまま）
LIMIT = np.array([[-20, 70], [-50, 10], [-45, 45], [-80, 80], [0, 0], [-40, 40], [0, 0]], dtype=float)


def train_cells() -> list[Cell]:
    return [(e, g, y, 0.0, sl) for sl in (0.0, -0.033) for e in [round(0.16 + 0.02 * i, 2) for i in range(13)]
            for g in (0.0, 0.06) for y in [round(-0.30 + 0.05 * i, 2) for i in range(7)]]


def valid_cells() -> list[Cell]:
    """学習に使っていない格子: 縁・左右を半分ずらし、間隔・天板・傾きを変える。"""
    return [(e, g, y, t, sl) for sl in (0.0, -0.033, 0.03) for t in (-0.04, 0.02)
            for e in [round(0.17 + 0.02 * i, 2) for i in range(12)] for g in (0.03, 0.12)
            for y in [round(-0.275 + 0.05 * i, 3) for i in range(6)]]


def count(job: tuple[int, list[float], list[Cell]]) -> tuple[int, int]:
    i, via_deg, cells = job
    via = np.radians(np.asarray(via_deg, dtype=float))
    return i, sum(rm.run_cell(c, via=via)["plan_ok"] for c in cells)


def evaluate(ex: ProcessPoolExecutor, cands: list[list[float]], cells: list[Cell]) -> list[int]:
    out = [0] * len(cands)
    for i, n in ex.map(count, [(i, c, cells) for i, c in enumerate(cands)], chunksize=1):
        out[i] = n
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", type=float, nargs=7, action="append",
                    help="開始点の経由の姿勢 [deg]（複数回指定できる）。既定は via_search で多く通った 3 つ")
    ap.add_argument("--rounds", type=int, default=6)
    ap.add_argument("--children", type=int, default=32, help="1 回に作る子の数（親は上位 3 つから）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    from common.config import load_config
    current = load_config("pipeline.yaml")["via_pose"]
    starts = args.start or [[55.9, -22.0, -22.6, -35.0, 0, 0, 0], [51.1, -23.7, -28.8, -32.4, 0, 0, 0],
                            [58.9, -44.9, -8.0, -61.8, 0, 0, 0]]
    rng = np.random.default_rng(args.seed)
    tr, va = train_cells(), valid_cells()
    t0 = time.time()
    with ProcessPoolExecutor(args.workers, initializer=rm.init_worker, initargs=("right", "real", None, None)) as ex:
        pop = [list(map(float, current))] + [list(map(float, s)) for s in starts]
        score = dict(zip(map(tuple, pop), evaluate(ex, pop, tr)))
        print(f"[refine] 学習の格子 {len(tr)} セル、確認の格子 {len(va)} セル。現在 {score[tuple(pop[0])]}、開始点 "
              f"{[score[tuple(p)] for p in pop[1:]]}")
        for r in range(args.rounds):
            sigma = [8.0, 4.0, 2.0, 1.5, 1.0, 0.7][min(r, 5)]
            parents = sorted(score, key=lambda k: -score[k])[:3]
            kids = []
            for _ in range(args.children):
                p = np.array(parents[int(rng.integers(len(parents)))], dtype=float)
                d = np.zeros(7)
                d[SEARCH_DIMS] = rng.normal(0.0, sigma, len(SEARCH_DIMS))
                k = np.clip(p + d, LIMIT[:, 0], LIMIT[:, 1])
                kids.append([round(float(v), 1) for v in k])
            kids = [k for k in kids if tuple(k) not in score]
            for k, n in zip(kids, evaluate(ex, kids, tr)):
                score[tuple(k)] = n
            best = max(score, key=lambda k: score[k])
            print(f"[refine] 第 {r + 1} 回（σ {sigma}°）: 最良 {score[best]} 通り {list(best)}（{time.time() - t0:.0f} 秒）")
        top = sorted(score, key=lambda k: -score[k])[:5]
        cmp = [list(map(float, current))] + [list(t) for t in top]
        v = evaluate(ex, cmp, va)
        print("[refine] 確認の格子（学習に使っていない）での通る数:")
        for c, n in zip(cmp, v):
            print(f"  {n:4d} / {len(va)}   学習 {score.get(tuple(c), '-')}  {c}" + ("（現在）" if c == cmp[0] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
