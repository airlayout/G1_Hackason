"""経由の姿勢（pipeline.yaml の via_pose）の候補を探して、ボトルを押せる範囲がどれだけ広がるかを調べる（計画だけ）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/via_search.py --n 200 --workers 8

右腕の 7 関節 [deg] を範囲の中でランダムに選び、sim/reach_map.py と同じ格子の各セルで、その経由の姿勢を使った計画が通るかを数える。
現在の via_pose の通る数・1 つの姿勢で最も多く通るもの・数個の姿勢を使い分けるとき（先に見つかった姿勢で計画する）の通る数を出す。
実機の開始の腕の姿勢が分かっていないと、最初の動き（開始→経由）の判定がずれる（reach_map.py の説明を参照）。
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

# 右腕の 7 関節 [deg]: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
LO = np.array([-10.0, -45.0, -30.0, -70.0, 0.0, 0.0, 0.0])
HI = np.array([60.0, 5.0, 30.0, 70.0, 0.0, 0.0, 0.0])


def eval_via(job: tuple[int, list[float], list[tuple[float, float, float, float, float]]]) -> tuple[int, list[int]]:
    i, via_deg, cells = job
    via = np.radians(np.asarray(via_deg, dtype=float))
    return i, [rm.run_cell(c, via=via)["plan_ok"] for c in cells]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=100, help="試す経由の姿勢の数（現在の via_pose に足す）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--edges", type=float, nargs="+", default=[round(0.16 + 0.02 * i, 2) for i in range(13)])
    ap.add_argument("--gaps", type=float, nargs="+", default=[0.0, 0.06])
    ap.add_argument("--ys", type=float, nargs="+", default=[round(-0.30 + 0.05 * i, 2) for i in range(7)])
    ap.add_argument("--tops", type=float, nargs="+", default=[0.0])
    ap.add_argument("--slopes", type=float, nargs="+", default=[0.0, -0.033])
    ap.add_argument("--clearance", type=float)
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    cells = [(e, g, y, t, sl) for sl in args.slopes for t in args.tops for e in args.edges for g in args.gaps for y in args.ys]
    rng = np.random.default_rng(args.seed)
    cur = np.degrees(np.asarray(load_via(), dtype=float)).tolist()
    cands = [cur] + [(LO + rng.random(7) * (HI - LO)).tolist() for _ in range(args.n)]
    t0 = time.time()
    print(f"[via] セル {len(cells)} × 候補 {len(cands)}（現在の via_pose を含む）")
    res: dict[int, list[int]] = {}
    with ProcessPoolExecutor(args.workers, initializer=rm.init_worker, initargs=("right", "real", None, args.clearance)) as ex:
        for i, ok in ex.map(eval_via, [(i, c, cells) for i, c in enumerate(cands)], chunksize=1):
            res[i] = ok
    M = np.array([res[i] for i in range(len(cands))], dtype=bool)   # 候補 × セル
    print(f"[via] 完了（{time.time() - t0:.0f} 秒）")
    base = M[0]
    print(f"現在の via_pose {np.round(cands[0], 1).tolist()}: 通る {base.sum()}/{len(cells)}")
    order = np.argsort(-M.sum(axis=1))
    print("1 つの姿勢で多く通るもの（上位 5）:")
    for i in order[:5]:
        print(f"  {M[i].sum():4d} 通り  {np.round(cands[i], 1).tolist()}" + ("（現在）" if i == 0 else ""))
    cover = np.zeros(len(cells), dtype=bool)
    chosen = []
    for _ in range(4):
        gain = (M & ~cover).sum(axis=1)
        j = int(np.argmax(gain))
        if gain[j] == 0:
            break
        cover |= M[j]
        chosen.append(j)
        print(f"使い分け {len(chosen)} 個目: {np.round(cands[j], 1).tolist()}  → 通る合計 {cover.sum()}/{len(cells)}（+{gain[j]}）")
    print(f"全候補のどれかで通る: {M.any(axis=0).sum()}/{len(cells)}（上限の目安）")
    # 通る縁の最小（slope 0、gap 0.0、y ごと）
    for label, mask in (("現在", base), ("使い分け", cover)):
        mins = []
        for y in args.ys:
            xs = [c[0] for c, ok in zip(cells, mask) if ok and c[2] == y and c[4] == 0.0 and c[1] == args.gaps[0]]
            mins.append(f"{y:+.2f}:{min(xs) * 100:.0f}" if xs else f"{y:+.2f}:-")
        print(f"通る机の縁 x の最小 [cm]（{label}、傾き 0、gap {args.gaps[0] * 100:.0f} cm、y: 最小の縁）: " + "  ".join(mins))
    return 0


def load_via() -> np.ndarray:
    from common.config import load_config
    return np.radians(np.asarray(load_config("pipeline.yaml")["via_pose"], dtype=float))


if __name__ == "__main__":
    sys.exit(main())
