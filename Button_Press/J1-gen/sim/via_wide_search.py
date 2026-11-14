"""広げた設定（reach_wide.yaml の指の向きの候補）のまま、経由の姿勢を広い範囲で探し直す（計画だけ。MuJoCo は動かさない）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/via_wide_search.py --map _local/button_press/reach_map/20261030_wide --workers 10

sim/via_search.py・via_refine.py は、指の向きの候補なし・肩 roll −50〜+10° で探した。ここでは
- 指の向きの候補 [[0,0],[15,0],[30,0],[45,0]]（reach_wide.yaml と同じ）を入れて、
- 肩 roll（腕を横へ開く、x 軸まわりに近い回転）を −90° まで、肩 pitch・yaw・肘も広げて、
- 頭カメラの画像に入るセル（--map の map.csv で visible = 1）だけで数える（画像に入らない所は動きを変えても押せないため）。
ランダムに --n 個 → 上位から山登り --rounds 回。1 つの経由の姿勢で通る数を最大にする。参考に、2 つの姿勢を使い分けたときの数も出す。
出力: _local/button_press/via_wide_search/<日時>/result.json（候補ごとの通る数、最良の姿勢、最良で新しく通る・通らなくなるセル）
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reach_map as rm  # noqa: E402

Cell = tuple[float, float, float, float, float]
TILTS = [(0.0, 0.0), (15.0, 0.0), (30.0, 0.0), (45.0, 0.0)]
WIDE_VIA = [68.3, -11.1, -40.9, -43.9, 0.0, -4.1, 0.0]
# 右腕 7 関節 [deg]: 肩 pitch・roll・yaw、肘、手首 roll・pitch・yaw。手首 roll・yaw は 0 のまま
SEARCH_DIMS = [0, 1, 2, 3, 5]
LIMIT = np.array([[-30, 100], [-90, 10], [-60, 60], [-55, 100], [0, 0], [-40, 40], [0, 0]], dtype=float)
REPO = Path(__file__).resolve().parents[3]


def load_cells(map_dir: Path) -> tuple[list[Cell], dict[Cell, dict[str, str]]]:
    """map.csv から、頭カメラの画像に入るセルと、そのときの結果（比べる元）を読む。"""
    rows = list(csv.DictReader(open(map_dir / "map.csv", newline="")))
    base = {}
    for r in rows:
        if r["visible"] == "1":
            c = (float(r["edge"]), float(r["gap"]), float(r["y"]), float(r["top"]), float(r.get("slope", 0) or 0))
            base[c] = r
    return list(base), base


def eval_via(job: tuple[int, list[float], list[Cell]]) -> tuple[int, list[int]]:
    i, via_deg, cells = job
    via = np.radians(np.asarray(via_deg, dtype=float))
    return i, [rm.run_cell(c, via=via, tilts=TILTS)["plan_ok"] for c in cells]


def evaluate(ex: ProcessPoolExecutor, cands: list[list[float]], cells: list[Cell]) -> list[list[int]]:
    out: list[list[int]] = [[] for _ in cands]
    for i, ok in ex.map(eval_via, [(i, c, cells) for i, c in enumerate(cands)], chunksize=1):
        out[i] = ok
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, required=True, help="reach_map.py の結果（広げた設定）。visible の列を使う")
    ap.add_argument("--n", type=int, default=120, help="ランダムに試す経由の姿勢の数")
    ap.add_argument("--rounds", type=int, default=6)
    ap.add_argument("--children", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    map_dir = args.map if args.map.is_absolute() else REPO / args.map
    out = args.out or REPO / "_local/button_press/via_wide_search" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)

    cells, base = load_cells(map_dir)
    rng = np.random.default_rng(args.seed)
    t0 = time.time()
    score: dict[tuple[float, ...], list[int]] = {}

    def log(msg: str) -> None:
        print(f"[via_wide] {msg}（{time.time() - t0:.0f} 秒）", flush=True)

    with ProcessPoolExecutor(args.workers, initializer=rm.init_worker, initargs=("right", "real", None, None)) as ex:
        rand = [WIDE_VIA] + [[round(float(v), 1) for v in LIMIT[:, 0] + rng.random(7) * (LIMIT[:, 1] - LIMIT[:, 0])]
                             for _ in range(args.n)]
        log(f"画像に入るセル {len(cells)}、ランダムの候補 {len(rand)}（今の広げた設定を含む）")
        for c, ok in zip(rand, evaluate(ex, rand, cells)):
            score[tuple(c)] = ok
        cur = sum(score[tuple(WIDE_VIA)])
        best = max(score, key=lambda k: sum(score[k]))
        log(f"ランダム: 今の広げた設定 {cur}、最良 {sum(score[best])} {list(best)}")
        for r in range(args.rounds):
            sigma = [10.0, 6.0, 4.0, 2.5, 1.5, 1.0][min(r, 5)]
            parents = sorted(score, key=lambda k: -sum(score[k]))[:4]
            kids = []
            for _ in range(args.children):
                p = np.array(parents[int(rng.integers(len(parents)))], dtype=float)
                d = np.zeros(7)
                d[SEARCH_DIMS] = rng.normal(0.0, sigma, len(SEARCH_DIMS))
                kids.append([round(float(v), 1) for v in np.clip(p + d, LIMIT[:, 0], LIMIT[:, 1])])
            kids = [k for k in kids if tuple(k) not in score]
            for k, ok in zip(kids, evaluate(ex, kids, cells)):
                score[tuple(k)] = ok
            best = max(score, key=lambda k: sum(score[k]))
            log(f"山登り 第 {r + 1} 回（σ {sigma}°）: 最良 {sum(score[best])} {list(best)}")

    keys = list(score)
    M = np.array([score[k] for k in keys], dtype=bool)
    b = keys.index(best)
    w = keys.index(tuple(WIDE_VIA))
    gained = [cells[j] for j in np.flatnonzero(M[b] & ~M[w])]
    lost = [cells[j] for j in np.flatnonzero(~M[b] & M[w])]
    gained_from = Counter(f"{base[c]['stage']}/{base[c]['partner']}" for c in gained)
    # 参考: 今の広げた設定に、もう 1 つの姿勢を足して使い分けたとき（先に通った方を使う）
    add = (M & ~M[w]).sum(axis=1)
    j2 = int(np.argmax(add))
    top = sorted(range(len(keys)), key=lambda i: -M[i].sum())[:10]
    res = {
        "cells_visible": len(cells), "tilts": TILTS, "limit_deg": LIMIT.tolist(), "n_candidates": len(keys),
        "wide_via": {"via_deg": WIDE_VIA, "ok": int(M[w].sum())},
        "best": {"via_deg": list(best), "ok": int(M[b].sum()), "gained": len(gained), "lost": len(lost),
                 "gained_by_old_reason": dict(gained_from)},
        "top10": [{"via_deg": list(keys[i]), "ok": int(M[i].sum())} for i in top],
        "pair_with_wide": {"via_deg": list(keys[j2]), "ok_together": int((M[w] | M[j2]).sum()), "added": int(add[j2])},
        "any_candidate_ok": int(M.any(axis=0).sum()),
        "gained_cells": [list(c) for c in gained], "lost_cells": [list(c) for c in lost],
    }
    (out / "result.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    log(f"今の広げた設定 {res['wide_via']['ok']}/{len(cells)} → 最良 {res['best']['ok']}（新しく通る {len(gained)}、"
        f"通らなくなる {len(lost)}）。使い分け（広げた設定 + 1 つ）{res['pair_with_wide']['ok_together']}。"
        f"どれかの候補で通る {res['any_candidate_ok']}")
    log(f"新しく通ったセルの元の理由: {dict(gained_from)}")
    print(f"[via_wide] 保存: {out}/result.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
