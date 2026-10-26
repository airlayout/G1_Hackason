"""1 回の起動で台ごとに何回も評価した記録から、偏らずに成功率を数える（V2、10-16）。

count_success.py は各台の最初の 1 回だけを数える（それより後を混ぜると、早く失敗して終わる回ほど多く数えて成功率が低く出る）。
ここでは「締め切りの時刻より前に始まった回」を全部数える。回が始まった時刻はその回の結果と関係しないので偏らない。
締め切りは記録の終わりから --tail ステップ前（既定 3000＝1 回の上限 60 秒）にし、締め切り前に始まった回は必ず終わっている。
評価は ++algo.config.eval.reset_policy_on_done=True（パッチ 0053）で回す（回の初めに方策の記憶を消す。学習と同じ）。

使い方: python count_success_cutoff.py <記録のディレクトリ> [--tail 3000] [--json out.json]
"""
import argparse
import json
import math
import sys

import numpy as np

from count_success import LAST_STAGE, THROUGH_X_M, load


def episodes(rec: dict, tail: int):
    stage, ep_len = rec["stage"], rec["episode_length"]
    x = rec["root_pos"][..., 0]
    hinge = np.abs(rec["door_dof_pos"][..., 0])
    steps, envs = stage.shape
    cutoff = steps - tail
    out = []
    for e in range(envs):
        starts = [0] + [t for t in range(1, steps) if ep_len[t, e] < ep_len[t - 1, e]]
        for k, a in enumerate(starts):
            if a >= cutoff:
                break
            b = starts[k + 1] if k + 1 < len(starts) else steps
            st, xs = stage[a:b, e], x[a:b, e]
            success = bool(np.any((st == LAST_STAGE) & (xs > THROUGH_X_M)))
            out.append({"env": e, "k": k, "start": a, "len": b - a, "success": success,
                        "finished": k + 1 < len(starts) or success, "start_stage": int(st[0]),
                        "max_stage": int(st.max()), "max_hinge_deg": float(np.degrees(hinge[a:b, e].max()))})
    return out, steps, cutoff


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dir")
    ap.add_argument("--tail", type=int, default=3000)
    ap.add_argument("--json")
    a = ap.parse_args()
    eps, steps, cutoff = episodes(load(a.record_dir), a.tail)
    if cutoff <= 0:
        print(f"記録が {steps} ステップしかない（締め切りの余裕 {a.tail} より短い）")
        return 2
    unfinished = [e for e in eps if not e["finished"]]
    counted = [e for e in eps if e["finished"] and e["start_stage"] == 0]
    k = sum(e["success"] for e in counted)
    n = len(counted)
    lo, hi = wilson(k, n)
    fail = [e for e in counted if not e["success"]]
    kinds = {
        "扉 10° 未満": sum(e["max_hinge_deg"] < 10 for e in fail),
        "10〜70°": sum(10 <= e["max_hinge_deg"] < 70 for e in fail),
        "70° 以上（通り抜けず）": sum(e["max_hinge_deg"] >= 70 for e in fail),
    }
    print(f"記録 {steps} ステップ・締め切り {cutoff}（終わっていない回 {len(unfinished)}）")
    print(f"成功（扉の先 1.5 m まで通り抜けた）: {k}/{n} = {100.0 * k / max(n, 1):.1f}%（95% の幅 {100 * lo:.1f}〜{100 * hi:.1f}%）")
    print("失敗の内訳（扉の最大の角度）: " + "・".join(f"{name} {v}" for name, v in kinds.items()))
    if a.json:
        json.dump({"success": k, "episodes": n, "rate": k / max(n, 1), "ci95": [lo, hi], "steps": steps, "cutoff": cutoff,
                   "unfinished": len(unfinished), "fail_kinds": kinds, "per_episode": counted}, open(a.json, "w"), ensure_ascii=False, indent=1)
    return 0 if not unfinished else 1


if __name__ == "__main__":
    sys.exit(main())
