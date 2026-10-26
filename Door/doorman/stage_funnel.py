"""評価の記録（run_eval_record.sh の states_*.npz）から、回ごとに「どの区間で止まり、どう終わったか」を数える（CPU だけ・学習に触れない）。

count_success_cutoff.py と同じく「記録の終わりから --tail ステップより前に始まった回」を数える（偏らない）。回ごとに:
  max_stage   届いた区間（0 扉へ歩く・1 構える・2 レバーに手・3 レバーを下げる・4 押して開ける・5 通り抜ける）
  終わり方     success（通り抜け）・fall（腰が FALL_Z より下）・stage_timeout（最後の区間に、その区間の持ち時間いっぱいいた）・other
  区間ごとの滞在 [ステップ]・レバーの最大の角度（door_dof_pos の 1 番）・蝶番の最大の角度（0 番）

使い方: python stage_funnel.py <記録> [<比べる記録> ...] [--tail 3000]
      台ごとの最初の 1 回だけの評価（eval_from_start.sh）: --tail 0 --first-only
"""
from __future__ import annotations

import argparse
import glob
import json
from collections import Counter
from pathlib import Path

import numpy as np

STAGE_NAMES = ["0 扉へ歩く", "1 構える", "2 レバーに手", "3 レバーを下げる", "4 押して開ける", "5 通り抜ける"]
STAGE_TIME = [500, 200, 200, 200, 600, 400]  # 評価の max_stage_time（区間の時間 2 倍）
FALL_Z = 0.45  # 腰の高さ [m]（立っていれば約 0.75）
THROUGH_X = 1.5


def load(d: str) -> dict:
    files = sorted(glob.glob(f"{d}/states_*.npz"))
    keys = ("stage", "episode_length", "root_pos", "door_dof_pos")
    parts = {k: [] for k in keys}
    for f in files:
        z = np.load(f)
        for k in keys:
            parts[k].append(z[k])
    return {k: np.concatenate(v) for k, v in parts.items()}


def episodes(rec: dict, tail: int, first_only: bool = False) -> list[dict]:
    stage, ep_len, pos, door = rec["stage"], rec["episode_length"], rec["root_pos"], rec["door_dof_pos"]
    steps, envs = stage.shape
    cutoff = steps - tail
    out = []
    for e in range(envs):
        starts = [0] + [t for t in range(1, steps) if ep_len[t, e] < ep_len[t - 1, e]]
        for k, a in enumerate(starts):
            if a >= cutoff or (first_only and k > 0):
                break
            b = starts[k + 1] if k + 1 < len(starts) else steps
            st = stage[a:b, e].astype(int)
            ms = int(st.max())
            success = bool(np.any((st == 5) & (pos[a:b, e, 0] > THROUGH_X)))
            dwell = [int((st == s).sum()) for s in range(6)]
            z_end = float(pos[b - 1, e, 2])
            if success:
                how = "success"
            elif z_end < FALL_Z or float(pos[a:b, e, 2].min()) < FALL_Z:
                how = "fall"
            elif dwell[ms] >= STAGE_TIME[ms] - 2:
                how = "stage_timeout"
            else:
                how = "other"
            out.append({"max_stage": ms, "how": how, "dwell": dwell,
                        "lever_deg": float(np.degrees(np.abs(door[a:b, e, 1]).max())),
                        "hinge_deg": float(np.degrees(np.abs(door[a:b, e, 0]).max()))})
    return out


def report(name: str, eps: list[dict]) -> None:
    n = len(eps)
    print(f"\n■ {name}（{n} 回）")
    print("| 止まった区間 | 回数 | 転んだ | 区間の時間切れ | その他 | 成功 | その区間にいた時間の中央 [秒] | レバーの最大角の中央 [°] |")
    print("|---|---|---|---|---|---|---|---|")
    for s in range(6):
        g = [x for x in eps if x["max_stage"] == s]
        if not g:
            continue
        c = Counter(x["how"] for x in g)
        dw = np.median([x["dwell"][s] for x in g]) / 50
        lv = np.median([x["lever_deg"] for x in g])
        print(f"| {STAGE_NAMES[s]} | {len(g)}（{100 * len(g) / n:.0f}%） | {c['fall']} | {c['stage_timeout']} | {c['other']} | {c['success']} | {dw:.1f} | {lv:.1f} |")
    reach = [sum(1 for x in eps if x["max_stage"] >= s) for s in range(6)]
    conv = "・".join(f"{s - 1}→{s} {100 * reach[s] / max(1, reach[s - 1]):.0f}%" for s in range(1, 6))
    print(f"区間の通り抜け率（その区間に来た回のうち、次へ進んだ割合）: {conv}")


def pooled(eps: list[dict]) -> dict:
    """全部の記録を合わせた数（鎖の判定用）: 回数・成功率・区間ごとの届いた数・通り抜け率"""
    n = len(eps)
    reach = [sum(1 for x in eps if x["max_stage"] >= s) for s in range(6)]
    conv = {f"{s - 1}to{s}": (reach[s] / reach[s - 1] if reach[s - 1] else None) for s in range(1, 6)}
    succ = sum(1 for x in eps if x["how"] == "success")
    return {"episodes": n, "success": succ, "success_rate": succ / n if n else None, "reach": reach, "conv": conv}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+")
    ap.add_argument("--tail", type=int, default=3000)
    ap.add_argument("--first-only", action="store_true", help="台ごとの最初の 1 回だけ（eval_from_start.sh の記録。--tail 0 と一緒に）")
    ap.add_argument("--json", help="全部の記録を合わせた数を JSON に書く（chain_s6va_1021.sh の判定用）")
    a = ap.parse_args()
    allep = []
    for d in a.records:
        eps = episodes(load(d), a.tail, a.first_only)
        allep += eps
        report(str(Path(d).relative_to(Path(d).parents[1])), eps)
    if a.json:
        Path(a.json).write_text(json.dumps(pooled(allep), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
