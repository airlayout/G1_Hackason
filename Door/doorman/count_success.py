"""評価の記録（state_recorder の states_*.npz）から、最初の区間から始めた回の成功率を数える。

DoorMan の評価の goal_reached は、ドアの環境では数えていない（legged_robot_base の既定のまま常に 0）。
そこで記録した区間と機体の位置から、回ごとに数え直す。

成功の定義（_stage_5_to_complete_condition と同じ）: 区間 5（通り抜け）にいて、機体が扉の面から +x に 1.5 m を越えた。
回の切れ目は episode_length が減った所。数えるのは各 env の最初の 1 回だけ（評価を
++algo.config.eval.eval_num_envs_episodes=True で回すと、全 env の最初の 1 回が終わるまで走る）。
2 回目以降を混ぜると、すぐ失敗して終わる回ほど多く数えられて成功率が低く出る。

成功の定義は --criterion で選ぶ:
    through（既定）: 上の通り抜け（段 2-1）
    door30: レバーを 40° 以上下げたうえで扉を 30° 以上開けた（段 2-2。2026-10-08 にユーザーが PoC 向けに決めた。
            通り抜けは段 2-4 でまとめて扱う）。summary.json の success・success_rate はこの定義になる
    passed: 段を見ずに、扉を 70° 以上開けたうえで扉の先 1.5 m まで抜けた（2026-11-08 計画書 8 章。段の持ち時間で
            打ち切らない評価 ++env.config.reset_on_overtime=False と組にして、段の判定の厳しさと切り分けて通り抜ける力を測る）
    arm10: 腕だけの段（パッチ 0097・env.config.arm_stage）。区間 1（構える）から始めた回を数え、区間 4 に入った
           （扉が 10° 開いた）回を成功にする（2026-11-09 計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md の手順 3・4）
    armthrough: 腕だけの段から始めて通り抜けまで（パッチ 0101・引き継ぎの評価）。区間 1 から始めた回を数え、through と同じ
                「区間 5 にいて扉の先 1.5 m」を成功にする
    arm3: 腕だけの段で、区間 1 から始めた回を数え、扉が 3° 以上開いた回を成功にする（パッチ 0098・11-09 ユーザー「引き継ぐ角度を
          下げる」。ラッチが外れて扉が動いた所で f3t3 に引き継ぐ。env.config.arm_stage.complete_door_deg=3.0 と組）

使い方:
    python count_success.py <記録のディレクトリ> [--pass-rate 0.8] [--criterion through|door30] [--json out.json]
終わりの値: 合格の線を越えたら 0、越えなければ 1、記録が読めなければ 2。
"""

import argparse
import glob
import json
import sys

import numpy as np

NUM_STAGES = 6
LAST_STAGE = NUM_STAGES - 1
THROUGH_X_M = 1.5          # _stage_5_to_complete_condition の 1.5 m
DOOR_OPEN_RAD = 0.174533   # 区間 3→4 の「扉が 10° 開いた」
HANDLE_DOWN_DEG = 40.0     # 段 2-2: レバーを下げた（DoorMan のラッチは 45° 回しきると外れる）
DOOR30_DEG = 30.0          # 段 2-2 の合格の線（2026-10-08 ユーザー判断 (a)）: レバーを下げて扉を 30° 以上開けた
PASSED_DOOR_DEG = 70.0     # passed: 区間 4→5 の door_through_angle_deg と同じ
ARM_START_STAGE = 1        # arm10: 腕だけの段は区間 1（構える）から始まる（arm_stage.start_stage の既定）
ARM_COMPLETE_STAGE = 4     # arm10: 区間 4 に入った＝扉が 10° 開いた（arm_stage.complete_stage の既定）
ARM_DOOR_DEG = 3.0         # arm3: 扉が 3° 開いた（arm_stage.complete_door_deg と同じ値）


def start_stage_of(criterion: str) -> int:
    """数える回の始めの区間（腕だけの段は 1、ほかは 0）"""
    return ARM_START_STAGE if criterion in ("arm10", "arm3", "armthrough") else 0


def load(record_dir: str) -> dict:
    files = sorted(glob.glob(f"{record_dir}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {record_dir}")
    chunks = [np.load(f, allow_pickle=True) for f in files]
    keys = ("stage", "episode_length", "root_pos", "door_dof_pos")
    return {k: np.concatenate([c[k] for c in chunks]) for k in keys}


def split_episodes(rec: dict) -> list[dict]:
    stage, ep_len = rec["stage"], rec["episode_length"]
    x, hinge = rec["root_pos"][..., 0], np.abs(rec["door_dof_pos"][..., 0])
    handle = rec["door_dof_pos"][..., 1]  # 扉の関節は 蝶番・レバー（・ラッチ）の順
    steps, envs = stage.shape
    episodes = []
    for e in range(envs):
        starts = [0] + [t for t in range(1, steps) if ep_len[t, e] < ep_len[t - 1, e]]
        a, b = 0, (starts[1] if len(starts) > 1 else steps)
        st, xs = stage[a:b, e], x[a:b, e]
        success = bool(np.any((st == LAST_STAGE) & (xs > THROUGH_X_M)))
        # passed: 扉が一度でも 70° 以上開いた後に、扉の先 1.5 m より先にいた歩がある（段は見ない）
        opened = np.degrees(np.maximum.accumulate(hinge[a:b, e])) >= PASSED_DOOR_DEG
        passed = bool(np.any(opened & (xs > THROUGH_X_M)))
        episodes.append({
            "env": e,
            "start_stage": int(st[0]),
            "max_stage": int(st.max()),
            "max_hinge_deg": float(np.degrees(hinge[a:b, e].max())),
            "max_handle_deg": float(np.degrees(handle[a:b, e].max())),
            "success": success,
            "passed": passed,
            "finished": len(starts) > 1 or success,
        })
    return episodes


def is_success(ep: dict, criterion: str) -> bool:
    if criterion == "door30":
        return ep["max_handle_deg"] >= HANDLE_DOWN_DEG and ep["max_hinge_deg"] >= DOOR30_DEG
    if criterion == "passed":
        return ep["passed"]
    if criterion == "arm10":
        return ep["max_stage"] >= ARM_COMPLETE_STAGE
    if criterion == "arm3":
        return ep["max_hinge_deg"] >= ARM_DOOR_DEG
    if criterion == "armthrough":
        return ep["success"]
    return ep["success"]


def summarize(episodes: list[dict], criterion: str = "through") -> dict:
    n = len(episodes)
    unfinished = sum(not ep["finished"] for ep in episodes)
    # 鍵の名前は episodes_from_stage0 のまま（stage_chain.sh などが読む）。腕だけの段では区間 1 から始めた回の数
    from_start = [ep for ep in episodes if ep["start_stage"] == start_stage_of(criterion) and ep["finished"]]
    m = len(from_start)
    reach = [sum(ep["max_stage"] >= s for ep in from_start) for s in range(NUM_STAGES)]
    summary = {
        "episodes": n,
        "unfinished": unfinished,
        "episodes_from_stage0": m,
        "success": sum(is_success(ep, criterion) for ep in from_start),
        "success_rate": (sum(is_success(ep, criterion) for ep in from_start) / m) if m else 0.0,
        "success_through": sum(ep["success"] for ep in from_start),
        "passed_door70": sum(ep["passed"] for ep in from_start),
        "door_opened_30deg_with_handle_down": sum(is_success(ep, "door30") for ep in from_start),
        "reach_stage_ge": reach,
        "door_opened_10deg": sum(ep["max_hinge_deg"] > np.degrees(DOOR_OPEN_RAD) for ep in from_start),
        "handle_down": sum(ep["max_handle_deg"] >= HANDLE_DOWN_DEG for ep in from_start),
        "success_with_handle_down": sum(ep["success"] and ep["max_handle_deg"] >= HANDLE_DOWN_DEG for ep in from_start),
    }
    per_env = [{"env": int(ep["env"]), "success": bool(is_success(ep, criterion)), "max_stage": int(ep["max_stage"]),
                "max_hinge_deg": round(float(ep["max_hinge_deg"]), 1),
                "max_handle_deg": round(float(ep["max_handle_deg"]), 1)} for ep in from_start]
    # numpy の整数・真偽値は json に書けないので Python の型に直す
    out = {k: ([int(v) for v in val] if isinstance(val, list) else (float(val) if k == "success_rate" else int(val)))
           for k, val in summary.items()}
    out["criterion"] = criterion
    out["start_stage"] = start_stage_of(criterion)
    out["per_env"] = per_env  # 動画に成否の印を付けるのに使う（eval_from_start.sh の VIDEO=1）
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dir")
    ap.add_argument("--pass-rate", type=float, default=0.8)
    ap.add_argument("--json")
    ap.add_argument("--criterion", choices=["through", "door30", "passed", "arm10", "arm3", "armthrough"], default="through")
    args = ap.parse_args()
    try:
        summary = summarize(split_episodes(load(args.record_dir)), args.criterion)
    except (FileNotFoundError, KeyError, ValueError) as err:
        print(f"記録を読めない: {err}", file=sys.stderr)
        return 2
    m = summary["episodes_from_stage0"]
    print(f"env {summary['episodes']} 台の最初の 1 回（区間 {summary['start_stage']} から始まり終わった回 {m}"
          f"・終わっていない回 {summary['unfinished']}）")
    print("区間 s 以上に届いた回: " + "  ".join(f"{s}:{c}/{m}" for s, c in enumerate(summary["reach_stage_ge"])))
    print(f"扉が 10° 以上開いた回: {summary['door_opened_10deg']}/{m}")
    print(f"レバーを {HANDLE_DOWN_DEG:.0f}° 以上下げた回: {summary['handle_down']}/{m}"
          f"（そのうえで通り抜けた回 {summary['success_with_handle_down']}）")
    print(f"レバーを {HANDLE_DOWN_DEG:.0f}° 以上下げて扉を {DOOR30_DEG:.0f}° 以上開けた回: "
          f"{summary['door_opened_30deg_with_handle_down']}/{m}")
    print(f"扉の先 {THROUGH_X_M} m まで通り抜けた回: {summary['success_through']}/{m}")
    print(f"段を見ずに扉を {PASSED_DOOR_DEG:.0f}° 以上開けて扉の先 {THROUGH_X_M} m まで抜けた回: {summary['passed_door70']}/{m}")
    label = {"door30": f"レバーを {HANDLE_DOWN_DEG:.0f}° 以上下げて扉を {DOOR30_DEG:.0f}° 以上開けた",
             "passed": f"段を見ずに扉を {PASSED_DOOR_DEG:.0f}° 以上開けて扉の先 {THROUGH_X_M} m まで抜けた",
             "arm10": f"腕だけで区間 {ARM_COMPLETE_STAGE} に入った（扉が 10° 開いた）",
             "arm3": f"腕だけで扉を {ARM_DOOR_DEG:.0f}° 以上開けた",
             "armthrough": f"腕だけの段から始めて扉の先 {THROUGH_X_M} m まで通り抜けた",
             }.get(args.criterion, f"扉の先 {THROUGH_X_M} m まで通り抜けた")
    print(f"成功（{label}）: {summary['success']}/{m} = {summary['success_rate']:.1%}（合格の線 {args.pass_rate:.0%}）")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)
    if m == 0:
        return 2
    return 0 if summary["success_rate"] >= args.pass_rate else 1


if __name__ == "__main__":
    sys.exit(main())
