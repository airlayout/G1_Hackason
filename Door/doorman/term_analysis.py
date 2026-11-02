"""評価の記録（パッチ 0089 の後）から、台ごとの 1 回目の終わり方を読み出す。
終わった理由のビット（term_reason）・終わった歩の段階・上半身の関節の速さ・終わる前の扉と力を、per_env.json に書き、
失敗の型ごとの数を出す。

使い方（OMEN で）: python term_analysis.py <記録> [<記録> ...]
"""
from __future__ import annotations

import glob
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

KEYS = (
    "stage", "episode_length", "door_dof_pos", "relative_door_pos", "hand_lever_force", "door_panel_contact_force",
    "term_reason", "term_stage", "term_upper_vel", "term_upper_joint", "upper_vel_max", "upper_vel_joint",
)
STAGE_NAMES = ("近づく", "構える", "つかむ", "レバー", "押し開け", "通り抜け")
BITS = ((32, "扉が 90°"), (16, "関節の速さ"), (8, "扉から 4 m"), (4, "倒れ"), (1, "持ち時間切れ"), (2, "完了か回の上限"))
DT = 0.02


def load(rec: Path) -> tuple[dict, list[str], dict]:
    files = sorted(glob.glob(f"{rec}/states_*.npz"))
    chunks = [np.load(f) for f in files]
    st = {k: np.concatenate([c[k] for c in chunks]) for k in KEYS if k in chunks[0].files}
    names = [str(n) for n in np.load(rec / "static.npz")["dof_names"]]
    summ = {e["env"]: e for e in json.load(open(rec / "summary.json"))["per_env"]}
    return st, names, summ


def kind_of(bits: int, success: bool) -> str:
    if success:
        return "成功"
    if bits < 0:
        return "理由なし"
    return next((name for bit, name in BITS if bits & bit), "理由なし")


def per_env(st: dict, names: list[str], summ: dict) -> list[dict]:
    out = []
    n_env = st["episode_length"].shape[1]
    for e in range(n_env):
        ep = st["episode_length"][:, e]
        drops = np.where(ep[1:] < ep[:-1])[0]
        if len(drops) == 0:
            continue
        b = int(drops[0]) + 1  # 戻された後の行（理由はここに持ち越されている）
        last = b - 1  # 終わる前の最後の行
        bits = int(st["term_reason"][b, e]) if "term_reason" in st else -1
        ok = bool(summ[e]["success"])
        door = np.degrees(np.abs(st["door_dof_pos"][: b, e, 0]))
        vel = st["upper_vel_max"][: b, e] if "upper_vel_max" in st else None
        row = {
            "env": e,
            "success": ok,
            "kind": kind_of(bits, ok),
            "bits": bits,
            "end_stage": int(st["term_stage"][b, e]) if "term_stage" in st else int(st["stage"][last, e]),
            "seconds": round((last + 1) * DT, 2),
            "door_max_deg": round(float(door.max()), 1),
            "door_end_deg": round(float(door[-1]), 1),
            "x_end": round(float(st["relative_door_pos"][last, e, 0]), 2),
            "lever_force_end": round(float(np.linalg.norm(st["hand_lever_force"][last, e])), 1),
            "panel_force_end": round(float(np.linalg.norm(st["door_panel_contact_force"][last, e])), 1),
        }
        if "term_upper_vel" in st:
            row["term_vel"] = round(float(st["term_upper_vel"][b, e]), 1)
            j = int(st["term_upper_joint"][b, e])
            row["term_joint"] = names[j] if j >= 0 else ""
        if vel is not None:
            # 終わる前の歩の速さ（記録の行は制御の 1 歩の終わりの値。打ち切りの判定と同じ時点）
            row["vel_prev_steps"] = [round(float(v), 1) for v in vel[max(0, last - 4): last + 1]]
            row["vel_max_before"] = round(float(vel[21:last + 1].max()) if last > 21 else 0.0, 1)
        out.append(row)
    return out


def report(rec: Path, rows: list[dict]) -> None:
    n = len(rows)
    if n == 0:
        print(f"== {rec.name}: 1 回目が終わった台が無い")
        return
    ok = sum(r["success"] for r in rows)
    print(f"== {rec.name}: 成功 {ok}/{n} = {ok / n:.1%}")
    kinds = Counter(r["kind"] for r in rows if not r["success"])
    for kind, c in kinds.most_common():
        stages = Counter(r["end_stage"] for r in rows if r["kind"] == kind and not r["success"])
        print(f"  {kind:8s} {c:3d}  段階 {dict(sorted(stages.items()))}")
    fast = [r for r in rows if r["kind"] == "関節の速さ"]
    if fast:
        print("  関節の速さの関節:", dict(Counter(r["term_joint"] for r in fast).most_common()))
        v = np.array([r["term_vel"] for r in fast])
        pre = np.array([r["vel_prev_steps"][-1] for r in fast])
        print(f"  打ち切りの歩の速さ 中央 {np.median(v):.1f}（{v.min():.1f}〜{v.max():.1f}）rad/s・その 1 歩前 中央 {np.median(pre):.1f}（{pre.min():.1f}〜{pre.max():.1f}）")
        lf = np.array([r["lever_force_end"] for r in fast])
        pf = np.array([r["panel_force_end"] for r in fast])
        print(f"  1 歩前の力 レバー 中央 {np.median(lf):.0f} N（最大 {lf.max():.0f}）・扉の板 中央 {np.median(pf):.0f} N（最大 {pf.max():.0f}）")
    okv = [r["vel_max_before"] for r in rows if r["success"] and "vel_max_before" in r]
    if okv:
        print(f"  成功した回の上半身の関節の速さの最大（21 歩目から）: 中央 {np.median(okv):.1f}・95% {np.percentile(okv, 95):.1f}・最大 {max(okv):.1f} rad/s")


def main() -> int:
    for arg in sys.argv[1:]:
        rec = Path(arg)
        st, names, summ = load(rec)
        rows = per_env(st, names, summ)
        (rec / "per_env.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
        report(rec, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
