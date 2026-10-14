"""評価の記録から、機体が扉を押した力をまとめる（段 2-3: どれだけの力で押しているか）。

記録（パッチ doorman6/0034 から）:
  hand_lever_force        使う手がレバーから受ける力 [N]（x＝前へ押す・z＝上下。下へ押すと負）。押す力の報酬と同じ量
  door_panel_contact_force 扉の板が受ける接触の力の合計 [N]。閉じている間は枠からの押し返しも入るので、扉が開いている間だけ見る
台ごとの最初の 1 回（区間 0 から）について、
  手で前へ押す力の最大・下へ押す力の最大・扉が OPEN_DEG 以上開いている間に板が受けた前向きの力の 90% 点（ぶつかった瞬間の山を避ける）
を出し、成功した回と失敗した回の中央値を比べる。成功は count_success.py の summary.json の per_env。

使い方:
    python push_force_stats.py <記録のディレクトリ>... [--json out.json]
"""

import argparse
import glob
import json
import sys

import numpy as np

OPEN_DEG = 2.0  # 扉がこれ以上開いていれば、板に当たっているのは機体だけ（枠から離れている）
PANEL_PCT = 90  # 板の力は山を避けて 90% 点


def per_episode(record_dir: str) -> list[dict]:
    files = sorted(glob.glob(f"{record_dir}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {record_dir}")
    keys = ("hand_lever_force", "door_panel_contact_force", "stage", "door_dof_pos", "episode_length")
    chunks = [np.load(f) for f in files]
    if "hand_lever_force" not in chunks[0].files:
        raise KeyError(f"hand_lever_force が無い（0034 より前の記録）: {record_dir}")
    rec = {k: np.concatenate([c[k] for c in chunks]) for k in keys}
    static = np.load(f"{record_dir}/static.npz")
    closer = static["door_hinge_max_force"] if "door_hinge_max_force" in static.files else None
    per_env = {e["env"]: e for e in json.load(open(f"{record_dir}/summary.json", encoding="utf-8"))["per_env"]}
    steps, envs = rec["stage"].shape
    out = []
    for env, info in per_env.items():
        ep = rec["episode_length"][:, env]
        ends = [t for t in range(1, steps) if ep[t] < ep[t - 1]]
        b = ends[0] if ends else steps
        hand = rec["hand_lever_force"][:b, env]
        panel = rec["door_panel_contact_force"][:b, env]
        hinge_deg = np.degrees(np.abs(rec["door_dof_pos"][:b, env, 0]))
        opened = hinge_deg > OPEN_DEG
        out.append({
            "env": int(env),
            "success": bool(info["success"]),
            "closer_nm": float(closer[env]) if closer is not None else float("nan"),
            "max_hinge_deg": float(hinge_deg.max()),
            "hand_push_max_n": float(hand[:, 0].max()),
            "hand_press_max_n": float(-hand[:, 2].min()),
            "panel_push_p90_n": float(np.percentile(panel[opened, 0], PANEL_PCT)) if opened.any() else 0.0,
            "opened_steps": int(opened.sum()),
        })
    return out


def describe(rows: list[dict], label: str) -> dict:
    keys = ("hand_push_max_n", "hand_press_max_n", "panel_push_p90_n", "max_hinge_deg", "closer_nm")
    res = {"label": label, "n": len(rows)}
    for k in keys:
        res[k] = float(np.median([r[k] for r in rows])) if rows else float("nan")
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dirs", nargs="+")
    ap.add_argument("--json")
    args = ap.parse_args()
    out = {}
    for d in args.record_dirs:
        try:
            rows = per_episode(d)
        except (FileNotFoundError, KeyError) as err:
            print(f"記録を読めない: {err}", file=sys.stderr)
            return 2
        ok = [r for r in rows if r["success"]]
        ng = [r for r in rows if not r["success"]]
        print(f"== {d}  成功 {len(ok)}/{len(rows)}")
        print("   （中央値）            手で前へ押す最大  手で下へ押す最大  板を前へ押す 90%点  扉の最大角  クローザー")
        for s in (describe(ok, "成功"), describe(ng, "失敗"), describe(rows, "全部")):
            print(f"   {s['label']:4s} {s['n']:4d} 回  {s['hand_push_max_n']:10.0f} N  {s['hand_press_max_n']:12.0f} N"
                  f"  {s['panel_push_p90_n']:14.0f} N  {s['max_hinge_deg']:8.1f}°  {s['closer_nm']:6.1f} N·m")
        out[d] = {"episodes": rows, "success": describe(ok, "成功"), "fail": describe(ng, "失敗"), "all": describe(rows, "全部")}
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
