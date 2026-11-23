"""11-16（計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md・⚠️ シムだけ）: 3c の押し方の新案の組（s0_n_chain_1116.sh）を
組ごとに並べる。Mac で回す。

  python s0_n_cmp_1116.py <JSON Lines>… [--json out.json]

台ごと（3c に入った台だけ）の値を組で中央値にする:
  e（効く横のずれ＝ΣMz÷Σ押し返し [m]・正は把手の側＝左）・|Mz| の平均 [N·m]・押し返しの中央・90%・押している間の振れ（標準偏差）[N]・
  右手の受け持ち・右手が板に触れていた割合・手首の位置（pelvis の向きの座標）・3c〜4a の体の回り [°]（毎周期の記録の pelvis の向き）
結果: 3c・4a に入った台・戸口の面を越えた台・本当の扉の最大（s0_pass_cmp_1114.rows_of）
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import Counter
from typing import Dict, List

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import s0_pass_cmp_1114 as pc  # noqa: E402

C_YAW = 25


def med(v) -> float:
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.median(v)) if v else math.nan


def yaw_turn_deg(trace: str) -> float:
    """3c に入ってからの pelvis の向きの変わり（終わり − 入り）[°]。左回りが正"""
    if not os.path.isfile(trace):
        return math.nan
    a = np.load(trace)
    sub = a[:, pc.C_SUB].astype(int)
    k = np.where((sub == pc.S3C) | (sub == pc.S4A))[0]
    if len(k) < 2:
        return math.nan
    yaw = np.unwrap(a[k, C_YAW])
    return float(np.degrees(yaw[-1] - yaw[0]))


def tag_rows(jsonl: str) -> List[Dict]:
    base = {m["env"]: m for m in pc.rows_of(jsonl)}
    out = []
    for line in open(jsonl):
        r = json.loads(line)
        m = dict(base[r["env"]])
        m["panel"] = r.get("panel", {}) or {}
        # 4a から done へ移っていれば越えた（扉が 90° まで開くと環境が回を戻し、done で 1 秒保った後の「成功」が書かれない回がある）
        subs = [x for x, _ in r.get("subs", [])]
        m["passed"] = m["passed"] or ("4a" in subs and subs[-1] == "done")
        m["yaw_turn"] = yaw_turn_deg(f"{jsonl}.trace_env{r['env']}_ep{r['episode']}.npy")
        out.append(m)
    return out


def summarize(tag: str, ms: List[Dict]) -> Dict:
    c3 = [m for m in ms if m["reached_3c"] and m["panel"].get("n", 0) > 0]
    P = [m["panel"] for m in c3]
    wr = [p.get("wrist_body_med") for p in P if p.get("wrist_body_med")]
    reasons = Counter((m["stop"][:24] if m["stop"] else "（止まらず）") for m in c3)
    return {
        "tag": tag, "n": len(ms), "n_3c": sum(m["reached_3c"] for m in ms), "n_4a": sum(m["reached_4a"] for m in ms),
        "n_pass": sum(m["passed"] for m in ms),
        "door_max_true_med_3c": med([m["door_max_true"] for m in c3]),
        "e_med": med([p.get("e") for p in P]), "mz_abs_med": med([p.get("mz_abs_mean") for p in P]),
        "push_med": med([p.get("push_med") for p in P]), "push_p90": med([p.get("push_p90") for p in P]),
        "push_std": med([p.get("push_std_on") for p in P]), "right_share": med([p.get("right_share") for p in P]),
        "right_touch": med([p.get("right_touch_frac") for p in P]), "torso_frac": med([p.get("torso_frac") for p in P]),
        "yaw_turn_med": med([m["yaw_turn"] for m in c3]),
        "wrist_med": np.round(np.median(np.array(wr), axis=0), 2).tolist() if wr else None,
        "stops": reasons.most_common(3),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    rows = []
    for j in a.jsonl:
        tag = os.path.basename(j).rsplit("_1116_", 1)[0].rsplit("_1117_", 1)[0]
        rows.append(summarize(tag, tag_rows(j)))
    print(f"{'組':12s} 台 3c 4a 越え  扉の最大  e[m]   |Mz|  押し返し 中央・90%・振れ  右手の分 触れた  体の回り  手首(左前・左横・右前・右横)")
    for s in rows:
        print(f"{s['tag']:12s} {s['n']:2d} {s['n_3c']:2d} {s['n_4a']:2d} {s['n_pass']:3d}   {s['door_max_true_med_3c']:5.1f}°  "
              f"{s['e_med']:+.3f}  {s['mz_abs_med']:4.1f}  {s['push_med']:5.1f}・{s['push_p90']:5.1f}・{s['push_std']:4.1f}     "
              f"{s['right_share']:.2f}    {s['right_touch']:.2f}   {s['yaw_turn_med']:+6.1f}°  {s['wrist_med']}")
    for s in rows:
        print(f"  {s['tag']}: 止まり方 {s['stops']}")
    if a.json:
        json.dump(rows, open(a.json, "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
