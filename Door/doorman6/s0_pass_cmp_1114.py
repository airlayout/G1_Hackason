"""11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 I・13.4・⚠️ 推測）: 3c〜段 4 の入り（4a）を
シムで回した組（s0_i_chain_1114.sh）を、組ごとに数える。Mac で回す。

  python s0_pass_cmp_1114.py <JSON Lines>… [--json out.json]

数える物（台ごと・本当の扉の角度は毎周期の記録の蝶番）:
  届いた小段（3a・3c・4a）・3c で台本の読みが 30° に届いたか・戸口の面を越えたか（4a から done で成功）・
  本当の扉の最大・3c の入りから越えるまでの秒・止まった理由・押されて下がった回数（0114）
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
sys.path.insert(0, os.path.join(HERE, "..", "real"))
import door_script as dscr  # noqa: E402

C_T, C_SUB, C_POSE, C_HINGE = 0, 1, slice(23, 27), 27
S3A, S3C, S4A = (dscr.SUBS.index(s) for s in ("3a", "3c", "4a"))


def rows_of(jsonl: str) -> List[Dict]:
    out = []
    for line in open(jsonl):
        r = json.loads(line)
        subs = [s for s, _ in r.get("subs", [])]
        tr = f"{jsonl}.trace_env{r['env']}_ep{r['episode']}.npy"
        m = {"env": r["env"], "stop": r.get("stop", ""), "subs": subs, "push_back_n": r.get("push_back_n"),
             "push_f_peak": r.get("push_f_peak"), "door_max_true": None, "t_pass": None, "x_max": None}
        if os.path.isfile(tr):
            a = np.load(tr)
            sub = a[:, C_SUB].astype(int)
            m["door_max_true"] = round(float(np.degrees(np.max(np.abs(a[:, C_HINGE])))), 1)
            c3 = np.where(sub == S3C)[0]
            if len(c3):
                m["x_max"] = round(float(np.max(a[c3[0]:, 23])), 3)
                m["door_3c_end_true"] = round(float(np.degrees(abs(a[-1, C_HINGE]))), 1)
        m["reached_3a"] = "3a" in subs
        m["reached_3c"] = "3c" in subs
        m["reached_4a"] = "4a" in subs
        m["passed"] = m["reached_4a"] and "成功" in m["stop"]
        if m["passed"]:
            t3c = next(t for s, t in r["subs"] if s == "3c")
            m["t_pass"] = round(float(r.get("stop_t", math.nan)) - float(t3c), 1)
        out.append(m)
    return out


def summarize(tag: str, ms: List[Dict]) -> Dict:
    n = len(ms)
    c3 = [m for m in ms if m["reached_3c"]]
    reasons = Counter((m["stop"][:28] if m["stop"] else "（止まらず）") for m in c3)
    med = lambda v: float(np.median(v)) if len(v) else math.nan  # noqa: E731
    return {"tag": tag, "n": n, "n_3a": sum(m["reached_3a"] for m in ms), "n_3c": len(c3), "n_4a": sum(m["reached_4a"] for m in ms),
            "n_pass": sum(m["passed"] for m in ms),
            "door_max_true_med_3c": med([m["door_max_true"] for m in c3 if m["door_max_true"] is not None]),
            "door_max_true_p90_3c": float(np.percentile([m["door_max_true"] for m in c3], 90)) if c3 else math.nan,
            "t_pass_med": med([m["t_pass"] for m in ms if m["t_pass"] is not None]),
            "push_back_med": med([m["push_back_n"] for m in c3 if m["push_back_n"] is not None]),
            "reasons_3c": reasons.most_common(4)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", nargs="+")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    out = []
    print("  組        台  3a  3c  4a  越えた   本当の扉の最大（3c に入った台）中央・上 90%   越えるまで 秒   下がった回数   3c 以降の止まり方（多い順）")
    for j in a.jsonl:
        tag = os.path.basename(j).rsplit("_1114_", 1)[0]
        s = summarize(tag, rows_of(j))
        out.append(s)
        print(f"  {tag:9s} {s['n']:3d} {s['n_3a']:3d} {s['n_3c']:3d} {s['n_4a']:3d}   {s['n_pass']:3d}       {s['door_max_true_med_3c']:5.1f}・{s['door_max_true_p90_3c']:5.1f}"
              f"                   {s['t_pass_med']:5.1f}         {s['push_back_med']:4.1f}        {s['reasons_3c']}")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
