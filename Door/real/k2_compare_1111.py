"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K2・K6a）: ana_stage2_sim_1111.py の json を並べて比べる。numpy だけ。

各評価（Y0・Y10・Y20・Y30・W0・W20）について:
  - 扉 3° の成功（ana の opened3）・レバーに触れた回
  - 触れた所: 手のひらのレバーの付け根からの横（y。負が棒の先の側）の 10・50・90% 点と、基準（--base）の中央からのずれ
  - 触れた時の体の回り（触れた時の向き − 回の始め）の中央
  - 付け根の近くか外に下りた回（y > NEAR_PIVOT_M。11-11 の実機の 2 回は −3 cm と +3.8 cm）の割合と、その回の成功の割合
  - 壁の近くを指先が通った回（ana の tip_near_wall_eps）
回の組は評価ごとに乱数の流れが違う（外乱を引くと構えの位置の乱数がずれる）ので、台ごとの組にはせず、全体の分布で比べる。

  python k2_compare_1111.py <dir> [--names Y0 Y10 Y20 Y30 W0 W20] [--base Y0]
  <dir> に k2_・k6a_・k4_・k3_<名前>.json がある（OMEN の ~/work_inoue/ana_1111）
"""
from __future__ import annotations

import argparse
import json
import os
from typing import Dict, List, Optional

import numpy as np

NEAR_PIVOT_M = -0.03  # これより付け根の側（y が大きい）に下りたら「付け根の近くか外」


def load(d: str, name: str) -> Optional[dict]:
    for pre in ("k2_", "k6a_", "k4_", "k3_"):
        p = os.path.join(d, f"{pre}{name}.json")
        if os.path.exists(p):
            with open(p) as f:
                return json.load(f)
    return None


def stats(js: dict) -> Dict[str, float]:
    rows: List[dict] = js["rows"]
    con = [r for r in rows if r["palm_rel_lever_at_contact"] is not None]
    y = np.array([r["palm_rel_lever_at_contact"][1] for r in con])
    yaw = np.array([r["dyaw_at_contact_deg"] for r in con])
    near = np.array([v > NEAR_PIVOT_M for v in y])
    ok = np.array([r["opened3"] for r in con])
    return {
        "n": len(rows), "opened3": int(sum(r["opened3"] for r in rows)), "contact": len(con),
        "y10": float(np.percentile(y, 10)), "y50": float(np.median(y)), "y90": float(np.percentile(y, 90)),
        "yaw50": float(np.median(yaw)), "near_frac": float(near.mean()),
        "near_ok": float(ok[near].mean()) if near.any() else float("nan"),
        "far_ok": float(ok[~near].mean()) if (~near).any() else float("nan"),
        "wall_eps": int(js["summary"].get("tip_near_wall_eps", -1)), "wall_x": js["summary"].get("wall_x", -0.07),
    }


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--names", nargs="+", default=["Y0", "Y10", "Y20", "Y30", "W0", "W20"])
    ap.add_argument("--base", default="Y0")
    a = ap.parse_args(argv)
    res = {n: stats(js) for n in a.names if (js := load(a.dir, n)) is not None}
    if a.base not in res:
        print(f"[k2_compare] 基準 {a.base} の json が無い")
        return 1
    b = res[a.base]["y50"]
    print("名前\t成功\t触れた\t触れた所 y [cm] 10/50/90\tずれ [cm]\t触れた時の回り [°]\t付け根の近く・外\tその回の成功\tほかの回の成功\t壁の近く（壁 x）")
    for n, s in res.items():
        print(f"{n}\t{s['opened3']}/{s['n']} ({100 * s['opened3'] / s['n']:.1f}%)\t{s['contact']}\t"
              f"{100 * s['y10']:.1f}/{100 * s['y50']:.1f}/{100 * s['y90']:.1f}\t{100 * (s['y50'] - b):+.1f}\t{s['yaw50']:+.1f}\t"
              f"{100 * s['near_frac']:.0f}%\t{100 * s['near_ok']:.0f}%\t{100 * s['far_ok']:.0f}%\t{s['wall_eps']}（{s['wall_x']}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
