"""確かめ ②（と計画書の V1 ③）: 親子でない剛体どうしの当たりの形の食い込みを数える（読むだけ・Mac）。

PhysX の関節つきの体は、関節で直接つながった親子の組だけを当たりから自動で外す。それ以外の組の形が重なっていると、
PhysX は押し離す力を出す（既定の姿勢で重なっていれば起動の瞬間に弾ける）。

姿勢:
  default  VIRAL6 の既定の関節角（g1_29dof.yaml の default_joint_angles）
  zero     全関節 0
  golden   正解の記録（golden_45150.npz の dof_pos [時刻, 台, 29]）。各台の最初の dones=1 のコマまで（その次がリセット後）
  real     実機の記録（logs_real/door_real_*.npz の q）。実物は重ならないので、ここで重なる組は形が実物より大きいところ

使い方: uvx --with usd-core --with numpy --with scipy --with python-fcl python check_overlap.py [--golden PATH] [--real]
"""
from __future__ import annotations

import argparse
import glob
import json
import os
from typing import Dict, List, Tuple

import numpy as np

import g1_geom as gg

HERE = os.path.dirname(os.path.abspath(__file__))
FIXED = os.path.join(HERE, "fixed", "g1_29dof_rev_1_0_col", "g1_29dof_rev_1_0_col.usd")
ORIG = os.path.join(HERE, "orig", "g1_29dof_rev_1_0", "g1_29dof_rev_1_0.usd")
REAL_LOGS = os.path.join(HERE, "..", "..", "real", "logs_real")
LIMIT_MM = 5.0  # 合格の線（食い込み 5 mm 以下）
CONTROL_DT = 0.02  # 正解の記録の 1 コマ（fps 200・control_decimation 4）


def added_links(fixed: gg.Robot, orig: gg.Robot) -> set:
    """直した方にだけ形がある剛体（今回足した形）"""
    return {c.link for c in fixed.colliders} - {c.link for c in orig.colliders}


def label(rb: gg.Robot, p: Tuple[int, int], added: set) -> str:
    a, b = rb.colliders[p[0]], rb.colliders[p[1]]
    mark = "＋" if (a.link in added or b.link in added) else "　"
    return f"{mark}{a.label} ↔ {b.label}"


def poses_check(rb: gg.Robot, dist: gg.Distance, added: set) -> Dict[str, list]:
    out = {}
    for name, q in (("default", gg.default_q()), ("zero", np.zeros(29))):
        res = dist.scan(gg.fk(rb, q), within=0.02)
        rows = sorted((d, p) for p, d in res.items() if d < LIMIT_MM / 1000)
        out[name] = [{"pair": label(rb, p, added), "dist_mm": 1000 * d} for d, p in rows]
        new_min = min([(d, p) for p, d in res.items() if rb.colliders[p[0]].link in added
                       or rb.colliders[p[1]].link in added] or [(1.0, (-1, -1))])
        out[name + "_added_min"] = None if new_min[1][0] < 0 else \
            {"pair": label(rb, new_min[1], added), "dist_mm": 1000 * new_min[0]}
    return out


def golden_frames(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """各台の最初の dones=1 のコマまで（そのコマは連続、次のコマからリセット後）。返り値: 姿勢 [N,29]・(時刻, 台) [N,2]"""
    z = np.load(path, allow_pickle=False)
    assert tuple(str(n) for n in z["dof_names"]) == gg.DOF, "関節の並びが違う"
    Q, dn = z["dof_pos"].astype(float), z["dones"]
    qs, idx = [], []
    for e in range(Q.shape[1]):
        hit = np.where(dn[:, e] > 0.5)[0]
        last = int(hit[0]) if len(hit) else Q.shape[0] - 1
        qs.append(Q[: last + 1, e])
        idx += [(t, e) for t in range(last + 1)]
    return np.vstack(qs), np.array(idx)


def real_frames(stride: int) -> Tuple[np.ndarray, np.ndarray]:
    qs, idx = [], []
    for k, f in enumerate(sorted(glob.glob(os.path.join(REAL_LOGS, "door_real_*.npz")))):
        q = np.load(f, allow_pickle=False)["q"].astype(float)
        keep = [t for t in range(0, len(q), stride) if np.isfinite(q[t]).all()]
        qs.append(q[keep])
        idx += [(t, k) for t in keep]
    return np.vstack(qs), np.array(idx)


def replay(rb: gg.Robot, dist: gg.Distance, Q: np.ndarray, idx: np.ndarray, added: set) -> dict:
    """コマごとの最大の食い込み（全組・足した形を含む組・元からの組だけ）と、組ごとの最大"""
    worst_all = np.zeros(len(Q))
    worst_new = np.zeros(len(Q))
    worst_old = np.zeros(len(Q))
    per_pair: Dict[Tuple[int, int], Tuple[float, int]] = {}
    over5: Dict[Tuple[int, int], int] = {}
    for n, q in enumerate(Q):
        for p, d in dist.scan(gg.fk(rb, q), within=0.0).items():
            depth = -1000 * d
            if depth <= 0:
                continue
            is_new = rb.colliders[p[0]].link in added or rb.colliders[p[1]].link in added
            worst_all[n] = max(worst_all[n], depth)
            if is_new:
                worst_new[n] = max(worst_new[n], depth)
            else:
                worst_old[n] = max(worst_old[n], depth)
            if depth > per_pair.get(p, (0.0, -1))[0]:
                per_pair[p] = (depth, n)
            if depth > LIMIT_MM:
                over5[p] = over5.get(p, 0) + 1

    def frac(w: np.ndarray, th: float) -> float:
        return float((w > th).mean())

    pairs = sorted(per_pair.items(), key=lambda x: -x[1][0])
    return {
        "n": int(len(Q)),
        "over5_all": frac(worst_all, 5), "over10_all": frac(worst_all, 10),
        "over5_new": frac(worst_new, 5), "over10_new": frac(worst_new, 10),
        "over5_old": frac(worst_old, 5), "over10_old": frac(worst_old, 10),
        "max_mm": float(worst_all.max()),
        "pairs": [{"pair": label(rb, p, added), "max_mm": d, "t": int(idx[n][0]), "env_or_log": int(idx[n][1]),
                   "frames_over5": over5.get(p, 0)} for p, (d, n) in pairs],
    }


def print_replay(name: str, r: dict, time_unit: str) -> None:
    print(f"   {name}: {r['n']} コマ。食い込み 5 mm 超 {100 * r['over5_all']:.2f}%・10 mm 超 {100 * r['over10_all']:.2f}%・"
          f"最大 {r['max_mm']:.1f} mm")
    print(f"      うち足した形を含む組: 5 mm 超 {100 * r['over5_new']:.2f}%・10 mm 超 {100 * r['over10_new']:.2f}%"
          f"／元からの形どうし: 5 mm 超 {100 * r['over5_old']:.2f}%・10 mm 超 {100 * r['over10_old']:.2f}%")
    for x in r["pairs"][:12]:
        when = (f"t={x['t']}（{x['t'] * CONTROL_DT:.2f} s）台 {x['env_or_log']}" if time_unit == "golden"
                else f"記録 {x['env_or_log']} の {x['t']} 行目")
        print(f"      最大 {x['max_mm']:6.1f} mm（5 mm 超 {x['frames_over5']} コマ）  {x['pair']}  {when}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", default=FIXED)
    ap.add_argument("--orig", default=ORIG)
    ap.add_argument("--golden", default=None)
    ap.add_argument("--real", action="store_true", help="実機の記録（logs_real/door_real_*.npz）も数える")
    ap.add_argument("--real-stride", type=int, default=5)
    ap.add_argument("--hulls", default="true,physx")
    ap.add_argument("--out", default=os.path.join(HERE, "check_overlap.json"))
    a = ap.parse_args()
    result: Dict[str, dict] = {}
    gold = golden_frames(a.golden) if a.golden else None
    real = real_frames(a.real_stride) if a.real else None
    for hull in a.hulls.split(","):
        fixed = gg.load_robot(a.usd, hull=hull)
        orig = gg.load_robot(a.orig, hull=hull)
        added = added_links(fixed, orig)
        for tag, rb in (("fixed", fixed), ("orig", orig)):
            key = f"{tag}/{hull}"
            dist = gg.Distance(rb)
            print(f"\n######## {key}（既存のメッシュの凸包: {'PhysX 風の外側の近似（上限の見積り）' if hull == 'physx' else '真の凸包'}）・形 {len(rb.colliders)} 個・"
                  f"親子でない組 {len(dist.pairs)}（＋＝足した形を含む組）")
            res = {"poses": poses_check(rb, dist, added)}
            for name in ("default", "zero"):
                rows = res["poses"][name]
                bad = [r for r in rows if r["dist_mm"] < -LIMIT_MM]
                print(f"[②] {name}: 距離 5 mm 未満の組 {len(rows)}・食い込み 5 mm 超 {len(bad)}"
                      f" → {'合格' if not bad else '不合格'}")
                for r in rows:
                    print(f"      {r['dist_mm']:+7.1f} mm  {r['pair']}")
                m = res["poses"][name + "_added_min"]
                if m:
                    print(f"      足した形のいちばん近い組: {m['dist_mm']:+.1f} mm  {m['pair']}")
            if gold is not None:
                res["golden"] = replay(rb, dist, gold[0], gold[1], added)
                print_replay("[V1③] 正解の記録の再生", res["golden"], "golden")
            if real is not None:
                res["real"] = replay(rb, dist, real[0], real[1], added)
                print_replay("[参考] 実機の記録の再生", res["real"], "real")
            result[key] = res
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print(f"\n[overlap] 結果: {a.out}")


if __name__ == "__main__":
    main()
