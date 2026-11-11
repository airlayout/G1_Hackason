"""11-07（案 A）: パッチ 0095 の応え方の層（env.config.loco_resp）の値を、実機の記録とシミュレータの HOMIE の掃引から決める。

  H(u): 層を切ったシミュレータで、脚に届いた指令 u を 3 秒保ったときの体の速さ（後ろの 2 秒の平均・台の平均・向きごと）
  R(c): 実機の純正で、指令 c に対して出た速さ（Door/real/ana_stock_gain_1106.summarize の大きさごとの平均・向きごと）
  流れ: 軸ごとに、その軸の指令がほぼ 0 で、ほかの軸で歩いている周期の実機の速さの平均（先に出して、下の倍率からは引く）
  層の倍率: u = H⁻¹(R(c) − 流れ) になるよう、knots（ana_stock_gain_1106.BINS の区間の真ん中）での u / c。
    HOMIE の取りこぼしを二重に数えない。区間の周期が足りなければ逆の向き、それも無ければ全体の傾き
  遅れ: 実機の遅れ − HOMIE の遅れ（0 未満は 0）。回ごとに ±30% の幅
  効かない帯: 0 から続く区間で、流れを引いた速さが指令の 15% 未満の所の上の端（向きの大きい方。無ければ 0）
出力: --out の json（loco_resp の値）と、評価用（,）・学習の連鎖用（|）の hydra の引数

使い方（OMEN）: python fit_loco_resp_1107.py --sim <層を切った掃引の records> --replay <replay の npz> --out <json> <実機の door_stock_*.npz>
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "real"))
import ana_sim_loco_resp_1107 as asl  # noqa: E402
import ana_stock_gain_1106 as asg  # noqa: E402

AXIS = ("vx", "vy", "wz")
DEAD_RATIO = 0.15
DELAY_SPREAD = 0.3


def sim_curve(rec: str, replay: str) -> Tuple[Dict[Tuple[int, int], List[Tuple[float, float]]], List[float]]:
    """{(軸, 向き): [(u, 速さ)…]} と軸ごとの HOMIE の遅れ [s]"""
    st = asl.load_states(rec)
    rp = np.load(replay)
    names = [str(n) for n in rp["names"]]
    K, E = len(names), st["root_pos"].shape[1]
    pts: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}
    lag_c: List[List[np.ndarray]] = [[], [], []]
    for e in range(E):
        n = names[e % K]
        if not n.startswith("sweep_"):
            continue
        ax = AXIS.index(n.split("_")[1])
        x = asl.per_env(st, e)
        legs, v = x["legs"][:, ax], x["v"][:, ax]
        for sgn in (1, -1):
            hold = np.nonzero(np.sign(legs) == sgn)[0]
            if len(hold) < 100:
                continue
            seg = hold[hold >= hold[0] + int(1.0 / asg.DT)]  # 保った 3 秒のうち後ろの 2 秒
            u, vel = float(np.mean(legs[seg])), float(np.nanmean(v[seg]))
            pts.setdefault((ax, sgn), []).append((abs(u), sgn * vel))
        lag_c[ax].append(np.stack([legs, v], -1))
    lags = []
    for ax in range(3):
        cv = np.concatenate(lag_c[ax])
        lags.append(asg.best_lag(cv[:, 0], cv[:, 1]) * asg.DT)
    curve = {}
    for key, p in pts.items():
        arr = np.array(p)
        us = np.unique(np.round(arr[:, 0], 4))
        curve[key] = [(float(u), float(np.mean(arr[np.isclose(arr[:, 0], u, atol=1e-3), 1]))) for u in us]
    return curve, lags


WALK_SPEED = 0.02  # 掃引でこれより速ければ HOMIE は歩いていた（立つ／歩くの線より下の点は逆算に使わない）


def invert(curve: List[Tuple[float, float]], speed: float) -> float:
    """H(u)＝speed になる u。HOMIE が歩いている点の倍率 H(u)/u だけを u で折れ線にし（外は端の倍率）、u = speed / 倍率(u) を解く。
    11-07: 掃引は 1 軸ずつ送るので、旋回だけ 0.2 rad/s 以下・前後だけ 0.05 m/s 以下では HOMIE が立ったまま（尺度を掛けた指令の
    大きさ 0.1 の線）。実機の列では他の軸も出ていて HOMIE は歩いているので、線の下の点で逆算すると倍率を大きく見積もる"""
    walk = [(u, h) for u, h in curve if h > WALK_SPEED and u > 0]
    if not walk or speed <= 0:
        return 0.0
    us = np.array([u for u, _ in walk])
    ratio = np.array([h / u for u, h in walk])
    u = speed / ratio[0]
    for _ in range(20):
        u = speed / float(np.interp(u, us, ratio))
    return float(u)


MIN_N = 25  # 区間の平均に使う最少の周期（0.5 秒）
ZERO_CMD = (0.02, 0.02, 0.05)  # この軸の指令がほぼ 0（流れを測る）
WALK_CMD = (0.05, 0.05, 0.1)   # ほかの軸で歩いている


def real_shifted(c: np.ndarray, v: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray], List[float]]:
    """軸ごとに遅れをずらした (指令, 速さ) と遅れ [s]"""
    cs, vs, lags = [], [], []
    for ax in range(3):
        lag = asg.best_lag(c[:, ax], v[:, ax])
        a, b = c[: len(c) - lag], v[lag:, ax]
        m = np.isfinite(b)
        cs.append(a[m])
        vs.append(b[m])
        lags.append(lag * asg.DT)
    return cs, vs, lags


BIAS_AXES = (1,)  # 11-07: 流れは横だけ（右へ流れるのが一番はっきりしている。前後・旋回に入れると小さい指令を逆向きに読む）


def fit(curve, sim_lags, c: np.ndarray, v: np.ndarray, scale=(1.0, 1.0, 1.0)) -> dict:
    """scale: 軸ごとに倍率へ一律に掛ける（実機の列で層＋HOMIE の傾きを実機に合わせる答え合わせの後で）"""
    cs, vs, real_lags = real_shifted(c, v)
    knots, gp, gn, dead, bias, delay = [], [], [], [], [], []
    for ax in range(3):
        cc, vv = cs[ax], vs[ax]
        a = cc[:, ax]
        # 流れ: この軸の指令がほぼ 0 で、ほかの軸で歩いている周期の速さの平均
        others = [o for o in range(3) if o != ax]
        walk = np.zeros(len(a), dtype=bool)
        for o in others:
            walk |= np.abs(cc[:, o]) > WALK_CMD[o]
        zero = (np.abs(a) < ZERO_CMD[ax]) & walk
        b0 = float(np.mean(vv[zero])) if (ax in BIAS_AXES and zero.sum() >= MIN_N) else 0.0
        bias.append(round(b0, 4))
        vc = vv - np.where(np.abs(a) > 0, b0, 0.0)  # 流れを引いた速さ
        edges = asg.BINS[ax]
        mids = [(lo + hi) / 2 for lo, hi in zip(edges[:-1], edges[1:])]
        nz = np.abs(a) > 1e-3
        slope = float(np.sum(a[nz] * vc[nz]) / np.sum(a[nz] ** 2)) if nz.any() else 0.0
        ratios = {1: [], -1: []}
        for lo, hi in zip(edges[:-1], edges[1:]):
            for sgn in (1, -1):
                sel = (sgn * a > lo) & (sgn * a <= hi)
                ratios[sgn].append(float(np.mean(vc[sel]) / np.mean(a[sel])) if sel.sum() >= MIN_N else None)
        g = {}
        for sgn in (1, -1):
            other = ratios[-sgn]
            r = [x if x is not None else (other[i] if other[i] is not None else slope) for i, x in enumerate(ratios[sgn])]
            g[sgn] = [round(scale[ax] * (max(invert(curve[(ax, sgn)], max(ri, 0.0) * k) / k, 0.0) if (ax, sgn) in curve else ri), 4)
                      for ri, k in zip(r, mids)]
            ratios[sgn] = r
        knots.append([round(k, 4) for k in mids])
        gp.append(g[1])
        gn.append(g[-1])
        # 効かない帯: 0 から続く区間で、流れを引いた速さが DEAD_RATIO 未満の所の上の端（向きの大きい方）
        d = 0.0
        for sgn in (1, -1):
            for (lo, hi), r in zip(zip(edges[:-1], edges[1:]), ratios[sgn]):
                if r < DEAD_RATIO:
                    d = max(d, hi)
                else:
                    break
        dead.append(round(d, 3))
        dl = max(real_lags[ax] - sim_lags[ax], 0.0) / asg.DT
        delay.append([int(round(dl * (1 - DELAY_SPREAD))), int(round(dl * (1 + DELAY_SPREAD)))])
    return dict(enabled=True, delay_steps=delay, dead=dead, knots=knots, gain_pos=gp, gain_neg=gn,
                bias=bias, gain_rand=0.15, bias_rand=0.5)


def hydra_args(resp: dict, sep: str) -> str:
    def lit(v):
        if isinstance(v, list):
            return "[" + sep.join(lit(x) for x in v) + "]"
        return str(v).lower() if isinstance(v, bool) else str(v)

    return (" " if sep == "," else ",").join(f"++env.config.loco_resp.{k}={lit(v)}" for k, v in resp.items())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", required=True)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", default="1,1,1", help="軸ごとに倍率へ一律に掛ける（前後,横,旋回）")
    ap.add_argument("records", nargs="+")
    a = ap.parse_args(argv)
    scale = tuple(float(x) for x in a.scale.split(","))
    curve, sim_lags = sim_curve(a.sim, a.replay)
    print("[fit] HOMIE の掃引（脚に届いた指令 u → 保った間の速さ）・遅れ " + "・".join(f"{AXIS[i]} {sim_lags[i]:.2f} s" for i in range(3)))
    for (ax, sgn), pts in sorted(curve.items()):
        print(f"   {AXIS[ax]} {'+' if sgn > 0 else '−'}: " + "  ".join(f"{u:.2f}→{v:.3f}" for u, v in pts))
    c, v = asg.collect(a.records)
    resp = fit(curve, sim_lags, c, v, scale)
    Path(a.out).write_text(json.dumps(resp, ensure_ascii=False, indent=1))
    print(f"[fit] 書いた: {a.out}\n{json.dumps(resp, ensure_ascii=False)}")
    print(f"[fit] 評価の引数:\n{hydra_args(resp, ',')}")
    print(f"[fit] 学習の連鎖の引数:\n{hydra_args(resp, '|')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
