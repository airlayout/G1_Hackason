"""11-07（案 A）: シミュレータの脚（層＋HOMIE）が歩きの指令にどれだけ応えたかを、実機と同じ道具（Door/real/ana_stock_gain_1106）で測る。

評価の記録（records/<名前>/states_*.npz・パッチ 0095 の homie_cmd・legs_cmd 入り）と、流した列（make_loco_replay_1107.py の npz）を読む。
台 i は列 i % K を流している。各台の最初の回だけを使う（回が戻ったら打ち切る）。体の速さは root の位置と向きを
odom（x・y・向き）と見て、実機と同じ 0.5 秒の幅の中心差分で体の座標に直す。

出すもの:
  1. 掃引の列: 指令（homie_cmd）に対する体の速さ＝層＋HOMIE の全体の応え方（層が切なら HOMIE だけ）。
     脚に届いた指令（legs_cmd）に対する体の速さ＝HOMIE だけの応え方
  2. 実機の列: シミュレータの応え方と、同じ指令の列で実機が動いた応え方を並べる
  3. 実機の列ごとの動いた量（前・左・向き）: シミュレータの台の平均と実機

使い方（OMEN・doorman6 の Python）: python ana_sim_loco_resp_1107.py <records の dir> <replay の npz>
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "real"))
import ana_stock_gain_1106 as asg  # noqa: E402


def load_states(rec_dir: str) -> Dict[str, np.ndarray]:
    files = sorted(glob.glob(os.path.join(rec_dir, "states_*.npz")))
    if not files:
        raise SystemExit(f"{rec_dir} に states_*.npz が無い")
    keys = ("root_pos", "root_quat_xyzw", "episode_length", "homie_cmd", "legs_cmd")
    out: Dict[str, List[np.ndarray]] = {k: [] for k in keys}
    for f in files:
        z = np.load(f)
        missing = [k for k in keys if k not in z.files]
        if missing:
            raise SystemExit(f"{f} に {missing} が無い（パッチ 0095 の前の記録）")
        for k in keys:
            out[k].append(z[k])
    return {k: np.concatenate(v) for k, v in out.items()}  # [S, E, …]


def yaw_of(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def first_episode(ep_len: np.ndarray) -> int:
    """最初の回の長さ（回の歩数が減ったら戻った＝そこで打ち切る）"""
    d = np.diff(ep_len)
    back = np.nonzero(d < 0)[0]
    return int(back[0]) + 1 if len(back) else len(ep_len)


def per_env(st: Dict[str, np.ndarray], e: int) -> Dict[str, np.ndarray]:
    n = first_episode(st["episode_length"][:, e])
    odom = np.stack([st["root_pos"][:n, e, 0], st["root_pos"][:n, e, 1], np.unwrap(yaw_of(st["root_quat_xyzw"][:n, e]))], -1)
    t = np.arange(n) * asg.DT
    return dict(v=asg.body_vel(odom, t), cmd=st["homie_cmd"][:n, e], legs=st["legs_cmd"][:n, e], odom=odom)


def motion(odom: np.ndarray, n: int) -> np.ndarray:
    """始めの向きの座標での動いた量（前・左・向き [deg]）"""
    odom = odom[np.isfinite(odom).all(1)][:n]
    d = odom[-1] - odom[0]
    c, s = math.cos(odom[0, 2]), math.sin(odom[0, 2])
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], math.degrees(d[2])])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("records")
    ap.add_argument("replay")
    a = ap.parse_args(argv)
    st = load_states(a.records)
    rp = np.load(a.replay)
    names = [str(n) for n in rp["names"]]
    K, E = len(names), st["root_pos"].shape[1]
    envs = [per_env(st, e) for e in range(E)]
    kind = lambda e: names[e % K]  # noqa: E731
    sweep = [envs[e] for e in range(E) if kind(e).startswith("sweep_")]
    real = [envs[e] for e in range(E) if kind(e).startswith("real_")]
    print(f"[sim] 記録 {a.records}: 台 {E}・列 {K}（掃引の台 {len(sweep)}・実機の列の台 {len(real)}）・"
          f"回の長さ 中央 {np.median([len(x['v']) for x in envs]) * asg.DT:.1f} 秒", flush=True)
    if sweep:
        cat = lambda key, xs: np.concatenate([x[key] for x in xs])  # noqa: E731
        print("\n===== 1. 掃引: 方策の指令 → 体（層＋HOMIE の全体）", flush=True)
        asg.print_summary(asg.summarize(cat("cmd", sweep), cat("v", sweep)), "sim")
        if not np.allclose(cat("cmd", sweep), cat("legs", sweep)):
            print("\n===== 1b. 掃引: 脚に届いた指令 → 体（HOMIE だけ）", flush=True)
            asg.print_summary(asg.summarize(cat("legs", sweep), cat("v", sweep)), "homie")
    if real:
        print("\n===== 2. 実機の列: シミュレータ（上）と実機（下）", flush=True)
        asg.print_summary(asg.summarize(np.concatenate([x["cmd"] for x in real]), np.concatenate([x["v"] for x in real])), "sim")
        rc, rv = [], []
        for k, n in enumerate(names):
            if not n.startswith("real_"):
                continue
            o = rp["real_odom"][k]
            ok = np.isfinite(o).all(1)
            t = np.arange(ok.sum()) * asg.DT
            rc.append(rp["cmd"][k][: ok.sum()])
            rv.append(asg.body_vel(o[ok], t))
        asg.print_summary(asg.summarize(np.concatenate(rc), np.concatenate(rv)), "real")
        print("\n===== 3. 実機の列ごとの動いた量（前 m・左 m・向き °）: シミュレータの台の平均 ± ばらつき ／ 実機", flush=True)
        for k, n in enumerate(names):
            if not n.startswith("real_"):
                continue
            o = rp["real_odom"][k]
            L = int(np.isfinite(o).all(1).sum())
            sims = np.array([motion(envs[e]["odom"], L) for e in range(E) if e % K == k and len(envs[e]["odom"]) >= L])
            if not len(sims):
                continue
            m, s, r = sims.mean(0), sims.std(0), motion(o, L)
            print(f"   {n[5:]:<40} シム {m[0]:+.2f}±{s[0]:.2f} {m[1]:+.2f}±{s[1]:.2f} {m[2]:+5.0f}±{s[2]:3.0f}（{len(sims)} 台）／"
                  f"実機 {r[0]:+.2f} {r[1]:+.2f} {r[2]:+5.0f}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
