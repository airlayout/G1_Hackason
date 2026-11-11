"""11-09: r1c7 が実機でレバーの前まで来て手を伸ばさない（4 回とも）。記録した観測を方策に入れ直し、入力を 1 群ずつ差し替えて
左腕が「伸ばす向き」の行動を出すようになるかを見る（開ループ。差し替えても体は記録どおりに動いたことにする）。

シム（eval_1109_r1c7・251 台）では段 1（pelvis が把手の 0.22 m 手前・横 +0.04 m）から 1.8 秒で、左肘 0.37 → −0.66・
左肩 roll −0.07 → +0.27・左肩 pitch +0.26 → −0.15 と伸ばしてレバーを押す。実機は同じ所で 15 秒伸ばさなかった。

  Navigation/.venv/bin/python Door/real/ana_reach_1109.py --stock <door_stock_*.npz> --vision <vision_*.npz> \
      --policy Door/real/logs_real/policy/vision/door_vision_r1c7.npz [--win 5:20]

出すもの: 各差し替えで、窓の間に方策が積もうとした腕の角度の変化（action_scale × delta_action_scale × Σ a_real [rad]。
腕の切り（今の角度 ±δ）は通さない）。左肘が −・左肩 roll が +・左肩 pitch が − へ大きく動けば「伸ばすつもり」。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402

READY, DOOR = 2, 3  # door_run_stock.PHASES の番号
ARM_NAMES = ("L肩p", "L肩r", "L肩y", "L肘", "R肩p", "R肩r", "R肩y", "R肘")
ARM_ACT = (3, 4, 5, 6, 10, 11, 12, 13)  # 行動 17 = [歩き 3, 左腕 7, 右腕 7]
SIM_REACH = {"L肩p": -0.41, "L肩r": +0.34, "L肘": -1.03, "R肩y": +0.48, "R肘": -0.47}  # シムの段 1 → 押した時の中央値の変化
LEG_TERMS_DOF = slice(0, 15)  # 脚 12 と腰 3（dof_pos・dof_vel・actions の中の位置）


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--win", default="5:20", help="DOOR の始めからの窓 [s]（a:b）")
    return ap.parse_args(argv)


def term(p: dc.Params, name: str) -> slice:
    a, b = p.layout[name]
    return slice(a, b)


def replay(p: dc.Params, vp: dvc.VisionParams, obs: np.ndarray, feats: np.ndarray, hgs: np.ndarray) -> np.ndarray:
    """DOOR の周期を始めから LSTM でなぞる（h・c は 0 から）。返り値は a_pol（鏡写しの側）[n, 17]"""
    hidden = vp.lstm[0][1].shape[1]
    h = np.zeros((len(vp.lstm), hidden))
    c = np.zeros((len(vp.lstm), hidden))
    out = []
    for o, f, g in zip(obs, feats, hgs):
        a, h, c = dvc.VisionActor(vp, f, g).forward(o, h, c)
        out.append(a)
    return np.array(out)


def intended_change(p: dc.Params, a_pol: np.ndarray, m: np.ndarray) -> np.ndarray:
    """窓 m の間に積もうとした腕の角度の変化 [rad]（実の向き・切りなし）"""
    a_real = a_pol @ p.act_mirror.T if p.mirror else a_pol
    return p.c["action_scale"] * p.c["delta_action_scale"] * a_real[m].sum(axis=0)


def main(argv=None) -> int:
    a = parse_args(argv)
    p = dc.load_params(a.policy, mirror=True)
    vp = dvc.load_vision(a.policy, p)
    zs = np.load(a.stock, allow_pickle=True)
    zv = np.load(a.vision, allow_pickle=True)
    feat_of = {int(k): f for k, f in zip(zv["seq"], zv["feat"])}
    ph, t = zs["phase"].astype(int), zs["t"]

    def feats_of(idx: np.ndarray) -> np.ndarray:
        last = None
        out = []
        for i in idx:
            k = zs["vis_seq"][i]
            f = feat_of.get(int(k)) if np.isfinite(k) else None
            last = f if f is not None else last
            out.append(last)
        return np.array(out)

    di = np.nonzero(ph == DOOR)[0]
    ri = np.nonzero(ph == READY)[0][-100:]  # READY の最後の 2 秒
    obs, hg, feats = zs["obs"][di].astype(float), zs["vis_hg"][di], feats_of(di)
    td = t[di] - t[di[0]]
    w0, w1 = (float(v) for v in a.win.split(":"))
    m = (td >= w0) & (td < w1)

    base = replay(p, vp, obs, feats, hg)
    err = np.abs(base - zs["a_pol"][di]).max()
    print(f"[reach] DOOR {len(di)} 周期・窓 {w0:g}〜{w1:g} s（{m.sum()} 周期）・記録の a_pol の再現の差の最大 {err:.1e}")

    obs_r = zs["obs"][ri].astype(float).mean(axis=0)
    feat_r = feats_of(ri).mean(axis=0)
    hg_r = zs["vis_hg"][ri].mean(axis=0)
    fl = term(p, "door_frame_loc")

    def with_terms(names, src: np.ndarray, sub: Optional[slice] = None, base_obs: Optional[np.ndarray] = None) -> np.ndarray:
        o = (obs if base_obs is None else base_obs).copy()
        for n in names:
            s = term(p, n)
            if sub is not None:
                s = slice(s.start + sub.start, s.start + sub.stop)
            o[:, s] = src[s]
        return o

    def closer(dx: float) -> np.ndarray:
        o = obs.copy()
        o[:, fl.start] -= dx  # 開口の中心の前（pelvis から）を dx だけ近く（鏡写しでも前は変わらない）
        return o

    cases: Dict[str, Callable[[], np.ndarray]] = {
        "記録どおり": lambda: base,
        "画像の特徴を READY の平均に": lambda: replay(p, vp, obs, np.repeat(feat_r[None], len(di), 0), hg),
        "画像の特徴を 0 に": lambda: replay(p, vp, obs, np.zeros_like(feats), hg),
        "頭の重力を READY の平均に": lambda: replay(p, vp, obs, feats, np.repeat(hg_r[None], len(di), 0)),
        "扉を 5 cm 近く": lambda: replay(p, vp, closer(0.05), feats, hg),
        "扉を 10 cm 近く": lambda: replay(p, vp, closer(0.10), feats, hg),
        "扉を 5 cm 遠く": lambda: replay(p, vp, closer(-0.05), feats, hg),
        "脚と腰の角度・速さ・行動を READY に": lambda: replay(
            p, vp, with_terms(("dof_pos", "dof_vel", "actions"), obs_r, LEG_TERMS_DOF), feats, hg),
        "胴の角速度・重力を READY に": lambda: replay(
            p, vp, with_terms(("base_ang_vel", "projected_gravity"), obs_r), feats, hg),
        "脚・胴の揺れを全部 READY に": lambda: replay(
            p, vp, with_terms(("base_ang_vel", "projected_gravity"), obs_r,
                              base_obs=with_terms(("dof_pos", "dof_vel", "actions"), obs_r, LEG_TERMS_DOF)), feats, hg),
    }
    head = "".join(f"{n:>7s}" for n in ARM_NAMES)
    print(f"{'差し替え':32s}{head}   伸ばす向きとの一致")
    sim = "".join(f"{SIM_REACH.get(n, float('nan')):+7.2f}" for n in ARM_NAMES)
    print(f"{'（シムの段 1 → 押した時の変化）':28s}{sim}")
    for name, fn in cases.items():
        ch = intended_change(p, fn(), m)
        arm = ch[list(ARM_ACT)]
        ref = np.array([SIM_REACH.get(n, 0.0) for n in ARM_NAMES])
        score = float(arm @ ref / (np.linalg.norm(ref) + 1e-9))
        print(f"{name:32s}" + "".join(f"{v:+7.2f}" for v in arm) + f"   {score:+.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
