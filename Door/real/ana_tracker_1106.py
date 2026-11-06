"""11-06: 画像の生徒の追う段（扉の頭の値に足す補正の LSTM）が、実機でどれだけ・いつ扉の値を動かしたかを記録でなぞる。

1. READY（hold）: door_core.step は追う段の状態を進めない＝毎周期ゼロの状態から 1 歩。記録の扉の値（vis_door）が
   「扉の頭＋ゼロからの 1 歩の補正」で再現できるかを確かめる
2. DOOR: d の直後に状態をゼロに戻し（reset_actor）、周期ごとに進める。記録どおりになぞって再現を確かめ、
   扉の頭だけの値・追う段込みの値・odom で動いた分を足した値（始めの値は READY の扉の頭）を並べる

使い方（OMEN・doorman6 の Python）:
  python ana_tracker_1106.py --vision <vision_*.npz> --stock <door_stock_*.npz> --policy door_vision_e2b.npz
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402

READY, DOOR = 2, 3  # door_run_stock.PHASES の番号


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--print-every", type=float, default=0.5, help="DOOR の表の間隔 [s]")
    return ap.parse_args(argv)


def real_xy(p: dc.Params, vp: dvc.VisionParams, obs: np.ndarray, d17: np.ndarray) -> np.ndarray:
    _, pos = dvc.door_real(p, vp, dvc.insert_door(vp, obs, d17))
    return pos[:2]


def main(argv=None) -> int:  # noqa: C901
    a = parse_args(argv)
    p = dc.load_params(a.policy, mirror=True)
    vp = dvc.load_vision(a.policy, p)
    zv = np.load(a.vision, allow_pickle=True)
    feat_of = {int(k): f for k, f in zip(zv["seq"], zv["feat"])}
    zs = np.load(a.stock, allow_pickle=True)
    s = {k: zs[k] for k in ("t", "phase", "vis_seq", "vis_hg", "vis_door", "obs", "odom", "pose")}
    ph = s["phase"].astype(int)

    def feat(i: int) -> Optional[np.ndarray]:
        k = s["vis_seq"][i]
        return feat_of.get(int(k)) if np.isfinite(k) else None

    # 1. READY: ゼロからの 1 歩で再現するか
    idx = [i for i in np.nonzero(ph == READY)[0] if feat(i) is not None and np.isfinite(s["vis_door"][i]).all()]
    if idx:
        err, corr = [], []
        for i in idx[:: max(1, len(idx) // 400)]:
            actor = dvc.VisionActor(vp, feat(i), s["vis_hg"][i])
            obs_new, _ = actor.prepare(s["obs"][i], None)
            d_new = dvc.door_from_obs(vp, obs_new)
            head = dvc.door_values(vp, s["obs"][i], feat(i), s["vis_hg"][i])
            err.append(np.abs(d_new - s["vis_door"][i]).max())
            corr.append(real_xy(p, vp, s["obs"][i], d_new) - real_xy(p, vp, s["obs"][i], head))
        corr = np.array(corr)
        print(f"[trk] READY {len(idx)} 周期: ゼロからの 1 歩で記録を再現する差の最大 {max(err):.1e}・"
              f"補正（前・左）の範囲 前 {corr[:, 0].min():+.3f}〜{corr[:, 0].max():+.3f}・左 {corr[:, 1].min():+.3f}〜{corr[:, 1].max():+.3f} m",
              flush=True)

    # 2. DOOR: d の直後から状態をなぞる
    door_idx = np.nonzero(ph == DOOR)[0]
    if not len(door_idx):
        print("[trk] DOOR の周期が無い", flush=True)
        return 0
    i0 = int(door_idx[0])
    r0 = [i for i in np.nonzero(ph == READY)[0] if i < i0 and feat(i) is not None][-50:]
    head0 = np.mean([real_xy(p, vp, s["obs"][i], dvc.door_values(vp, s["obs"][i], feat(i), s["vis_hg"][i])) for i in r0], axis=0)
    o0 = s["odom"][i0]
    print(f"[trk] DOOR {len(door_idx)} 周期（{s['t'][door_idx[-1]] - s['t'][i0]:.1f} s）・始めの扉（READY の扉の頭の平均）前 {head0[0]:.3f} 左 {head0[1]:+.3f}",
          flush=True)
    state = None
    err, last_t = [], -1e9
    print("   t[s]  扉の頭 前・左      追う段込み 前・左   odom で足した 前・左   補正 前・左   向き[°]", flush=True)
    for i in door_idx:
        f = feat(i)
        if f is None:
            continue
        actor = dvc.VisionActor(vp, f, s["vis_hg"][i])
        obs_new, state = actor.prepare(s["obs"][i], state)
        d_new = dvc.door_from_obs(vp, obs_new)
        if np.isfinite(s["vis_door"][i]).all():
            err.append(np.abs(d_new - s["vis_door"][i]).max())
        t = s["t"][i] - s["t"][i0]
        if t - last_t < a.print_every:
            continue
        last_t = t
        head = real_xy(p, vp, s["obs"][i], dvc.door_values(vp, s["obs"][i], f, s["vis_hg"][i]))
        full = real_xy(p, vp, s["obs"][i], d_new)
        dx, dy, dyaw = s["odom"][i] - o0
        yaw0 = o0[2]
        # 始めの pelvis の座標での動き → 今の pelvis から見た扉の中心
        mx, my = math.cos(-yaw0) * dx - math.sin(-yaw0) * dy, math.sin(-yaw0) * dx + math.cos(-yaw0) * dy
        rx, ry = head0[0] - mx, head0[1] - my
        c, sn = math.cos(-dyaw), math.sin(-dyaw)
        od = (c * rx - sn * ry, sn * rx + c * ry)
        print(f"  {t:5.1f}  {head[0]:+.2f} {head[1]:+.2f}        {full[0]:+.2f} {full[1]:+.2f}         {od[0]:+.2f} {od[1]:+.2f}"
              f"          {full[0] - head[0]:+.2f} {full[1] - head[1]:+.2f}    {math.degrees(dyaw):+5.0f}", flush=True)
    if err:
        print(f"[trk] DOOR をなぞった再現の差の最大 {max(err):.1e}（{len(err)} 周期）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
