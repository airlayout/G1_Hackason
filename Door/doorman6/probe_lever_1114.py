"""11-14（計画の段 C）: 扉だけを動かす試し（s0_probe_1114.sh・パッチ 0112 の lever_probe）の記録から、シムのレバーのばねと
掛け金が、場面に入れた値（static.npz の door_lever_*）どおりに効いているかを確かめる（計器の確かめ）。

  ~/miniforge3/envs/doorman6/bin/python probe_lever_1114.py records/start_<日時> [--hmax 2.5]

台 i のレバーには hmax × i/(台数−1) の下げる回りが 1 秒から掛かる。ばねが入れた値どおりなら、落ち着いた角度は
θ = (回り − 予圧) / 硬さ（0〜止まりで切る）。4 秒から蝶番を開ける向きに押すので、θ が掛け金の外れる角度を越えた台だけ扉が開く。
"""
from __future__ import annotations

import argparse
import glob
import math

import numpy as np

SETTLE = (3.0, 3.9)      # レバーが落ち着いた角度を読む時刻 [s]（1 秒から押す）
HINGE_FROM_S = 4.0       # 蝶番を押し始める時刻 [s]
OPEN_DEG = 2.0           # これより開いたら「開いた」
START_SEARCH_STEPS = 10  # 起動直後に引き直される台がある（check_closer_in_sim.py と同じ）


def load(record: str):
    static = dict(np.load(f"{record}/static.npz", allow_pickle=True))
    files = sorted(glob.glob(f"{record}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {record}")
    chunks = [np.load(f) for f in files]
    door = np.concatenate([c["door_dof_pos"] for c in chunks])
    ep = np.concatenate([c["episode_length"] for c in chunks])
    return static, door, ep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--hmax", type=float, default=2.5)
    a = ap.parse_args(argv)
    static, door, ep = load(a.record)
    dt = float(static["dt"])
    n = door.shape[1]
    lev = {k[len("door_lever_"):]: static[k] for k in static if k.startswith("door_lever_")}
    if "spring_k" not in lev:
        print("[probe] static.npz に door_lever_* が無い（パッチ 0112 の前の env）")
        return 2
    print(f"[probe] {a.record}: 台 {n}・歩 {door.shape[0]}・扉の関節 {door.shape[2]}・dt {dt}")
    rows = []
    for i in range(n):
        resets = np.flatnonzero(ep[:START_SEARCH_STEPS, i] == 0)
        s0 = int(resets[-1]) if len(resets) else 0
        t = (np.arange(door.shape[0]) - s0) * dt
        nxt = np.flatnonzero((ep[s0 + 1:, i] == 0)) if s0 + 1 < len(ep) else []
        end = s0 + 1 + int(nxt[0]) if len(nxt) else door.shape[0]
        w = (t >= SETTLE[0]) & (t <= SETTLE[1]) & (np.arange(len(t)) < end)
        h = (t >= HINGE_FROM_S) & (np.arange(len(t)) < end)
        if not w.any():
            continue
        tau = a.hmax * i / max(1, n - 1)
        k, tau0, mx = float(lev["spring_k"][i]), float(lev["spring_tau0"][i]), float(lev["max_deg"][i])
        rel = float(lev["latch_release_deg"][i])
        th = math.degrees(float(np.mean(door[w, i, 1])))
        th_pred = min(max(math.degrees((tau - tau0) / k), 0.0), mx)
        latch = float(np.mean(door[w, i, 2])) if door.shape[2] > 2 else math.nan
        latch_pred = float(lev["latch_throw"][i]) * min(th / rel, 1.0) if math.isfinite(rel) else math.nan
        hinge = math.degrees(float(np.max(door[h, i, 0]))) if h.any() else math.nan
        opened = hinge > OPEN_DEG
        rows.append((i, tau, k, tau0, mx, th_pred, th, rel, latch_pred, latch, hinge, opened, th >= rel - 0.5))
    print(" 台   回り  硬さ  予圧  止まり  θ予想  θシム  外れ角  掛け金の引き 予想/シム mm  蝶番の最大°  開いた  θ≥外れ角")
    for r in rows:
        print(f" {r[0]:3d}  {r[1]:4.2f}  {r[2]:4.2f}  {r[3]:4.2f}  {r[4]:5.1f}  {r[5]:5.1f}  {r[6]:5.1f}  {r[7]:5.1f}"
              f"      {1000 * r[8]:5.1f} / {1000 * r[9]:5.1f}      {r[10]:6.1f}     {'○' if r[11] else '・'}      {'○' if r[12] else '・'}")
    arr = np.array([(r[5], r[6]) for r in rows])
    free = np.array([0.5 < r[5] < r[4] - 0.5 for r in rows])  # 0° と止まりで切られていない台
    err = arr[free, 1] - arr[free, 0]
    agree = sum(r[11] == r[12] for r in rows)
    print(f"[probe] ばね: 切られていない {int(free.sum())} 台で θシム − θ予想 中央 {np.median(err):+.2f}°・最大 {np.max(np.abs(err)):.2f}°")
    print(f"[probe] 掛け金: 開いた／開かないが θ≥外れ角と合う {agree}/{len(rows)} 台")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
