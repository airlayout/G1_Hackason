"""11-12 台本（S8）: 台本の回（door_run_stock.py --staged --script）の記録 1 本を 1 枚にまとめる。回と回の間に最初に見る。

  python ana_script_run.py logs_real/20261112/door_stock_<日時>.npz [--door 0.94,2.05,0.99,0.064]

出すもの: 止まった理由・小段ごとの時間と終わりの値（手のひら−当てる点・扉の角度・肩 pitch と肘の力・体の回り）・
触れた時と下げた時の値・脚の指令を出した回数と pelvis の進み・腕の温度の最高・周期。
"""
from __future__ import annotations

import argparse
import ast
import math
from typing import Dict

import numpy as np

import door_script as dscr
import door_staged as ds

L_ARM = slice(15, 22)
NAMES = ("肩pitch", "肩roll", "肩yaw", "肘", "手首roll", "手首pitch", "手首yaw")


def load_meta(z) -> Dict:
    m = z["meta"]
    m = m.item() if m.shape == () else m
    return m if isinstance(m, dict) else ast.literal_eval(str(m))


def yaw_deg(quat: np.ndarray) -> np.ndarray:
    w, x, y, zz = quat.T
    return np.degrees(np.arctan2(2 * (w * zz + x * y), 1 - 2 * (y * y + zz * zz)))


def summarize(path: str, door: str) -> str:
    z = np.load(path, allow_pickle=True)
    if "script" not in z.files:
        return f"[ana] {path}: 台本の列（script）が無い（--script の回ではない）"
    meta = load_meta(z)
    w, _, lh, bs = (float(v) for v in (meta.get("args", {}).get("door") or door).split(","))
    ref = ds.lever_ref_point(w, bs, lh, not meta.get("args", {}).get("lever_right", False))
    t, sc, tau, q, pose = z["t"], z["script"], z["tau"], z["q"], z["pose"]
    ok = np.isfinite(sc[:, 0])
    out = [f"[ana] {path}", f"[ana] 止まった理由: {meta.get('reason', '?')}"]
    if not ok.any():
        return "\n".join(out + ["[ana] 台本に入っていない（段 0・1 で止まった）"])
    t0 = t[ok][0]
    yaw = yaw_deg(z["quat"])
    sub = sc[:, 0]
    idx = np.where(ok)[0]
    starts = [idx[0]] + [k for a, k in zip(idx[:-1], idx[1:]) if sub[k] != sub[a]]
    out.append("[ana] 小段   始まり  長さ   手のひら−当てる点 [前,横,上] cm   扉°   肩pitch/肘の力 N·m（終わり・最大）  体の回り°")
    for n, k0 in enumerate(starts):
        k1 = (starts[n + 1] - 1) if n + 1 < len(starts) else idx[-1]
        seg = slice(k0, k1 + 1)
        d = 100 * (sc[k1, 4:7] - ref)
        tp, te = tau[seg, 15], tau[seg, 18]
        out.append(f"[ana] {dscr.SUBS[int(sub[k0])]:7s} {t[k0] - t0:6.2f} {t[k1] - t[k0]:5.2f}   [{d[0]:+6.1f},{d[1]:+6.1f},{d[2]:+6.1f}]"
                   f"            {sc[k1, 7]:5.1f}   {tp[-1]:+5.1f}/{te[-1]:+5.1f}・{np.abs(tp).max():4.1f}/{np.abs(te).max():4.1f}"
                   f"            {yaw[k1] - yaw[idx[0]]:+5.1f}")
    lag = sc[ok, 8]
    walk = z["vel_want"][ok][:, 0] > 0
    pulses = int(np.sum(walk[1:] & ~walk[:-1]) + (1 if walk[0] else 0))
    p_ok = pose[ok]
    out.append(f"[ana] 扉の最大 {np.nanmax(sc[ok, 7]):.1f}°・遅れ（手のひら−行き先の上下）最大 {100 * np.nanmax(np.abs(lag)) if np.isfinite(lag).any() else float('nan'):.1f} cm・"
               f"手の前への出 最大 {np.nanmax(sc[ok, 10]):.2f} m")
    out.append(f"[ana] 脚: 指令を出した回数 {pulses}・出していた時間 {walk.sum() * 0.02:.1f} 秒・pelvis の進み 前 {100 * (p_ok[-1, 0] - p_ok[0, 0]):+.1f} "
               f"横 {100 * (p_ok[-1, 1] - p_ok[0, 1]):+.1f} cm・向き {math.degrees(p_ok[-1, 2] - p_ok[0, 2]):+.1f}°")
    goal_err = np.linalg.norm(sc[ok, 1:4] - sc[ok, 4:7], axis=1)
    out.append(f"[ana] 行き先と手のひらの差: 中央 {100 * np.nanmedian(goal_err):.1f} cm・最大 {100 * np.nanmax(goal_err):.1f} cm")
    sent, q_l = z["arm_sent"][ok][:, :7], q[ok][:, L_ARM]
    lead = np.abs(sent - q_l).max(axis=0)
    out.append("[ana] 送った目標と実の角度の差の最大 [rad]: " + "・".join(f"{n} {v:.2f}" for n, v in zip(NAMES, lead)))
    temp = z["arm_temp"][ok]
    out.append(f"[ana] 腕の温度: 始め 最高 {np.nanmax(temp[0]):.0f} °C → 終わり 最高 {np.nanmax(temp[-1]):.0f} °C（関節 {int(np.nanargmax(temp[-1]))}）・"
               f"周期 中央 {1000 * np.median(z['period'][ok]):.1f} ms・最大 {1000 * z['period'][ok].max():.1f} ms")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--door", default="0.94,2.05,0.99,0.064")
    a = ap.parse_args(argv)
    for f in a.npz:
        print(summarize(f, a.door))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
