"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K8・L2c）: 実機の段 2 で、腕の推定トルクから手のひらに掛かる外力を出し、
押した回（070921・072728）と空振りの回（073522・075139）が分かれるかを見る。numpy だけ。

外力の出し方（静的なつり合い）: 左腕の関節 j のトルク τ_j = G_j(q) − a_jᵀ((p_h − p_j) × F)
  G_j: 腕の重さを支えるトルク（URDF の部品の質量と重心・重力の向きは IMU の roll・pitch から pelvis の座標へ）
  F: 環境が手のひら（rubber_hand の原点）に掛ける力（pelvis の座標）。肩 3・肘 1 の 4 式から最小二乗で 3 成分を出す（手首は軽く雑音が大きいので使わない）
  手で押し下げるとレバーは手を上へ押し返す → 世界の上向きの成分 F_up が正
計器の確かめ: 段 1（腕は構えのまま止まっている）の F の大きさの中央が小さいこと。腕を振る間の慣性は入れていない（振り出しで偽の力が出うる）

  ../../.venv/bin/python k8_press_torque_1111.py logs_real/20261111/door_stock_20261111_0{70921,72728,73522,75139}.npz [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk

G = 9.81
ARM_JOINTS = ("left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint")
PRESSED = {"door_stock_20261111_070921.npz": True, "door_stock_20261111_072728.npz": True,
           "door_stock_20261111_073522.npz": False, "door_stock_20261111_075139.npz": False}  # 計画書 9.19 の回の一覧
SMOOTH_S = 0.1  # 力を均す窓 [s]（中央値）
SWING_END_S = 1.2  # 段 2 のこの時刻までは振り出し（レバーに届かない）。偽の力の大きさを見る


def link_masses(urdf: str) -> Dict[str, tuple]:
    """部品 → (質量, 部品の座標での重心)"""
    out = {}
    for ln in ET.parse(urdf).getroot().findall("link"):
        ine = ln.find("inertial")
        if ine is None:
            continue
        o = ine.find("origin")
        com = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        out[ln.get("name")] = (float(ine.find("mass").get("value")), com)
    return out


def descendants(chain: fk.ArmFK, link: str) -> List[str]:
    kids = [c for c, j in chain.joints.items() if j[1] == link]
    out = list(kids)
    for c in kids:
        out += descendants(chain, c)
    return out


def joint_child(chain: fk.ArmFK, name: str) -> str:
    return next(c for c, j in chain.joints.items() if j[0] == name)


def hand_force(chain: fk.ArmFK, masses: Dict[str, tuple], q: np.ndarray, tau: np.ndarray, g_pelvis: np.ndarray) -> np.ndarray:
    """1 時刻の手のひらの外力 F（pelvis の座標）。g_pelvis は重力の加速度のベクトル（pelvis の座標・大きさ G）"""
    ph = fk.palm_in_pelvis(chain, q, "left")
    rows, rhs = [], []
    for name in ARM_JOINTS:
        child = joint_child(chain, name)
        pj, Rj = chain.link_pose(child, q)
        aj = Rj @ chain.joints[child][4]
        gj = 0.0  # 重さを支えるのに要るトルク（モーターが出す側）
        for ln in [child] + descendants(chain, child):
            if ln not in masses:
                continue
            m, com = masses[ln]
            pl, Rl = chain.link_pose(ln, q)
            gj -= aj @ np.cross(pl + Rl @ com - pj, m * g_pelvis)
        rows.append(-np.cross(ph - pj, aj))  # a·((p_h − p_j) × F) = −((p_h − p_j) × a)·F
        rhs.append(gj - float(tau[chain.index[name]]))  # τ = G − a·((p_h − p_j) × F) → 行·F = G − τ
    F, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    return F


def gravity_in_pelvis(quat_wxyz: np.ndarray) -> np.ndarray:
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat_wxyz)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    R = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])  # pelvis → 水平の座標（yaw なし）
    return R.T @ np.array([0.0, 0.0, -G]), R


def smooth(x: np.ndarray, t: np.ndarray, win: float) -> np.ndarray:
    out = np.empty_like(x)
    for k in range(len(x)):
        m = (t >= t[k] - win / 2) & (t <= t[k] + win / 2)
        out[k] = np.median(x[m])
    return out


def analyze(path: Path, chain: fk.ArmFK, masses: Dict[str, tuple]) -> Dict:
    z = dict(np.load(path, allow_pickle=True))
    st, t = z["staged"], z["t"]
    in1 = np.isfinite(st[:, 0]) & (st[:, 0] == 1)
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 2] == 2)
    k0 = int(np.flatnonzero(in2)[0])
    idx = np.flatnonzero(in2)
    tt = t[idx] - t[k0]

    def series(ks):
        f_up, f_n = [], []
        for k in ks:
            g, R = gravity_in_pelvis(z["quat"][k])
            F = hand_force(chain, masses, z["q"][k], z["tau"][k], g)
            f_up.append(float((R @ F)[2]))
            f_n.append(float(np.linalg.norm(F)))
        return np.array(f_up), np.array(f_n)

    up, _ = series(idx)
    up_s = smooth(up, tt, SMOOTH_S)
    k1 = np.flatnonzero(in1)
    q1 = k1[-min(len(k1), 50):]  # 段 1 の終わりの 1 秒（止まって構えている）
    _, n1 = series(q1) if len(q1) else (np.array([]), np.array([np.nan]))
    sp = z["tau"][idx, chain.index["left_shoulder_pitch_joint"]]
    swing = tt <= SWING_END_S
    km = int(np.argmax(up_s))
    return {"file": path.name, "pressed": PRESSED.get(path.name), "policy_s": float(tt[-1]),
            "still_force_med_n": float(np.median(n1)), "f_up_max_n": float(up_s.max()), "t_f_up_max": float(tt[km]),
            "f_up_max_swing_n": float(up_s[swing].max()) if swing.any() else None,
            "f_up_max_after_swing_n": float(up_s[~swing].max()) if (~swing).any() else None,
            "shoulder_pitch_tau_absmax": float(np.abs(sp).max()),
            "f_up_marks": {f"{s:.1f}": round(float(np.interp(s, tt, up_s)), 1) for s in np.arange(0.0, min(tt[-1], 4.0) + 1e-9, 0.25)}}


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    chain = fk.load_chain(a.urdf)
    masses = link_masses(a.urdf or fk.default_urdf())
    rows = [analyze(Path(f), chain, masses) for f in a.files]
    print("記録\t押した\t段1の外力の中央 [N]\t上向きの力の最大 [N]（時刻）\t振り出しの間の最大\tその後の最大\t肩 pitch |τ| 最大")
    for r in rows:
        print(f"{r['file'][11:26]}\t{r['pressed']}\t{r['still_force_med_n']:.1f}\t{r['f_up_max_n']:.1f}（{r['t_f_up_max']:.2f} s）\t"
              f"{r['f_up_max_swing_n']:.1f}\t{r['f_up_max_after_swing_n']:.1f}\t{r['shoulder_pitch_tau_absmax']:.1f}")
    pos = [r["f_up_max_after_swing_n"] for r in rows if r["pressed"]]
    neg = [r["f_up_max_after_swing_n"] for r in rows if r["pressed"] is False]
    if pos and neg:
        print(f"押した回の最小 {min(pos):.1f} N・空振りの回の最大 {max(neg):.1f} N・比 {min(pos) / max(max(neg), 1e-6):.2f}")
    if a.json:
        Path(a.json).write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
