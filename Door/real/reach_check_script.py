"""11-12 台本の机の上の確かめ（計画書 3 節）: 構えの位置から、台本の行き先（経由点・レバーの上・押し下げ・扉 0〜30°）へ
左腕が可動域の内で届くかを逆運動学で出す。機体もシムも使わない。

  python reach_check_script.py [script_params.json] [--door 0.94,2.05,0.99,0.064] [--pelvis-z 0.73]

扉が開いた先は、pelvis を「手の前への出が reach_lo〜reach_hi に収まる所」まで前へ進めた姿勢でも確かめる（脚が出た後）。
"""
from __future__ import annotations

import argparse
import math
from typing import List, Tuple

import numpy as np

import door_frame as dfr
import door_script as dscr
import door_staged as ds
import g1_arm_fk as fk
import g1_arm_ik as aik
import hand_door_angle as hda

HOME7 = np.array([0.29, 0.2, -0.02, 0.99, 0.15, 0.02, -0.02])  # 11-11 の記録の、純正が腕を下ろしている角度


def to_pelvis(goal: np.ndarray, pose: dfr.PlanarPose, pelvis_z: float) -> np.ndarray:
    return dfr.rot_z(pose.yaw).T @ (goal - np.array([pose.x, pose.y, pelvis_z]))


def door_point(line: hda.DoorLine, ref: np.ndarray, deg: float) -> np.ndarray:
    v = ref[:2] - line.hinge_xy
    a = line.open_sign * math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    xy = line.hinge_xy + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
    return np.array([xy[0], xy[1], ref[2]])


def check(p: dscr.ScriptParams, door: Tuple[float, float, float, float], pelvis_z: float) -> List[Tuple[str, float, float, float]]:
    """[(名前, 残りの差 m, 腕の伸び m, 可動域の端までの最小 rad)]"""
    w, _, lh, bs = door
    chain = fk.load_chain()
    ik = aik.ArmIK(chain, "left")
    ref = ds.lever_ref_point(w, bs, lh, True)
    line = hda.door_line(w, True)
    st = dscr.script_stance(p, w, bs, lh, True)
    pose = st.target
    q29 = np.zeros(29)
    posture = np.asarray(p.posture)
    pts = [(f"経由点 {k}", ref + np.array(v)) for k, v in enumerate(p.via)] + [("レバーの上", ref + np.array(p.above)), ("触れる", ref.copy()),
           ("押し下げた", ref - np.array([0, 0, p.press_depth + p.hold_extra]))]
    pressed = ref - np.array([0, 0, p.press_depth])
    rows, q7 = [], HOME7.copy()
    for name, g in pts:
        q7, err = ik.solve(q29, q7, to_pelvis(g, pose, pelvis_z), posture)
        rows.append((name, err, aik.elbow_reach(chain, ik._full(q29, q7)), float(min((q7 - ik.lo).min(), (ik.hi - q7).min())), q7.copy()))
    for deg in (3, 6, 10, 15, 20, 25, 30):
        g = door_point(line, pressed, deg)
        q_s, err = ik.solve(q29, q7, to_pelvis(g, pose, pelvis_z), posture)
        rows.append((f"扉 {deg}°（脚は構えのまま）", err, aik.elbow_reach(chain, ik._full(q29, q_s)),
                     float(min((q_s - ik.lo).min(), (ik.hi - q_s).min())), q_s.copy()))
        fwd = to_pelvis(g, pose, pelvis_z)[0]
        if fwd > p.reach_hi:  # 脚が出て、手の前への出が reach_lo と reach_hi の真ん中に戻った姿勢
            adv = fwd - 0.5 * (p.reach_lo + p.reach_hi)
            moved = dfr.PlanarPose(pose.x + adv * math.cos(pose.yaw), pose.y + adv * math.sin(pose.yaw), pose.yaw)
            q_m, err = ik.solve(q29, q7, to_pelvis(g, moved, pelvis_z), posture)
            rows.append((f"扉 {deg}°（脚が {100 * adv:.0f} cm 前へ）", err, aik.elbow_reach(chain, ik._full(q29, q_m)),
                         float(min((q_m - ik.lo).min(), (ik.hi - q_m).min())), q_m.copy()))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("params", nargs="?", default=None)
    ap.add_argument("--door", default="0.94,2.05,0.99,0.064")
    ap.add_argument("--pelvis-z", type=float, default=0.73)
    a = ap.parse_args(argv)
    p = dscr.load_params(a.params)
    door = tuple(float(v) for v in a.door.split(","))
    st = dscr.script_stance(p, door[0], door[3], door[2], True).target
    print(f"[reach] 構え x {st.x:+.3f} y {st.y:+.3f} 向き {math.degrees(st.yaw):+.0f}°・pelvis の高さ {a.pelvis_z} m")
    bad = 0
    for name, err, reach, margin, q7 in check(p, door, a.pelvis_z):
        ok = err < 0.01
        info = "脚は構えのまま" in name  # 腕だけでどこまで届くかの目安（届かなくてよい。脚が出る）
        bad += (not ok) and not info
        print(f"[reach] {'○' if ok else ('△' if info else '×')} {name:22s} 残り {100 * err:5.1f} cm・腕の伸び {reach:.2f} m・端まで {margin:.2f} rad・q {np.round(q7, 2).tolist()}")
    print(f"[reach] 届かない点: {bad}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
