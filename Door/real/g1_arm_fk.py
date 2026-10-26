"""G1（29 関節）の手のひらの位置を、関節の角度から順運動学で出す（2026-10-23、腕だけの空振りの解析）。numpy だけ。

関節のつながりは unitree の URDF（g1_29dof）から読む: pelvis → 腰 yaw・roll・pitch → torso_link → 肩 pitch・roll・yaw → 肘 →
手首 roll・pitch・yaw → 手のひら（left/right_hand_palm_joint の固定の先 = rubber_hand）。関節の番号は SDK の順（腰 12〜14・左腕 15〜21・右腕 22〜28）。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

def _default_urdf() -> Path:
    """このファイルの隣（g1_29dof.urdf）か、Mac の作業場所の G1_Hackason_Entame にある unitree の URDF"""
    here = Path(__file__).resolve().parent
    if (here / "g1_29dof.urdf").exists():
        return here / "g1_29dof.urdf"
    for up in here.parents:
        c = up / "G1_Hackason_Entame" / "Entame" / "real" / "integrated_demo" / "motiondecode-test" / "external" / "g1_official_29dof.urdf"
        if c.exists():
            return c
    raise FileNotFoundError("g1 の URDF が見つからない（g1_arm_fk.py の隣に g1_29dof.urdf を置く）")
SIDE_CHAIN = {
    side: ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
           f"{side}_shoulder_pitch_joint", f"{side}_shoulder_roll_joint", f"{side}_shoulder_yaw_joint", f"{side}_elbow_joint",
           f"{side}_wrist_roll_joint", f"{side}_wrist_pitch_joint", f"{side}_wrist_yaw_joint", f"{side}_hand_palm_joint"]
    for side in ("left", "right")
}
SDK_INDEX = {"waist_yaw_joint": 12, "waist_roll_joint": 13, "waist_pitch_joint": 14}
for _k, _n in enumerate(("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")):
    SDK_INDEX[f"left_{_n}_joint"] = 15 + _k
    SDK_INDEX[f"right_{_n}_joint"] = 22 + _k


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx  # URDF の rpy は固定軸 X→Y→Z の順


def axis_angle(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def load_chain(urdf: "Path | None" = None) -> Dict[str, List[Tuple[str, np.ndarray, np.ndarray, np.ndarray]]]:
    """側ごとに [(関節名, 原点の回転, 原点の位置, 軸)]"""
    root = ET.parse(urdf or _default_urdf()).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    out = {}
    for side, names in SIDE_CHAIN.items():
        chain = []
        for n in names:
            j = joints[n]
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
            chain.append((n, rpy_matrix(*rpy), xyz, axis))
        out[side] = chain
    return out


def palm_in_pelvis(chain, q: np.ndarray, side: str) -> np.ndarray:
    """関節の角度 q（SDK の順・29）→ pelvis の座標での手のひらの位置"""
    R, p = np.eye(3), np.zeros(3)
    for name, R0, xyz, axis in chain[side]:
        p = p + R @ xyz
        R = R @ R0
        if name in SDK_INDEX:
            R = R @ axis_angle(axis, float(q[SDK_INDEX[name]]))
    return p
