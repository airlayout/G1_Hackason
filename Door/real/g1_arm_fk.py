"""11-09: G1 の URDF（学習の USD と同じ g1_29dof_rev_1_0.urdf）で、関節 29 の角度から腕の部品の位置を pelvis の座標で出す（numpy だけ）。

  fk = ArmFK(urdf_path)
  pts = fk.points(q29)   # {"L肩": xyz, "L肘": xyz, "L手": xyz, "L指先": xyz, "R…": …}（pelvis の座標 [m]）

関節の並びは SDK の 0〜28（脚 12・腰 3・左腕 7・右腕 7）。指先は手のひらの原点から、手首→手のひらの向きへ HAND_TIP_M 先とした（目安）。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

import numpy as np

JOINT_ORDER = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
HAND_TIP_M = 0.10  # 手のひらの原点から指先までの目安
LINKS = {"肩": "shoulder_roll_link", "肘": "elbow_link", "手首": "wrist_yaw_link", "手": "rubber_hand"}


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(ax: np.ndarray, a: float) -> np.ndarray:
    ax = ax / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


class ArmFK:
    def __init__(self, urdf: str) -> None:
        root = ET.parse(urdf).getroot()
        self.joints = {}  # 子の部品 → (関節名, 親, 位置, 回転, 軸, 種類)
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
            self.joints[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"), xyz, _rpy(*rpy), axis, j.get("type"))
        self.index = {n: i for i, n in enumerate(JOINT_ORDER)}

    def link_pose(self, link: str, q: np.ndarray):
        """pelvis から見た部品の位置と向き"""
        chain = []
        while link != "pelvis":
            chain.append(self.joints[link])
            link = self.joints[link][1]
        R, p = np.eye(3), np.zeros(3)
        for name, _, xyz, rot, axis, kind in reversed(chain):
            p = p + R @ xyz
            R = R @ rot
            if kind in ("revolute", "continuous") and name in self.index:
                R = R @ _axis_angle(axis, float(q[self.index[name]]))
        return p, R

    def points(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        out = {}
        for side, pre in (("L", "left_"), ("R", "right_")):
            for jp, suffix in LINKS.items():
                out[side + jp] = self.link_pose(pre + suffix, q)[0]
            d = out[side + "手"] - out[side + "手首"]
            out[side + "指先"] = out[side + "手"] + HAND_TIP_M * d / (np.linalg.norm(d) + 1e-9)
        return out


def default_urdf() -> str:
    return str(Path(__file__).resolve().parents[1] / "doorman6/usd_fix/urdf_src/g1_29dof_rev_1_0.urdf")


# 10-23 の口（hand_reach_dry.py が使う）。11-09 に ArmFK へ書き換えたとき消えていたので、ArmFK の上に戻した
def load_chain(urdf: "str | Path | None" = None) -> ArmFK:
    """順運動学の入れ物を作る（URDF を省くと default_urdf）"""
    return ArmFK(str(urdf) if urdf else default_urdf())


def palm_in_pelvis(chain: ArmFK, q: np.ndarray, side: str) -> np.ndarray:
    """関節の角度 q（SDK の順・29）→ pelvis の座標での手のひら（{side}_hand_palm_joint の先の rubber_hand の原点）"""
    return chain.link_pose(f"{side}_rubber_hand", q)[0]
