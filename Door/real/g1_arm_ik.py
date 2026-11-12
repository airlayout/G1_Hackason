"""11-12 台本の計画（docs/plan/2026-11-12-g1-door-scripted-stage123.md・S1）: 腕 7 軸の逆運動学。numpy だけ。

手のひら（{side}_rubber_hand の原点）の位置を pelvis の座標の目標へ寄せる。向きは縛らず、余った自由度は基準の姿勢
（posture）へ寄せる。1 回の step は減衰つき最小二乗の 1 歩（前の解から追う形。50 Hz の輪で毎周期 1 回呼ぶ）。

  ik = ArmIK(chain, "left")
  q7 = ik.step(q29, q7_cmd, target_pelvis, posture7)   # q29 の腰・ほかの関節は実の値、腕は q7_cmd に差し替えて解く
  q7, err = ik.solve(q29, q7_start, target_pelvis, posture7)  # 机の上の確かめ用（収束するまで回す）

ヤコビアンは鎖をたどって解析的に出す（関節の軸 × (手のひら − 関節の位置)）。順運動学は g1_arm_fk.ArmFK と同じ URDF。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import List, Tuple

import numpy as np

import g1_arm_fk as fk

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
DAMPING = 0.05          # 減衰つき最小二乗の λ（特異な姿勢の近くで暴れない）
POSTURE_GAIN = 0.05     # 余った自由度で基準の姿勢へ寄せる強さ（1 歩で残りのこの割合）
MAX_STEP_RAD = 0.04     # 1 歩の関節の動きの上限（50 Hz で 2 rad/s）
MAX_ERR_M = 0.03        # 1 歩で追う手のひらの差の上限（遠い目標へ一気に飛ばない）
DIR_WEIGHT_M = 0.10     # 手の向き（手首 → 手のひらの単位ベクトル）の差 1 を位置の差何 m ぶんに数えるか（手の長さの目安）
LIMIT_MARGIN = 0.05     # 可動域の端から内側へ [rad]


def arm_indices(side: str) -> List[int]:
    """SDK の並び（0〜28）での腕 7 軸の番号"""
    if side not in ("left", "right"):
        raise ValueError(f"side は left か right: {side}")
    return [fk.JOINT_ORDER.index(f"{side}_{n}_joint") for n in ARM_JOINTS]


def joint_limits(urdf: str, side: str) -> Tuple[np.ndarray, np.ndarray]:
    """URDF の腕 7 軸の可動域（下・上）"""
    lim = {}
    for j in ET.parse(urdf).getroot().findall("joint"):
        li = j.find("limit")
        if li is not None and li.get("lower") is not None:
            lim[j.get("name")] = (float(li.get("lower")), float(li.get("upper")))
    names = [f"{side}_{n}_joint" for n in ARM_JOINTS]
    missing = [n for n in names if n not in lim]
    if missing:
        raise ValueError(f"URDF に可動域が無い: {missing}")
    return np.array([lim[n][0] for n in names]), np.array([lim[n][1] for n in names])


class ArmIK:
    def __init__(self, chain: fk.ArmFK, side: str = "left", urdf: "str | None" = None) -> None:
        self.chain, self.side = chain, side
        self.idx = arm_indices(side)
        self.link = f"{side}_rubber_hand"
        self.lo, self.hi = joint_limits(urdf or fk.default_urdf(), side)
        self.lo, self.hi = self.lo + LIMIT_MARGIN, self.hi - LIMIT_MARGIN

    def _full(self, q29: np.ndarray, q7: np.ndarray) -> np.ndarray:
        q = np.array(q29, dtype=float, copy=True)
        q[self.idx] = q7
        return q

    def palm(self, q29: np.ndarray, q7: np.ndarray) -> np.ndarray:
        """腕を q7 に差し替えた時の手のひら（pelvis の座標）"""
        return self.chain.link_pose(self.link, self._full(q29, q7))[0]

    def jacobian(self, q29: np.ndarray, q7: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """手のひらの位置と、腕 7 軸に対する位置のヤコビアン（3×7）。鎖を 1 回たどる"""
        q = self._full(q29, q7)
        links, link = [], self.link
        while link != "pelvis":
            links.append(self.chain.joints[link])
            link = self.chain.joints[link][1]
        R, p = np.eye(3), np.zeros(3)
        frames = {}
        for name, _, xyz, rot, axis, kind in reversed(links):
            p = p + R @ xyz
            R = R @ rot
            if kind in ("revolute", "continuous") and name in self.chain.index:
                frames[name] = (p.copy(), R @ (axis / np.linalg.norm(axis)))
                R = R @ fk._axis_angle(axis, float(q[self.chain.index[name]]))
        J = np.zeros((3, 7))
        for k, n in enumerate(ARM_JOINTS):
            origin, ax = frames[f"{self.side}_{n}_joint"]
            J[:, k] = np.cross(ax, p - origin)
        self._frames, self._p = frames, p
        return p, J

    def direction(self, q29: np.ndarray, q7: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """手の向き（手首 yaw の関節 → 手のひらの単位ベクトル。pelvis の座標）と、そのヤコビアン（3×7）"""
        p, _ = self.jacobian(q29, q7)
        frames = self._frames
        v = p - frames[f"{self.side}_wrist_yaw_joint"][0]
        u = v / float(np.linalg.norm(v))
        Ju = np.zeros((3, 7))
        for k, n in enumerate(ARM_JOINTS):
            Ju[:, k] = np.cross(frames[f"{self.side}_{n}_joint"][1], u)
        return u, Ju

    def step(self, q29: np.ndarray, q7: np.ndarray, target: np.ndarray, posture: np.ndarray,
             direction: "np.ndarray | None" = None) -> np.ndarray:
        """1 歩ぶん寄せた腕 7 軸（新しい配列）。目標か角度が有限でなければ q7 をそのまま返す。
        direction（pelvis の座標の単位ベクトル）を渡すと、手の向きもそこへ寄せる（11-12: 手の先を扉へ向けない）"""
        q7 = np.asarray(q7, dtype=float)
        target = np.asarray(target, dtype=float)
        if not (np.isfinite(q7).all() and np.isfinite(target).all() and np.isfinite(np.asarray(q29, dtype=float)).all()):
            return q7.copy()
        p, J = self.jacobian(q29, q7)
        err = target - p
        n = float(np.linalg.norm(err))
        if n > MAX_ERR_M:
            err = err * (MAX_ERR_M / n)
        if direction is not None and np.isfinite(np.asarray(direction, dtype=float)).all():
            u, Ju = self.direction(q29, q7)
            e_dir = DIR_WEIGHT_M * (np.asarray(direction, dtype=float) - u)
            nd = float(np.linalg.norm(e_dir))
            if nd > MAX_ERR_M:
                e_dir = e_dir * (MAX_ERR_M / nd)
            J, err = np.vstack([J, DIR_WEIGHT_M * Ju]), np.concatenate([err, e_dir])
        JJt = J @ J.T + (DAMPING ** 2) * np.eye(J.shape[0])
        pinv = J.T @ np.linalg.inv(JJt)
        dq = pinv @ err + (np.eye(7) - pinv @ J) @ (POSTURE_GAIN * (np.asarray(posture, dtype=float) - q7))
        big = float(np.abs(dq).max())
        if big > MAX_STEP_RAD:
            dq = dq * (MAX_STEP_RAD / big)
        return np.clip(q7 + dq, self.lo, self.hi)

    def solve(self, q29: np.ndarray, q7: np.ndarray, target: np.ndarray, posture: np.ndarray,
              iters: int = 400, tol: float = 1e-3) -> Tuple[np.ndarray, float]:
        """収束するまで step を回す（机の上の届きの確かめ用）。返すのは解と残りの差 [m]"""
        q7 = np.asarray(q7, dtype=float).copy()
        for _ in range(iters):
            if float(np.linalg.norm(np.asarray(target, dtype=float) - self.palm(q29, q7))) < tol:
                break
            q7 = self.step(q29, q7, target, posture)
        return q7, float(np.linalg.norm(np.asarray(target, dtype=float) - self.palm(q29, q7)))


def elbow_reach(chain: fk.ArmFK, q29: np.ndarray, side: str = "left") -> float:
    """肩から手のひらまでの距離 [m]（腕の伸び。伸び切りの目安）"""
    s = chain.link_pose(f"{side}_shoulder_roll_link", q29)[0]
    h = chain.link_pose(f"{side}_rubber_hand", q29)[0]
    return float(math.sqrt(float((h - s) @ (h - s))))
