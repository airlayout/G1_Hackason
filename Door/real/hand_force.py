"""11-13: 腕の関節の力（tau_est）から、手が外（扉・レバー）から受けている力を見積もる。numpy だけ（PC2 の python3.8 で回る）。

  hf = HandForce(chain, urdf, "left", tip_m=0.10)   # tip_m: 力が掛かる点（手のひらの原点から手の先へ [m]。0 は手のひらの原点）
  hf.calibrate(q, quat, tau)        # 何にも触れていない間（台本の 2b）に毎周期呼ぶ。直近 0.5 秒の残りを基準にする
  F = hf.force(q, quat, tau)        # pelvis の座標の力 [N]（基準が無ければ NaN）。扉の座標へは呼ぶ側で回す

式（止まっている腕の釣り合い）: tau = g(q) − Jᵀ F  →  F = pinv(Jᵀ)(g(q) − tau − 残り)
  g(q) は URDF の質量と重心から出す、腕を今の姿勢で支えるのに要る関節の力。J は力が掛かる点のヤコビアン（3×7）。
  力が掛かる点は手のひらの原点から手首 → 手のひらの向きへ tip_m 先（11-13: 台本は手の先＝10 cm 先でレバーを押す。
  シムで手のひらの原点のまま解くと、押している間の上下を接触センサの 1.8 倍に出した）。
  残り（基準）は、触れていない時の g − tau。力の読みの基準のずれ（左腕は 10-29 に 5 N·m ずれていた）と模型の違いを引く。

⚠️ 見積もり: 腕の加速・関節の摩擦は見ていない（速く動かしている間は外れる）。当たる点は 1 点とみなす。
   11-13 の記録 3 本で MuJoCo の逆動力学と比べた値は ana_hand_force_1113.py。
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
GRAVITY = 9.81
CALIB_N = 25          # 基準に使う周期の数（50 Hz で 0.5 秒）
FILTER_ALPHA = 0.2    # 力のなまし（50 Hz で時定数 約 0.1 秒）
TIP_M = fk.HAND_TIP_M  # 手の先（手のひらの原点から 0.10 m）。台本の手の先で押す形の当たる点


def _inertials(urdf: str) -> dict:
    """部品の名前 → (質量 kg, 重心（部品の座標）)"""
    out = {}
    for link in ET.parse(urdf).getroot().findall("link"):
        ine = link.find("inertial")
        if ine is None or ine.find("mass") is None:
            continue
        o = ine.find("origin")
        com = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        out[link.get("name")] = (float(ine.find("mass").get("value")), com)
    return out


class HandForce:
    def __init__(self, chain: fk.ArmFK, urdf: str, side: str = "left", tip_m: float = 0.0) -> None:
        self.chain, self.side, self.tip_m = chain, side, float(tip_m)
        self.idx = [fk.JOINT_ORDER.index(f"{side}_{n}_joint") for n in ARM_JOINTS]
        ine = _inertials(urdf)
        # 肩 pitch の部品から先の、腕にぶら下がる部品（枝分かれも含む）
        root = f"{side}_shoulder_pitch_link"
        children = {}
        for child, (_, parent, *_r) in chain.joints.items():
            children.setdefault(parent, []).append(child)
        self.links: List[str] = []
        stack = [root]
        while stack:
            link = stack.pop()
            self.links.append(link)
            stack.extend(children.get(link, []))
        self.mass = {k: ine[k] for k in self.links if k in ine}
        if not self.mass:
            raise ValueError(f"URDF に腕の質量が無い: {urdf}")
        self.joint_child = {chain.joints[c][0]: c for c in self.links}  # 関節の名前 → 子の部品
        self.calib: Deque[np.ndarray] = deque(maxlen=CALIB_N)
        self.res0: Optional[np.ndarray] = None
        self.filt: Optional[np.ndarray] = None

    def _poses(self, q: np.ndarray) -> dict:
        return {k: self.chain.link_pose(k, q) for k in self.links}

    def gravity_and_jacobian(self, q: np.ndarray, quat_wxyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(腕を支えるのに要る関節の力 g(q) [N·m]・7 つ, 力が掛かる点のヤコビアン 3×7)。pelvis の座標"""
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat_wxyz)
        gvec = dfr.tilt_matrix(roll, pitch).T @ np.array([0.0, 0.0, -GRAVITY])  # pelvis の座標での重力の向き
        poses = self._poses(q)
        palm = poses[f"{self.side}_rubber_hand"][0]
        if self.tip_m:
            d = palm - poses[f"{self.side}_wrist_yaw_link"][0]
            palm = palm + self.tip_m * d / (float(np.linalg.norm(d)) + 1e-9)
        g, J = np.zeros(7), np.zeros((3, 7))
        for k, n in enumerate(ARM_JOINTS):
            child = self.joint_child[f"{self.side}_{n}_joint"]
            origin, R = poses[child]
            axis = self.chain.joints[child][4]
            ax = R @ (axis / np.linalg.norm(axis))  # 軸は自分の回りの回転で変わらないので、子の向きで回してよい
            J[:, k] = np.cross(ax, palm - origin)
            # この関節より先の部品の重さ
            tq, stack = 0.0, [child]
            while stack:
                link = stack.pop()
                if link in self.mass:
                    m, com = self.mass[link]
                    p, Rl = poses[link]
                    tq += float(ax @ np.cross(p + Rl @ com - origin, m * gvec))
                stack.extend(c for c in self.links if self.chain.joints[c][1] == link)
            g[k] = -tq
        return g, J

    def calibrate(self, q: np.ndarray, quat_wxyz: np.ndarray, tau: np.ndarray) -> None:
        """触れていない間に呼ぶ: 直近 CALIB_N 周期の g − tau の平均を基準にする"""
        if not (np.isfinite(q).all() and np.isfinite(tau).all()):
            return
        g, _ = self.gravity_and_jacobian(q, quat_wxyz)
        self.calib.append(g - np.asarray(tau, dtype=float)[self.idx])
        self.res0 = np.mean(self.calib, axis=0)

    def force(self, q: np.ndarray, quat_wxyz: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """手が外から受ける力（pelvis の座標・N・なまし済み）。基準が無い・値が壊れている時は NaN"""
        if self.res0 is None or not (np.isfinite(q).all() and np.isfinite(tau).all()):
            return np.full(3, np.nan)
        g, J = self.gravity_and_jacobian(q, quat_wxyz)
        F = np.linalg.lstsq(J.T, g - np.asarray(tau, dtype=float)[self.idx] - self.res0, rcond=None)[0]
        self.filt = F if self.filt is None else self.filt + FILTER_ALPHA * (F - self.filt)
        return self.filt.copy()

    def reset(self) -> None:
        self.calib.clear()
        self.res0, self.filt = None, None
