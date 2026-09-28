"""腕の重力補償トルクの計算。

PD 制御だけだと、腕の重さの分だけ目標から下がる（MuJoCo で Kp=60 のとき 1.7〜2.5°）。
各関節で腕の重さを支えるのに要るトルク（重力トルク）を計算し、指令の tau（フィードフォワード
トルク）として送ると、この下がりが減る。

計算には公式モデル（g1_29dof_rev_1_0.xml、IK 用 URDF と同じ版）を MuJoCo で使う。
pelvis を固定した模型に、pelvis から見た重力の向きを与えて、関節角 q での重力トルクを求める。
pelvis から見た重力の向きは、lowstate の IMU の姿勢（四元数）から求める
（座った状態などで pelvis が傾いていても正しく計算するため。ヨー角は結果に影響しない）。

⚠️ lowstate の imu_state が pelvis の IMU であることは未確認（G1 は胴体側にも IMU がある）。
   ほぼ直立なら差は小さい。実機日に倍率 0 → 0.5 → 1.0 で腕の下がり方を比べて確かめる。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..robot_model import JOINT_NAMES, load_model

GRAVITY = 9.81


def quat_wxyz_to_mat(q: np.ndarray) -> np.ndarray:
    """四元数 (w, x, y, z) → 回転行列。"""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class GravityModel:
    def __init__(self, robot_cfg: dict[str, Any]) -> None:
        import mujoco

        self._mj = mujoco
        self.model = load_model(robot_cfg, fixed_base=True)
        self.data = mujoco.MjData(self.model)
        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._vadr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])

    def torques(self, q: np.ndarray, imu_quat_wxyz: np.ndarray | None = None) -> np.ndarray:
        """関節角 q（29、motor 番号順）で、重力を支えるのに要る各関節のトルク [Nm]。"""
        r = np.eye(3) if imu_quat_wxyz is None else quat_wxyz_to_mat(imu_quat_wxyz)
        # ワールドの重力 (0, 0, −g) を pelvis 座標に直す
        self.model.opt.gravity[:] = r.T @ np.array([0.0, 0.0, -GRAVITY])
        d = self.data
        d.qpos[self._qadr] = q
        d.qvel[:] = 0.0
        self._mj.mj_forward(self.model, d)
        # 速度 0 のとき qfrc_bias は重力による力。これと同じ大きさのトルクで支える
        return d.qfrc_bias[self._vadr].copy()
