"""頭カメラの座標 → pelvis 座標の変換。タスク4。

座標の決まり:
- カメラ座標（光学座標）: RealSense と同じく x 右、y 下、z 前方（画像の奥）[m]
- d435_link: URDF の頭カメラのリンク。x 前方、y 左、z 上
- pelvis: IK と同じ基準（公式 URDF の根元リンク）。x 前方、y 左、z 上

変換の順番: カメラ座標 → d435_link（軸の並べ替え）→ torso_link（configs/robot.yaml の head_camera の
xyz / rpy）→ pelvis（腰 3 関節の FK。URDF + Pinocchio）→ 較正の補正（localize.yaml）。

⚠️ RealSense のカラーカメラの光学中心と、URDF の d435_link の原点が同じかは未確認
   （D435 のカラーセンサは本体の中心から横にずれている）。一定のずれは実機日の較正で補正する。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .config import resolve_repo_path
from .robot_model import JOINT_NAMES, NUM_MOTORS, WAIST_IDX

# 光学座標の軸を d435_link で表したもの（列が光学座標の x, y, z）
R_LINK_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF の RPY（x→y→z の固定軸回転）→ 回転行列。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class HeadCameraTransform:
    def __init__(self, robot_cfg: dict[str, Any], calibration_offset: np.ndarray | None = None) -> None:
        import pinocchio as pin

        self._pin = pin
        self.model = pin.buildModelFromUrdf(str(resolve_repo_path(robot_cfg["model"]["urdf_path"])))
        names = [self.model.names[i] for i in range(1, self.model.njoints)]
        if names != list(JOINT_NAMES):
            raise ValueError("URDF の関節の並びが motor 番号順と違う")
        self.data = self.model.createData()
        cam = robot_cfg["head_camera"]
        self.parent_frame = self.model.getFrameId(cam["parent_body"])
        self.r_parent_link = rpy_to_matrix(*cam["rpy"])
        self.p_parent_link = np.asarray(cam["xyz"], dtype=float)
        self.offset = np.zeros(3) if calibration_offset is None else np.asarray(calibration_offset, dtype=float)

    def pelvis_from_optical(self, q_waist: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """腰の角度（yaw, roll, pitch）での、光学座標 → pelvis 座標の回転 R と平行移動 p（p_pelvis = R x + p）。

        較正の補正（offset）は含まない（to_pelvis で足す）。
        """
        q = np.zeros(NUM_MOTORS)
        q[list(WAIST_IDX)] = q_waist
        self._pin.framesForwardKinematics(self.model, self.data, q)
        t = self.data.oMf[self.parent_frame]
        r_link = t.rotation @ self.r_parent_link
        p_link = t.rotation @ self.p_parent_link + t.translation
        return r_link @ R_LINK_OPTICAL, p_link

    def to_pelvis(self, p_optical: np.ndarray, q_waist: np.ndarray) -> np.ndarray:
        """光学座標の点 → pelvis 座標の点（較正の補正込み）。"""
        r, p = self.pelvis_from_optical(q_waist)
        return r @ np.asarray(p_optical, dtype=float) + p + self.offset
