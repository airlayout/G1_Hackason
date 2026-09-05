"""Unitree公式G1記述を基準にしたMappingセンサrig。"""

from __future__ import annotations

import math

# unitreerobotics/unitree_ros の g1_29dof.urdf を基準にする。
# 実機では機体revisionの確認後、校正済みrig設定で置き換える。
REFERENCE_URDF = "unitreerobotics/unitree_ros:robots/g1_description/g1_29dof.urdf"

TORSO_TO_D435_XYZ = (0.0576235, 0.01753, 0.41987)
TORSO_TO_D435_RPY = (0.0, 0.8307767239493009, 0.0)
TORSO_TO_MID360_XYZ = (0.0002835, 0.00003, 0.40618)
TORSO_TO_MID360_RPY = (0.0, 0.04014257279586953, 0.0)


def rpy_to_quat_xyzw(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """URDF RPYからIsaacLab 3.xの(x,y,z,w) quaternionへ変換する。"""

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_to_quat_wxyz(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """旧RayCaster設定で使う(w,x,y,z) quaternionへ変換する。"""

    x, y, z, w = rpy_to_quat_xyzw(roll, pitch, yaw)
    return (w, x, y, z)
