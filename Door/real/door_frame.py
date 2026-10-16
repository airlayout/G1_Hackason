"""段 5: 実機の機体の姿勢から、生徒が見る扉の値（door_frame_loc・door_geometry・complete）を作る。numpy だけ。

扉の座標（シミュレータと同じ。spec_actor.md §2.1）:
  原点 = 開口の中心の床、+x = 押して入る向き（廊下 → 部屋）、+y = +x を向いたときの左、+z = 上
  閉じた扉の板の中央の面が x = 0。把手が左（door_open_lr = +1、UIS Room 4）は、レバーが y = +(幅/2 − バックセット)、蝶番が y = −幅/2

door_frame_loc（9）= pelvis の座標で見た [開口の中心の位置（z は pelvis の高さに置き換え）, 扉の +x 軸, 扉の +z 軸]
  pos = R^T·(p_door' − p_pelvis)、tan = R^T·e_x、norm = R^T·e_z（R は扉の座標での pelvis の向き。傾きも含む）
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlanarPose:
    """扉の座標での pelvis の床の上の位置と向き"""

    x: float  # [m] 開口の中心から +x 方向（扉の手前は負）
    y: float  # [m]
    yaw: float  # [rad] 扉の +x を向いて 0、左回りが正


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def roll_pitch_from_quat_wxyz(q: np.ndarray):
    """機体 → 世界の四元数（w, x, y, z）から roll・pitch（ZYX の順の角度）"""
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def yaw_from_quat_wxyz(q: np.ndarray) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_matrix(roll: float, pitch: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return ry @ rx


def door_frame_loc(pose: PlanarPose, imu_quat_wxyz: np.ndarray) -> np.ndarray:
    """pelvis から見た扉の枠の位置と向き（9）。向きの yaw は測位（pose）、roll・pitch は IMU から取る"""
    roll, pitch = roll_pitch_from_quat_wxyz(imu_quat_wxyz)
    R = rot_z(pose.yaw) @ tilt_matrix(roll, pitch)  # 扉の座標での pelvis の向き
    offset = np.array([-pose.x, -pose.y, 0.0])  # 開口の中心（z は pelvis の高さ）− pelvis
    return np.concatenate([R.T @ offset, R.T @ np.array([1.0, 0.0, 0.0]), R.T @ np.array([0.0, 0.0, 1.0])])


def door_geometry(width: float, height: float, lever_height: float, backset: float, lever_left: bool = True) -> np.ndarray:
    """[幅, 高さ, レバーの高さ, バックセット, lr, 1 − lr, io]（実の向き。io は学習で常に 0 だったので 0）"""
    lr = 1.0 if lever_left else -1.0
    return np.array([width, height, lever_height, backset, lr, 1.0 - lr, 0.0])


def compose_planar(start: PlanarPose, odom_start: PlanarPose, odom_now: PlanarPose) -> PlanarPose:
    """巻き尺で測った最初の姿勢に、odom の始めからの動き（odom の始めの向きで見た差）を足す"""
    dx, dy = odom_now.x - odom_start.x, odom_now.y - odom_start.y
    c, s = math.cos(-odom_start.yaw), math.sin(-odom_start.yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy  # odom の始めの機体の向きで見た移動
    c2, s2 = math.cos(start.yaw), math.sin(start.yaw)
    dyaw = math.atan2(math.sin(odom_now.yaw - odom_start.yaw), math.cos(odom_now.yaw - odom_start.yaw))
    return PlanarPose(start.x + c2 * lx - s2 * ly, start.y + s2 * lx + c2 * ly, start.yaw + dyaw)


def door_closed_angle_obs() -> np.ndarray:
    """扉が閉じていると分かっているときの door_angle_lidar（シミュレータの初めの値と同じ形: 角度 0・測れた・古さ 0.1 s）"""
    return np.array([0.0, 1.0, 0.1])
