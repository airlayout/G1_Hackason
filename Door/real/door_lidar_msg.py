"""段 5・L1（2026-10-23）: ライダーの推定器（door_lidar_node.py）から制御（door_run_real.py）へ渡す電文と、受け取った後の計算。numpy だけ。

電文（UDP・1 回の測定ごと・10 Hz）: 扉の座標でのライダー（頭）の床の上の姿勢と扉の角度。pelvis への直しは制御の側で行う
（腰の関節と pelvis の IMU は制御のプロセスが 1 kHz で持っているので、推定器に rt/lowstate を購読させない。10-16 は 1 kHz の
受け取りが Python を詰まらせて制御の周期を乱した）。

pelvis への直し（lidar_to_pelvis）:
  ライダーは torso_link の上（URDF の mid360_joint: 0.406 m 上）にあり、torso は腰の 3 関節（yaw・roll・pitch）で pelvis につながる。
  床の上で見た「pelvis から見たライダーの水平の位置」を、pelvis の傾き（IMU）と腰の関節から順運動学で出し、
  首の垂れなどの分（日によって違う）は取付の補正 head_xy（旋回で測る）として足す。向きは腰の yaw の分だけ pelvis と違う。
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

MAGIC = b"DLT1"
FMT = "<4sIdddddfHHff"  # magic・連番・測った時刻（PC2 の time.time()）・x・y・yaw・扉の角度[rad]・角度の点の数・状態・残差・傾き
SIZE = struct.calcsize(FMT)
STATE_OK = 1  # 重ね直しが成功した
STATE_ANGLE_OK = 2  # 扉の角度が測れた
PORT = 47321

# URDF（unitree の g1_29dof）: pelvis → waist_yaw（原点）→ waist_roll（−0.0039635, 0, 0.035）→ waist_pitch（0, 0, 0.019）→ torso_link
WAIST_ROLL_XYZ = np.array([-0.0039635, 0.0, 0.035])
WAIST_PITCH_XYZ = np.array([0.0, 0.0, 0.019])
MID360_XYZ = np.array([0.0002835, 0.00003, 0.40618])  # torso_link → mid360_link
WAIST_IDX = (12, 13, 14)  # SDK の関節の番号: waist_yaw・waist_roll・waist_pitch


@dataclass(frozen=True)
class LidarMsg:
    seq: int
    stamp: float  # [s] PC2 の時刻（点群を受け取った時）
    x: float  # 扉の座標でのライダーの床の上の位置 [m]
    y: float
    yaw: float  # [rad]
    angle: float  # 扉の角度 [rad]（nan なら測れない）
    angle_points: int
    state: int
    rms_m: float
    tilt_deg: float

    @property
    def pose_ok(self) -> bool:
        return bool(self.state & STATE_OK)

    @property
    def angle_ok(self) -> bool:
        return bool(self.state & STATE_ANGLE_OK) and math.isfinite(self.angle)


def pack(m: LidarMsg) -> bytes:
    return struct.pack(FMT, MAGIC, m.seq & 0xFFFFFFFF, m.stamp, m.x, m.y, m.yaw, m.angle, m.rms_m,
                       min(m.angle_points, 65535), m.state, m.tilt_deg, 0.0)


def unpack(b: bytes) -> Optional[LidarMsg]:
    if len(b) != SIZE:
        return None
    magic, seq, stamp, x, y, yaw, angle, rms, npts, state, tilt, _ = struct.unpack(FMT, b)
    if magic != MAGIC or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
        return None
    return LidarMsg(seq, stamp, x, y, yaw, angle, npts, state, rms, tilt)


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def lidar_in_pelvis(q_waist: Tuple[float, float, float]) -> np.ndarray:
    """pelvis の座標で見たライダーの位置（URDF の順運動学。首は固定とみなす）"""
    wy, wr, wp = q_waist
    return _rz(wy) @ (WAIST_ROLL_XYZ + _rx(wr) @ (WAIST_PITCH_XYZ + _ry(wp) @ MID360_XYZ))


def lidar_offset_xy(q_waist: Tuple[float, float, float], roll: float, pitch: float, head_xy: Tuple[float, float]) -> np.ndarray:
    """pelvis の向き（yaw）で見た、床の上の pelvis → ライダーの水平の位置。head_xy は首の垂れなどの補正（旋回で測る）"""
    p = (_ry(pitch) @ _rx(roll)) @ lidar_in_pelvis(q_waist)  # pelvis の傾きを入れて床の上へ
    return p[:2] + np.array(head_xy)


def lidar_to_pelvis(x: float, y: float, yaw: float, q_waist: Tuple[float, float, float], roll: float, pitch: float,
                    head_xy: Tuple[float, float]) -> Tuple[float, float, float]:
    """扉の座標でのライダーの床の上の姿勢 → pelvis の床の上の姿勢（x, y, yaw）"""
    yaw_p = math.atan2(math.sin(yaw - q_waist[0]), math.cos(yaw - q_waist[0]))  # ライダー（torso）は腰の yaw だけ pelvis と違う向き
    o = lidar_offset_xy(q_waist, roll, pitch, head_xy)
    c, s = math.cos(yaw_p), math.sin(yaw_p)
    return x - (c * o[0] - s * o[1]), y - (s * o[0] + c * o[1]), yaw_p


def head_xy_from_center_offset(center_xy: Tuple[float, float], q_waist: Tuple[float, float, float], roll: float,
                               pitch: float) -> Tuple[float, float]:
    """旋回で測った「pelvis（robot_center）から見たライダーの水平の位置」から、順運動学の分を引いた補正"""
    fk = lidar_offset_xy(q_waist, roll, pitch, (0.0, 0.0))
    return float(center_xy[0] - fk[0]), float(center_xy[1] - fk[1])
