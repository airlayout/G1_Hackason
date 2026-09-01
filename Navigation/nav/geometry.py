"""平面上の姿勢と、四元数 <-> yaw の変換をここだけに閉じ込める。

`slam_operate` は姿勢の表現が場所によって違う:

| 場所 | 表現 |
|---|---|
| 1804 / 1102 の入力 | 四元数 `q_x,q_y,q_z,q_w` |
| `pos_info` の `currentPose` | 四元数 |
| `ctrl_info` の `startPose` / `targetPose` / `currentPose` | `roll,pitch,yaw` |

混在したまま扱うと必ず取り違えるので、**平面のyawだけを持つ `Pose2D` を正**とし、
変換はこのモジュールの外で書かない。

G1のナビは室内平地が適用条件（`Navigation/README.md`「純正APIの制約」）で、
roll/pitchは制御対象ではない。z も 1102 に渡す値は「地図の高さ」であって
指示できる自由度ではないため、`Pose2D` は x/y/yaw のみを持つ。

座標系は Mid360-IMU 原点、X軸が機体正面、Z軸が鉛直上向き（公式「一、介绍」）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# 四元数の正規化で 0 除算を避けるための下限。
# これを下回る四元数は不正データとして扱う。
_QUATERNION_MIN_NORM = 1e-9


@dataclass(frozen=True)
class Pose2D:
    """平面上の姿勢。yawの単位はラジアン。

    frozen にしているのは、経路分割の途中で書き換わると
    「どの区間の目標だったか」が追えなくなるため。
    """

    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(other.x - self.x, other.y - self.y)

    def heading_to(self, other: "Pose2D") -> float:
        """自分から相手を向く方位[rad]。同一点なら自分のyawを返す。"""

        dx = other.x - self.x
        dy = other.y - self.y
        if math.hypot(dx, dy) < _QUATERNION_MIN_NORM:
            return self.yaw
        return math.atan2(dy, dx)

    def with_yaw(self, yaw: float) -> "Pose2D":
        return Pose2D(self.x, self.y, normalize_angle(yaw))


def normalize_angle(angle: float) -> float:
    """角度を (-pi, pi] に畳む。"""

    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """yaw[rad] -> (q_x, q_y, q_z, q_w)。Z軸まわりの回転のみ。"""

    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(q_x: float, q_y: float, q_z: float, q_w: float) -> float:
    """(q_x,q_y,q_z,q_w) -> yaw[rad]。roll/pitchは捨てる。

    正規化されていない四元数も受ける（実機のJSONを直接食わせるため）。
    ノルムが 0 に近い場合は姿勢を決められないので ValueError を投げる。
    """

    norm = math.sqrt(q_x * q_x + q_y * q_y + q_z * q_z + q_w * q_w)
    if norm < _QUATERNION_MIN_NORM:
        raise ValueError(f"四元数のノルムが0に近く姿勢を決められない: {(q_x, q_y, q_z, q_w)}")
    x, y, z, w = q_x / norm, q_y / norm, q_z / norm, q_w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def interpolate(start: Pose2D, end: Pose2D, ratio: float) -> Pose2D:
    """start->end を線分として内分する。yawは進行方向を向かせる。

    ratio は 0.0〜1.0 に丸める。区間分割で端を越える呼び方をしても
    線分の外に出た点を返さないため。
    """

    clamped = min(1.0, max(0.0, ratio))
    x = start.x + (end.x - start.x) * clamped
    y = start.y + (end.y - start.y) * clamped
    return Pose2D(x, y, start.heading_to(end))
