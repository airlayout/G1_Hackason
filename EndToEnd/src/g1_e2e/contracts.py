"""物理単位・SDK関節順による共通契約。NNの特徴量や履歴はここに含めない。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float32]


def vector(value: object, size: int, name: str) -> FloatArray:
    """形状と有限値を検査し、呼び出し元から独立した配列を返す。"""
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name}: finite shape ({size},) required, got {result.shape}")
    return result.copy()


@dataclass(frozen=True)
class Observation:
    """1制御時刻の観測。関節はSDK順、姿勢はxyzw、身体座標はx前/y左/z上。

    joint_torque_nmはIsaacのapplied_torque。implicit PDでは推定値であり実測力覚ではない。
    """

    episode_id: int
    step: int
    simulation_time_s: float
    joint_position_rad: FloatArray
    joint_velocity_rad_s: FloatArray
    joint_torque_nm: FloatArray
    orientation_xyzw: FloatArray
    angular_velocity_body_rad_s: FloatArray
    projected_gravity_body: FloatArray
    # 真値はセンサー入力と分け、評価・criticなどで明示的に選択する。
    privileged: dict[str, FloatArray] = field(default_factory=dict)


@dataclass(frozen=True)
class JointCommand:
    """29関節のhybrid PD指令。正規化・相対角度への変換はモデル側の責務。"""

    position_rad: FloatArray
    velocity_rad_s: FloatArray = field(default_factory=lambda: np.zeros(29, dtype=np.float32))
    feedforward_torque_nm: FloatArray = field(default_factory=lambda: np.zeros(29, dtype=np.float32))

    def validated(self) -> JointCommand:
        """シミュレーションへ値を渡す直前に検証する。"""
        return JointCommand(
            vector(self.position_rad, 29, "position_rad"),
            vector(self.velocity_rad_s, 29, "velocity_rad_s"),
            vector(self.feedforward_torque_nm, 29, "feedforward_torque_nm"),
        )


@dataclass(frozen=True)
class VelocityCommand:
    """上位方策の速度指令。各3成分はbase座標vx m/s、vy m/s、yaw_rate rad/s。"""

    requested: FloatArray
    applied: FloatArray

    def validated(self):
        return VelocityCommand(
            vector(self.requested, 3, "requested velocity"), vector(self.applied, 3, "applied velocity")
        )


@dataclass(frozen=True)
class StepResult:
    """環境遷移。タスク固有の報酬は環境の外で計算する。"""

    observation: Observation
    requested_command: JointCommand
    applied_command: JointCommand
    terminated: bool
    truncated: bool
    info: dict[str, object]


class Controller(Protocol):
    """MLP・Transformer・ONNX・非NNでも同じ入口を実装する。"""

    def reset(self) -> None:
        """エピソード間の履歴・隠れ状態を消去する。"""
        ...

    def act(self, observation: Observation) -> JointCommand:
        """観測をモデル固有の入力へ変換し、物理単位の指令を返す。"""
        ...
