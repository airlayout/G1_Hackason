"""任意コントローラーの接続例。基部固定で左肩を動かして関節対応を検証する。"""

from __future__ import annotations

import numpy as np

from g1_e2e.contracts import JointCommand, Observation
from g1_e2e.robot import RobotSpec


class ArmSweepController:
    """NNの場合もact内を推論処理へ差し替えれば同じ環境を使える。"""

    def __init__(self, robot: RobotSpec, options: dict) -> None:
        self.position = robot.default_position.copy()
        self.joint = robot.joint_names.index("left_shoulder_pitch_joint")
        self.amplitude = float(options.get("amplitude_rad", 0.2))

    def reset(self) -> None:
        """時刻はObservationから受け取るため隠れ状態は持たない。"""

    def act(self, observation: Observation) -> JointCommand:
        """環境の内部APIを使わず、物理単位の指令を返す。"""
        position = self.position.copy()
        position[self.joint] += self.amplitude * np.sin(np.pi * observation.simulation_time_s)
        return JointCommand(position)


def create_controller(*, robot: RobotSpec, control_dt: float, options: dict) -> ArmSweepController:
    """run.sh --controller examples.arm_sweep:create_controllerで接続する。"""
    return ArmSweepController(robot, options)
