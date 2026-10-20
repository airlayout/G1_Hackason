"""NNなしで指令経路を調べるための基準姿勢コントローラー。"""

from __future__ import annotations

from g1_e2e.contracts import JointCommand, Observation
from g1_e2e.robot import RobotSpec


class HoldController:
    """固定姿勢を返す。浮遊基部でのバランス制御は行わない。"""

    def __init__(self, robot: RobotSpec) -> None:
        self.position = robot.default_position.copy()

    def reset(self) -> None:
        """履歴を持たない。"""

    def act(self, observation: Observation) -> JointCommand:
        """SDK順の位置指令を返す。"""
        return JointCommand(self.position.copy())


def create_controller(*, robot: RobotSpec, control_dt: float, options: dict) -> HoldController:
    """外部モデルと同じfactory形式。"""
    return HoldController(robot)
