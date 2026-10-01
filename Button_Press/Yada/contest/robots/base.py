"""ロボットの共通の形。ランナー（contest/runner.py）はこれだけを使うので、シミュレーターと実機を差し替えられる。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from ..interface import Observation


@dataclass
class StepInfo:
    """1 周期ごとの、採点と記録のための情報（エージェントには渡さない）。"""

    lit: dict[str, bool] = field(default_factory=dict)  # ボタンごとの点灯（実機では人や画像で判定する）
    # ロボットと乗り場（壁・扉・盤）の接触力の最大 [N]（ボタンを押す力は含まない）。測れなければ None
    max_contact_force: float | None = None


class Robot(ABC):
    """ランナーから見たロボット。"""

    name: str  # "mujoco" / "isaac" / "real"
    base_enabled: bool  # 下半身の速度の指令が効くか

    @abstractmethod
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """全 29 関節の可動範囲 (下限, 上限) [rad]。"""

    @abstractmethod
    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        """上半身 17 関節の (kp, kd)。"""

    @abstractmethod
    def observe(self, t: float) -> Observation:
        """今の観測。t は試行の開始からの時間。"""

    @abstractmethod
    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        """上半身の目標の角度（安全のための処理のあと）と、下半身の速度の指令を送る。"""

    @abstractmethod
    def advance(self, dt: float) -> StepInfo:
        """dt 秒進める（シミュレーションは物理を進める。実機は周期を待つ）。"""

    def close(self) -> None:
        pass
