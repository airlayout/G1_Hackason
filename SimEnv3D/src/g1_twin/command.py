"""速度コマンドの定義と、送信先を差し替えるための抽象層。

デジタルツイン上の G1 と実機 G1 の双方に同じコマンドを流せるようにするため、
「コマンドの表現」と「送信先」を分離している。

送信先の実装:
- SimCommandSink   : シミュレーション内のポリシーへ渡す（現在利用可能）
- DdsCommandSink   : unitree_sdk2py 経由で実機へ（未実装・スタブ）
- Ros2CommandSink  : ROS2 Twist として発行（未実装・スタブ）
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


# 学習済みポリシー (Isaac-Velocity-Flat-G1-v0) の学習時コマンド範囲。
# IsaacLab の flat_env_cfg.py の設定値に対応する:
#     lin_vel_x = (0.0, 1.0)   <- 前進のみ。後退は学習されていない
#     lin_vel_y = (-0.5, 0.5)
#     ang_vel_z = (-1.0, 1.0)
#
# 後退について: 学習範囲が 0.0 始まりのため後退は未学習だが、実測では
# -0.2 m/s までは汎化で歩ける。-0.3 m/s 以上で転倒することを確認済み。
# そのため安全側の -0.2 を下限とする。
LIN_VEL_X_RANGE: tuple[float, float] = (-0.2, 1.0)
LIN_VEL_Y_RANGE: tuple[float, float] = (-0.5, 0.5)
ANG_VEL_Z_RANGE: tuple[float, float] = (-1.0, 1.0)


def _clamp(value: float, limits: tuple[float, float]) -> float:
    """値を範囲内に収める。"""
    low, high = limits
    return max(low, min(high, value))


@dataclass(frozen=True)
class VelocityCommand:
    """G1 への上位速度コマンド。

    Attributes:
        vx: 前後方向の速度 [m/s]（前が正）
        vy: 左右方向の速度 [m/s]（左が正）
        yaw_rate: 旋回角速度 [rad/s]（反時計回りが正）
    """

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    def clamped(self) -> "VelocityCommand":
        """ポリシーの学習範囲内に収めた新しいコマンドを返す。"""
        return VelocityCommand(
            vx=_clamp(self.vx, LIN_VEL_X_RANGE),
            vy=_clamp(self.vy, LIN_VEL_Y_RANGE),
            yaw_rate=_clamp(self.yaw_rate, ANG_VEL_Z_RANGE),
        )

    def is_zero(self) -> bool:
        """停止コマンドかどうか。"""
        return self.vx == 0.0 and self.vy == 0.0 and self.yaw_rate == 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        """(vx, vy, yaw_rate) のタプルとして返す。"""
        return (self.vx, self.vy, self.yaw_rate)


class CommandSink(ABC):
    """速度コマンドの送信先を表す抽象基底クラス。

    シミュレーションと実機を同じインターフェースで扱うために用いる。
    """

    @abstractmethod
    def send(self, command: VelocityCommand) -> None:
        """コマンドを送信する。"""

    def close(self) -> None:
        """リソースを解放する。既定では何もしない。"""
        return None


class SimCommandSink(CommandSink):
    """シミュレーション内のポリシーへコマンドを渡す送信先。

    直近のコマンドを保持するだけの薄い実装。
    実行ループ側が `latest` を読み取ってポリシーの観測に載せる。
    """

    def __init__(self) -> None:
        self._latest: VelocityCommand = VelocityCommand()

    def send(self, command: VelocityCommand) -> None:
        """コマンドを保持する。範囲外の値はクランプされる。"""
        self._latest = command.clamped()

    @property
    def latest(self) -> VelocityCommand:
        """直近に受け取ったコマンド。"""
        return self._latest


class DdsCommandSink(CommandSink):
    """unitree_sdk2py (DDS) 経由で実機 G1 へ送るための送信先。

    現時点では実機に接続できないため未実装。実機接続時にここを埋める。
    `unitree_sdk2py` は未インストールなので、導入も併せて必要になる。
    """

    def __init__(self, network_interface: str) -> None:
        self._network_interface = network_interface
        raise NotImplementedError(
            "[G1] DDS 送信は未実装です。実機接続時に unitree_sdk2py を導入して実装してください。"
        )

    def send(self, command: VelocityCommand) -> None:
        raise NotImplementedError


class Ros2CommandSink(CommandSink):
    """ROS2 の Twist メッセージとして発行する送信先。

    現時点では未実装。ROS2 連携が必要になった時点で実装する。
    """

    def __init__(self, topic: str = "/cmd_vel") -> None:
        self._topic = topic
        raise NotImplementedError(
            "[G1] ROS2 送信は未実装です。ROS2 連携が必要になった時点で実装してください。"
        )

    def send(self, command: VelocityCommand) -> None:
        raise NotImplementedError
