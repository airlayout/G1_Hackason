"""腕の指令の送り先（バックエンド）の共通インターフェース。

- SimBackend（backend_sim.py）: MuJoCo。胴体固定で、arm_sdk の weight のブレンドを近似する
- DdsBackend（backend_dds.py）: 実機。rt/arm_sdk（プランA）または rt/lowcmd（プランB）へ DDS で送る
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from .types import JointCommand, JointState


class ArmBackend(ABC):
    #: ログ表示用の名前
    name: str = "base"
    #: weight（arm_sdk の制御権）を上げ下げする経路か。lowcmd は False
    uses_weight: bool = True
    #: True なら送信しない（計算して表示するだけ）
    dry_run: bool = False

    @abstractmethod
    def open(self) -> None:
        """接続する（DDS の初期化、MuJoCo モデルの読み込みなど）。"""

    @abstractmethod
    def read_state(self) -> JointState | None:
        """最新の関節状態。まだ 1 回も受信していなければ None。"""

    @abstractmethod
    def send(self, cmd: JointCommand) -> None:
        """指令を 1 回送る。dry_run なら送らない。"""

    @abstractmethod
    def tick(self, dt: float) -> None:
        """次の制御周期まで進める（実機は待つ、シミュレーションは dt だけ物理を進める）。"""

    @abstractmethod
    def close(self) -> None:
        """後片付け。"""
