"""入力（USBボタン）と出力（ロボット）の差し替え口。"""

from __future__ import annotations

import abc
import math

from .safe_stop import Command, Observation


class ButtonSource(abc.ABC):
    @abc.abstractmethod
    def is_ok(self) -> bool:
        """True=ボタンは解除状態で配線も健全。**読めない・不明は False**（フェイルセーフ）。"""


class SerialLoopSource(ButtonSource):
    """USB-シリアル変換器で NC 接点をループ検出する。

    配線: 変換器の DTR 出力 → NC接点(ZB2-BE102 の NC 端子) → CTS 入力。
    押下・断線・USB抜けのどれでも CTS が落ちる/例外になり、停止側に倒れる。
    DTR/CTS の極性と無接続時の読みは変換器で違うので、実機で押下前後を必ず測ること。
    """

    def __init__(self, port: str) -> None:
        import serial  # 関数内 import: pyserial の無い Mac でもモジュールを読めるように

        self._ser = serial.Serial(port)
        self._ser.dtr = True

    def is_ok(self) -> bool:
        try:
            return bool(self._ser.cts)
        except (OSError, ValueError):
            return False


class RobotPort(abc.ABC):
    @abc.abstractmethod
    def observe(self) -> Observation: ...

    @abc.abstractmethod
    def apply(self, cmd: Command) -> None:
        """毎周期呼ばれる。実装側で冪等にする。"""


class G1Port(RobotPort):
    """実機 G1（unitree_sdk2py）。**実機未検証**。腕・ハンドの保持は未実装。

    歩行の停止(StopMove)とダンピング(Damp)、IMU 由来の傾き・角速度のみ。
    腕・ハンドは上位層が `SafeStop.permits_motion` を見て指令を止めること。
    """

    def __init__(self, iface: str) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(0, iface)
        self._loco = LocoClient()
        self._loco.SetTimeout(1.0)
        self._loco.Init()
        self._low = None
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._on_low, 10)

    def _on_low(self, msg) -> None:
        self._low = msg

    def observe(self) -> Observation:
        m = self._low
        if m is None:
            return Observation(None, None, None)
        roll, pitch, _ = m.imu_state.rpy
        cos_tilt = max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))
        gx, gy, gz = m.imu_state.gyroscope
        return Observation(math.degrees(math.acos(cos_tilt)), None, math.sqrt(gx * gx + gy * gy + gz * gz))

    def apply(self, cmd: Command) -> None:
        if cmd.locomotion == "stop_move":
            self._loco.StopMove()
        elif cmd.locomotion == "damp":
            self._loco.Damp()
