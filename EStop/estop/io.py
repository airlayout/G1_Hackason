"""入力（USBボタン）と出力（ロボット）の差し替え口。"""

from __future__ import annotations

import abc
import math
import time

from .safe_stop import Command, Observation


class ButtonSource(abc.ABC):
    @abc.abstractmethod
    def is_ok(self) -> bool:
        """True=ボタンは解除状態で配線も健全。**読めない・不明は False**（フェイルセーフ）。"""


SEND_PERIOD_S = 0.05   # 確認バイトを送る周期
ECHO_TIMEOUT_S = 0.2   # この時間、正しい返りが無ければ停止
BAUD = 9600
_NONCE_MAX = 0x7F      # 0x01〜0x7F を順に使う。0x00/0xFF は浮いた線の誤読と区別するため避ける


class SerialEchoSource(ButtonSource):
    """USB-シリアル変換器の TXD → NC接点 → RXD で、送った 1 バイトが戻るかを見る。

    配線: 変換器の TXD ── NC接点(ZB2-BE102) ── RXD。5V ピンは使わない。
    接点が閉じている間だけ送信バイトが RXD に戻る。押下・断線・USB 抜け・変換器の故障は
    返りが途絶えて停止側に倒れる。毎回違う値を使うので、RXD が HIGH/LOW に固着しても
    「正常」と誤読しない。電圧の極性やプルアップに依存しない。
    """

    def __init__(self, port: str, ser=None, clock=time.monotonic) -> None:
        if ser is None:
            import serial  # 関数内 import: pyserial の無い Mac でもモジュールを読めるように

            ser = serial.Serial(port, BAUD, timeout=0)
        self._ser = ser
        self._clock = clock
        self._nonce = 0
        self._t_sent = float("-inf")
        self._t_echo = float("-inf")

    def is_ok(self) -> bool:
        now = self._clock()
        try:
            for b in self._ser.read(self._ser.in_waiting or 0):
                if b == self._nonce:
                    self._t_echo = now
            if now - self._t_sent >= SEND_PERIOD_S:
                self._nonce = self._nonce % _NONCE_MAX + 1
                self._ser.write(bytes([self._nonce]))
                self._t_sent = now
        except (OSError, ValueError):
            return False
        return now - self._t_echo <= ECHO_TIMEOUT_S


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
