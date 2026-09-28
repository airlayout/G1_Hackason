"""実機のバックエンド。unitree_sdk2py で DDS に直接送る（ラボ PC から有線で接続する前提）。

- プランA: rt/arm_sdk。内蔵の歩行コントローラが動いている状態で、腕だけを weight でブレンドして指令する。
  weight は motor_cmd[29].q（0.0〜1.0）。**デバッグモードには入らない**（受け手がいなくなる）。
- プランB: rt/lowcmd。デバッグモード（内蔵コントローラ停止）で全関節に指令する。
  腕以外（脚・腰）は開始時の姿勢を保持する指令を送り続ける。**座った状態か吊り下げで使う。**

参考: unitree_sdk2_python の example/g1/high_level/g1_arm7_sdk_dds_example.py（arm_sdk）と
example/g1/low_level/g1_low_level_example.py（lowcmd）。

落とし穴（g1-starter-kit の記録より）:
- ゼロトルク（FSM 0）状態では motor_state[].mode が 0 で、送信は成功するのに何も動かない。
  リモコンでダンピング（FSM 1）に入れて有効化する。→ ArmCommander.start() で確認して拒否する
- 指令の mode_machine は lowstate の値と一致させる。→ 受信した値をそのまま使う
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from ..robot_model import ARM_SDK_WEIGHT_IDX, NUM_MOTORS
from .backend import ArmBackend
from .types import JointCommand, JointState

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"


def active_joints(cmd: JointCommand) -> list[int]:
    """指令を書く関節（kp か kd が 0 でない関節）の motor 番号。"""
    return [i for i in range(NUM_MOTORS) if cmd.kp[i] > 0 or cmd.kd[i] > 0]


def fill_lowcmd(msg: Any, cmd: JointCommand, mode_machine: int, use_weight: bool) -> None:
    """unitree_hg の LowCmd_ に指令を書き込む（CRC は呼び出し側で付ける）。

    kp か kd が 0 でない関節だけに mode=1 と q/kp/kd/tau を書く。それ以外は触らない（既定値 0 = 指令なし）。
    """
    msg.mode_pr = 0  # PR モード（足首を pitch / roll で指令する。公式サンプルと同じ）
    msg.mode_machine = int(mode_machine)
    for i in active_joints(cmd):
        c = msg.motor_cmd[i]
        c.mode = 1
        c.q = float(cmd.q[i])
        c.dq = 0.0
        c.kp = float(cmd.kp[i])
        c.kd = float(cmd.kd[i])
        c.tau = float(cmd.tau[i])
    if use_weight:
        msg.motor_cmd[ARM_SDK_WEIGHT_IDX].q = float(np.clip(cmd.weight, 0.0, 1.0))


class DdsBackend(ArmBackend):
    def __init__(
        self,
        path: str,
        network_interface: str,
        domain_id: int = 0,
        dry_run: bool = True,
    ) -> None:
        if path not in ("arm_sdk", "lowcmd"):
            raise ValueError(f"path は arm_sdk か lowcmd: {path}")
        self.path = path
        self.uses_weight = path == "arm_sdk"
        self.topic = TOPIC_ARM_SDK if self.uses_weight else TOPIC_LOWCMD
        self.name = f"{self.topic}{'（dry-run）' if dry_run else ''}"
        self.dry_run = dry_run
        self._iface = network_interface
        self._domain_id = domain_id
        self._lock = threading.Lock()
        self._state: JointState | None = None
        self._next_deadline: float | None = None
        self._send_count = 0

    def open(self) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        if not self._iface:
            raise ValueError(
                "configs/arm.yaml の network_interface が空。G1 につないでいる有線 NIC の名前"
                "（`ip -br a` で確認）を設定すること"
            )
        print(f"[dds] DDS 初期化（domain={self._domain_id}, NIC={self._iface}）")
        ChannelFactoryInitialize(self._domain_id, self._iface)
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._on_state, 10)
        self._pub = None
        if not self.dry_run:
            self._pub = ChannelPublisher(self.topic, LowCmd_)
            self._pub.Init()
        self._crc = CRC()
        self._msg = unitree_hg_msg_dds__LowCmd_()
        print(f"[dds] 送信先 {self.topic}（{'送信しない: dry-run' if self.dry_run else '送信する'}）")

    def _on_state(self, msg: Any) -> None:
        q = np.array([msg.motor_state[i].q for i in range(NUM_MOTORS)])
        dq = np.array([msg.motor_state[i].dq for i in range(NUM_MOTORS)])
        mode = np.array([msg.motor_state[i].mode for i in range(NUM_MOTORS)], dtype=int)
        st = JointState(q=q, dq=dq, motor_mode=mode, mode_machine=int(msg.mode_machine), stamp=time.monotonic())
        with self._lock:
            self._state = st

    def read_state(self) -> JointState | None:
        with self._lock:
            return self._state

    def send(self, cmd: JointCommand) -> None:
        st = self.read_state()
        if st is None:
            raise RuntimeError("lowstate を受信していないので送信できない（mode_machine が分からない）")
        fill_lowcmd(self._msg, cmd, st.mode_machine, self.uses_weight)
        self._msg.crc = self._crc.Crc(self._msg)
        self._send_count += 1
        if self._pub is not None:
            self._pub.Write(self._msg)

    def tick(self, dt: float) -> None:
        # 周期がずれていかないよう、前回の締め切りから dt ずつ進める
        now = time.monotonic()
        if self._next_deadline is None or now - self._next_deadline > 0.5:
            self._next_deadline = now
        self._next_deadline += dt
        rest = self._next_deadline - time.monotonic()
        if rest > 0:
            time.sleep(rest)

    def close(self) -> None:
        print(f"[dds] 送信回数 {self._send_count}")
