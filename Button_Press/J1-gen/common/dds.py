"""DDS（unitree_sdk2py）の初期化と、lowstate の受信。実機用。

ChannelFactoryInitialize は 1 つのプロセスで 1 回しか呼べないので、ここで 1 回だけ呼ぶようにする
（腕の指令部分 backend_dds.py、収録ツール、位置を求めるスクリプトで共有する）。
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from .arm.types import JointState
from .robot_model import NUM_MOTORS

TOPIC_LOWSTATE = "rt/lowstate"

_initialized: tuple[int, str] | None = None
_lock = threading.Lock()


def init_dds(domain_id: int, network_interface: str) -> None:
    """DDS を初期化する（2 回目以降は、同じ設定なら何もしない。違う設定ならエラー）。"""
    global _initialized
    if not network_interface:
        raise ValueError(
            "network_interface が空。G1 につないでいる有線 NIC の名前（`ip -br a` で確認）を設定すること"
        )
    with _lock:
        if _initialized is not None:
            if _initialized != (domain_id, network_interface):
                raise RuntimeError(f"DDS はすでに {_initialized} で初期化されている（{domain_id}, {network_interface} とは違う）")
            return
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        print(f"[dds] DDS 初期化（domain={domain_id}, NIC={network_interface}）")
        ChannelFactoryInitialize(domain_id, network_interface)
        _initialized = (domain_id, network_interface)


def parse_lowstate(msg: Any) -> JointState:
    """unitree_hg の LowState_ → JointState。"""
    q = np.array([msg.motor_state[i].q for i in range(NUM_MOTORS)])
    dq = np.array([msg.motor_state[i].dq for i in range(NUM_MOTORS)])
    mode = np.array([msg.motor_state[i].mode for i in range(NUM_MOTORS)], dtype=int)
    # unitree の IMU 四元数は (w, x, y, z) の順
    imu = np.array(msg.imu_state.quaternion, dtype=float)
    return JointState(q=q, dq=dq, motor_mode=mode, mode_machine=int(msg.mode_machine),
                      stamp=time.monotonic(), imu_quat=imu)


class LowStateReader:
    """rt/lowstate を受信して、最新の JointState を保持する。"""

    def __init__(self, domain_id: int, network_interface: str) -> None:
        self._domain_id = domain_id
        self._iface = network_interface
        self._lock = threading.Lock()
        self._state: JointState | None = None
        self.count = 0

    def open(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        init_dds(self._domain_id, self._iface)
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._on_state, 10)

    def _on_state(self, msg: Any) -> None:
        st = parse_lowstate(msg)
        with self._lock:
            self._state = st
            self.count += 1

    def latest(self) -> JointState | None:
        with self._lock:
            return self._state

    def wait(self, timeout_s: float) -> JointState:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            st = self.latest()
            if st is not None:
                return st
            time.sleep(0.05)
        raise TimeoutError(f"{timeout_s} 秒待っても lowstate が届かない（有線接続・NIC 名・G1 の電源を確認）")
