"""DDS（unitree_sdk2py）の初期化と、lowstate の受信。実機用（と、ループバックの模擬ロボット用）。

ChannelFactoryInitialize は 1 つのプロセスで 1 回しか呼べないので、ここで 1 回だけ呼ぶようにする
（腕の指令部分 backend_dds.py、収録ツール、位置を求めるスクリプトで共有する）。

実機と模擬ロボット（sim/sim_robot_server.py）を混ぜないための決まり:
- 模擬ロボットはネットワークの口 lo（ループバック。このマシンの中だけ）と domain 1 だけを使う
- 口が lo なら domain を 1 にする（設定の domain_id は使わない）。実機の口なら設定の domain（G1 は 0）を使う
- 模擬ロボットは lowstate の reserve[0] に目印（SIM_MARKER）を入れる。最初に受け取った lowstate で、
  口と目印が食い違っていたら中止する（check_peer）
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from .arm.types import JointState
from .robot_model import NUM_MOTORS

TOPIC_LOWSTATE = "rt/lowstate"

SIM_INTERFACE = "lo"
SIM_DOMAIN_ID = 1
SIM_MARKER = 0x5117B07  # 模擬ロボットの目印（lowstate の reserve[0]）


class PeerMismatchError(RuntimeError):
    """ネットワークの口（実機 / 模擬ロボット）と、受け取った lowstate の相手が食い違っている。"""


def is_sim_interface(network_interface: str) -> bool:
    return network_interface == SIM_INTERFACE


def resolve_domain(network_interface: str, domain_id: int) -> int:
    """口が lo（模擬ロボット）なら domain 1、それ以外は設定の値。"""
    return SIM_DOMAIN_ID if is_sim_interface(network_interface) else int(domain_id)


def peer_label(network_interface: str) -> str:
    return "模擬ロボット（ループバック）" if is_sim_interface(network_interface) else "実機"


def check_peer(state: JointState, network_interface: str) -> str:
    """受け取った lowstate が、口から想定した相手のものかを確かめる。食い違っていれば PeerMismatchError。"""
    sim = is_sim_interface(network_interface)
    if sim and not state.sim_marker:
        raise PeerMismatchError(
            f"口が {SIM_INTERFACE}（模擬ロボット）なのに、lowstate に模擬ロボットの目印が無い。中止する"
        )
    if not sim and state.sim_marker:
        raise PeerMismatchError(
            f"口が {network_interface}（実機）なのに、模擬ロボットの lowstate を受け取った。中止する"
        )
    label = "模擬ロボット（目印を確認）" if sim else "実機（模擬ロボットの目印なし）"
    print(f"[dds] 相手: {label}")
    return label

_initialized: tuple[int, str] | None = None
_lock = threading.Lock()


def init_dds(domain_id: int, network_interface: str) -> None:
    """DDS を初期化する（2 回目以降は、同じ設定なら何もしない。違う設定ならエラー）。"""
    global _initialized
    if not network_interface:
        raise ValueError(
            "network_interface が空。G1 につないでいる有線 NIC の名前（`ip -br a` で確認）を設定すること"
        )
    if is_sim_interface(network_interface):
        domain_id = SIM_DOMAIN_ID
    elif int(domain_id) == SIM_DOMAIN_ID:
        raise ValueError(f"実機の口（{network_interface}）で、模擬ロボット用の domain {SIM_DOMAIN_ID} は使えない")
    with _lock:
        if _initialized is not None:
            if _initialized != (domain_id, network_interface):
                raise RuntimeError(f"DDS はすでに {_initialized} で初期化されている（{domain_id}, {network_interface} とは違う）")
            return
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        print(f"[dds] ネットワークの口: {network_interface}、domain {domain_id}、相手: {peer_label(network_interface)}")
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
                      stamp=time.monotonic(), imu_quat=imu, sim_marker=int(msg.reserve[0]) == SIM_MARKER)


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
        """最初の lowstate を待ち、相手（実機 / 模擬ロボット）が口と合っているかを確かめる。"""
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            st = self.latest()
            if st is not None:
                check_peer(st, self._iface)
                return st
            time.sleep(0.05)
        raise TimeoutError(f"{timeout_s} 秒待っても lowstate が届かない（有線接続・NIC 名・G1 の電源を確認）")
