"""DDS（unitree_sdk2py）の初期化と、lowstate の受信。実機用（と、ループバックの模擬ロボット用）。

ChannelFactoryInitialize は 1 つのプロセスで 1 回しか呼べないので、ここで 1 回だけ呼ぶようにする
（腕の指令部分 backend_dds.py、収録ツール、位置を求めるスクリプトで共有する）。

実機と模擬ロボット（sim/sim_robot_server.py）を混ぜないための決まり:
- 模擬ロボットはネットワークの口 lo（ループバック。このマシンの中だけ）と domain 1 だけを使う
- 口が lo なら domain を 1 にする（設定の domain_id は使わない）。実機の口なら設定の domain（G1 は 0）を使う
- 模擬ロボットは lowstate の reserve[0] に目印（SIM_MARKER）を入れる。最初に受け取った lowstate で、
  口と目印が食い違っていたら中止する（check_peer）

lowstate は実機で約 1,000 通 / 秒届く。SDK の ChannelSubscriber のコールバックで全部受けると、それだけで 1 コアの約 72%
を使う（2026-10-31、PC2 で実測。変換の parse_lowstate の分は 2% ほどで、ほとんどは 1 通ごとの Python の呼び出しと
受信データの変換）。そこで、最新の 1 通だけを持つ DataReader を作り、読むときに取りに行く（LatestLowState。
50 Hz で読んで約 10%）。全部の数を数えたいとき（受け取り頻度の確認）だけ、コールバックで受ける。
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


_participant: Any = None


def _shared_participant() -> Any:
    """init_dds の後に、このプロセスで 1 つの DomainParticipant を作って使い回す（ChannelFactory と同じ domain・口）。"""
    global _participant
    with _lock:
        if _initialized is None:
            raise RuntimeError("init_dds を先に呼ぶこと")
        if _participant is None:
            from cyclonedds.domain import DomainParticipant

            _participant = DomainParticipant(_initialized[0])
        return _participant


class LatestLowState:
    """rt/lowstate の最新の 1 通だけを持つ受け口。latest() を呼んだときに新しい 1 通があれば取り込む。

    stamp は取り込んだ時刻（届いてから、読む間隔ぶん遅れることがある）。lowstate が途切れると新しい 1 通が来ないので、
    stamp は古いまま残り、今までどおり「何秒途切れているか」で見分けられる。count は取り込んだ数（届いた数ではない）。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._reader: Any = None
        self._state: JointState | None = None
        self.count = 0

    def open(self, domain_id: int, network_interface: str) -> None:
        from cyclonedds.qos import Policy, Qos
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        init_dds(domain_id, network_interface)
        dp = _shared_participant()
        self._reader = DataReader(dp, Topic(dp, TOPIC_LOWSTATE, LowState_), Qos(Policy.History.KeepLast(1)))

    def latest(self) -> JointState | None:
        from cyclonedds.internal import InvalidSample

        with self._lock:
            if self._reader is not None:
                for msg in self._reader.take(N=1):
                    if not isinstance(msg, InvalidSample):
                        self._state = parse_lowstate(msg)
                        self.count += 1
            return self._state


class LowStateReader:
    """rt/lowstate を受信して、最新の JointState を保持する。

    every_message=False（既定）: 読むときに最新の 1 通を取りに行く（LatestLowState）。count は取り込んだ数
    every_message=True: SDK のコールバックで全部受ける（CPU を多く使う）。count は届いた数（受け取り頻度の確認用）
    """

    def __init__(self, domain_id: int, network_interface: str, every_message: bool = False) -> None:
        self._domain_id = domain_id
        self._iface = network_interface
        self._every = every_message
        self._lock = threading.Lock()
        self._state: JointState | None = None
        self._latest = LatestLowState()
        self._count = 0

    def open(self) -> None:
        if not self._every:
            self._latest.open(self._domain_id, self._iface)
            return
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        init_dds(self._domain_id, self._iface)
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._on_state, 10)

    def _on_state(self, msg: Any) -> None:
        st = parse_lowstate(msg)
        with self._lock:
            self._state = st
            self._count += 1

    @property
    def count(self) -> int:
        return self._count if self._every else self._latest.count

    def latest(self) -> JointState | None:
        if not self._every:
            return self._latest.latest()
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
