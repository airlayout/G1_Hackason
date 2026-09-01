"""`slam_operate` との通信手段を抽象化する。

`nav/mission.py` はここだけを通して外と話す。差し替えの狙いはこう:

| 実装 | 相手 | 動く場所 |
|---|---|---|
| `FakeTransport`（`sim/fake_service.py`） | プロセス内の偽サービス | どこでも（標準ライブラリのみ） |
| `RealTransport` | 実機PC1の `slam_operate` | PC2（unitree_sdk2py が要る） |

**時計もここに持たせる。** 偽サービスは仮想時間で動かしたい（テストが数秒で終わる）が、
実機は実時間で待つ必要がある。`mission.py` が `time.sleep` を直接呼ぶと
この切り替えができなくなるので、`now()` と `sleep()` をトランスポートの責務にした。

`RealTransport` は `unitree_sdk2py` を**関数の中で import する**。
このモジュール自体は手元のMac（DDSが入らない）でも読めるようにしておきたいため。
"""

from __future__ import annotations

import abc
import json
import threading
import time
from typing import Any

from .protocol import (
    SERVICE_NAME,
    SERVICE_VERSION,
    TOPIC_SLAM_INFO,
    TOPIC_SLAM_KEY_INFO,
    TYPE_CTRL_INFO,
    TYPE_TASK_RESULT,
    CtrlInfo,
    ServiceResponse,
    parse_ctrl_info,
    parse_response,
    parse_slam_info,
    parse_task_result,
)

# 事前登録が必要なapi-id。`Client._RegistApi` を通していないapi-idは
# RPC_ERR_CLIENT_API_NOT_REG(3103) で即座に弾かれ、機体まで届かない。
REGISTERED_API_IDS = (1801, 1802, 1804, 1102, 1201, 1202, 1901)

# RPCの応答待ち[s]。1804は実測0.01秒で返る（失敗時）が、
# 成功時は地図の読み込みが入るので長めに取る。
DEFAULT_RPC_TIMEOUT_S = 10.0


class SlamTransport(abc.ABC):
    """`slam_operate` への窓口。"""

    @abc.abstractmethod
    def call(self, api_id: int, request: dict[str, Any]) -> ServiceResponse:
        """APIを1つ呼ぶ。応答が壊れていたら ValueError を投げる。"""

    @abc.abstractmethod
    def latest_ctrl_info(self) -> CtrlInfo | None:
        """最後に受け取った `ctrl_info`。まだ1つも来ていなければ None。"""

    @abc.abstractmethod
    def take_task_results(self) -> list:
        """溜まっている `task_result` を取り出して消す。

        取り出したら消すのは、同じ到達通知で次の区間まで終わったことに
        しないため。`is_arrived` は区間ごとに1回だけ効いてほしい。
        """

    @abc.abstractmethod
    def now(self) -> float:
        """単調増加する秒。実機は実時間、偽サービスは仮想時間。"""

    @abc.abstractmethod
    def sleep(self, seconds: float) -> None:
        """時間を進める。"""

    def close(self) -> None:
        """後片付け。既定では何もしない。"""


class RealTransport(SlamTransport):
    """実機PC1の `slam_operate` を叩く。**PC2の上で動かす前提**。

    Client PC（Mac）には cyclonedds も unitree_sdk2py も入っていないので、
    このクラスは手元では動かない。import だけは通るようにしてある。

    ⚠️ **未検証。** 2026-09-02時点で1802/1804/1102の成功パスを一度も通していない。
    `_Call` の戻り値の `code` はRPC層の状態コードで、レスポンスJSONの
    `errorCode` とは別物（実測: 507のとき code=1）。
    """

    def __init__(
        self,
        *,
        network_interface: str = "eth0",
        domain_id: int = 0,
        sdk_path: str | None = "/home/unitree/unitree_sdk2_python",
        rpc_timeout_s: float = DEFAULT_RPC_TIMEOUT_S,
    ) -> None:
        self._lock = threading.Lock()
        self._latest_ctrl_info: CtrlInfo | None = None
        self._task_results: list = []
        self._client = self._build_client(network_interface, domain_id, sdk_path, rpc_timeout_s)
        self._subscribers = self._subscribe()

    def call(self, api_id: int, request: dict[str, Any]) -> ServiceResponse:
        code, data = self._client._Call(api_id, json.dumps(request))
        if not data:
            raise ValueError(f"api-id {api_id} の応答が空（RPC code={code}）")
        return parse_response(data)

    def latest_ctrl_info(self) -> CtrlInfo | None:
        with self._lock:
            return self._latest_ctrl_info

    def take_task_results(self) -> list:
        with self._lock:
            taken, self._task_results = self._task_results, []
        return taken

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def close(self) -> None:
        self._subscribers.clear()

    # -- ここから下は unitree_sdk2py を触る --

    @staticmethod
    def _build_client(interface: str, domain_id: int, sdk_path: str | None, timeout: float):
        import sys

        if sdk_path and sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.rpc.client import Client

        ChannelFactoryInitialize(domain_id, interface)
        client = Client(SERVICE_NAME, False)
        client.SetTimeout(timeout)
        client._SetApiVerson(SERVICE_VERSION)
        for api_id in REGISTERED_API_IDS:
            client._RegistApi(api_id, 0)
        return client

    def _subscribe(self) -> list:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        info = ChannelSubscriber(TOPIC_SLAM_INFO, String_)
        info.Init(self._on_slam_info, 10)
        key = ChannelSubscriber(TOPIC_SLAM_KEY_INFO, String_)
        key.Init(self._on_slam_key_info, 10)
        return [info, key]

    def _on_slam_info(self, message) -> None:
        """`rt/slam_info` は約5Hz。`ctrl_info` 以外（robot_data/pos_info）は捨てる。"""

        try:
            kind, _ = parse_slam_info(message.data)
            if kind != TYPE_CTRL_INFO:
                return
            parsed = parse_ctrl_info(message.data)
        except ValueError:
            return  # 壊れた1フレームで巡回を止める理由はない。次のフレームが5Hzで来る
        with self._lock:
            self._latest_ctrl_info = parsed

    def _on_slam_key_info(self, message) -> None:
        try:
            kind, _ = parse_slam_info(message.data)
            if kind != TYPE_TASK_RESULT:
                return
            parsed = parse_task_result(message.data)
        except ValueError:
            return
        with self._lock:
            self._task_results.append(parsed)
