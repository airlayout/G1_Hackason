#!/usr/bin/env python3
"""PC2から`slam_operate`のAPIを叩いて建図を開始・終了する。

`Mapping/real/backends/onboard_unitree/src/g1_onboard_lio.cpp`のPython移植。
C++版はDockerコンテナ内でしか動かないが、こちらは`unitree_sdk2py`だけで動くので
PC2にscpするだけで使える。

⚠️ **実機未検証（2026-09-02時点）。** 実装はC++版と`Navigation/nav/protocol.py`の
両方に一致させ、`tests/test_mapping_ctl.py`で突き合わせてあるが、この経路で実際に
RPCを投げたことはまだ無い。

既知の事実:

- `1801`（建図開始）はC++版で成功実績がある（2026-08-26のMapping初回試験）
- `1802`（建図終了・保存）は**成功パスを誰も通していない**。2026-08-26は2回とも
  停止時に通信が切れて届かなかった
- `1802`の`address`は**PC1（192.168.123.161）のファイルシステム**を指す。
  PC2に置いたパスを渡しても読めない。公式はディスク圧迫を避けるため
  `test1.pcd`〜`test10.pcd`の使い回しを推奨している

**PC2のPython 3.8で動くように書いてある**（`unitree_sdk2py`が入っているのは
そちらだけで、`nav/protocol.py`の3.10+構文は読めないため自己完結させている）。

使い方:

    python3 mapping_ctl.py probe                       # サービスの応答を確認する
    python3 mapping_ctl.py start                       # 1801 建図開始
    python3 mapping_ctl.py stop --map /home/unitree/test1.pcd   # 1802 終了・保存
    python3 mapping_ctl.py close                       # 1901 SLAM終了
"""
import argparse
import json
import sys
import threading
import time

DEFAULT_SDK_PATH = "/home/unitree/unitree_sdk2_python"

# --- `slam_operate` v1.0.0.1。C++版のkServiceName/kApiVersionと同じ ---
SERVICE_NAME = "slam_operate"
SERVICE_VERSION = "1.0.0.1"

# --- api-id。C++版のk*と同じ ---
API_START_MAPPING = 1801
API_END_MAPPING = 1802
API_CLOSE_SLAM = 1901

# C++版が`UT_ROBOT_CLIENT_REG_API_NO_PROI`で登録しているのはこの3つだけ。
# 登録していないapi-idはRPC_ERR_CLIENT_API_NOT_REG(3103)で機体まで届かない。
REGISTERED_API_IDS = (API_CLOSE_SLAM, API_START_MAPPING, API_END_MAPPING)

# 公式で「固定值」と明記されている値
SLAM_TYPE_INDOOR = "indoor"

TOPIC_SLAM_INFO = "rt/slam_info"
TOPIC_SLAM_KEY_INFO = "rt/slam_key_info"

# 1802が書く既定の保存先。PC1上のパスであることに注意
DEFAULT_MAP_PATH = "/home/unitree/test1.pcd"

# 実測で判明しているerrorCode
ERROR_LOAD_PCD_FAILED = 507


# ----------------------------------------------------------------- リクエスト
# `Navigation/nav/protocol.py`の同名関数と同じJSONを返す。
# 一致は tests/test_mapping_ctl.py で固定してある。


def start_mapping_request():
    """1801 建図開始。"""
    return {"data": {"slam_type": SLAM_TYPE_INDOOR}}


def end_mapping_request(address):
    """1802 建図終了・保存。addressはPC1のファイルシステム上のパス。"""
    return {"data": {"address": address}}


def close_slam_request():
    """1901 SLAM終了。"""
    return {"data": {}}


def parse_response(payload):
    """レスポンスJSONを読む。壊れていたらValueErrorにする（黙って握らない）。"""
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8")
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except ValueError as error:
            raise ValueError("レスポンスがJSONとして読めません: {!r}".format(payload)) from error
    if not isinstance(payload, dict):
        raise ValueError("レスポンスがオブジェクトではありません: {!r}".format(payload))
    return {
        "succeed": bool(payload.get("succeed", False)),
        "errorCode": int(payload.get("errorCode", -1)),
        "info": str(payload.get("info", "")),
        "data": payload.get("data") or {},
    }


# --------------------------------------------------------------------- 本体


class SlamOperateClient(object):
    """`slam_operate`への窓口。C++版のSlamClientと同じ手順で初期化する。"""

    def __init__(self, interface="eth0", domain_id=0, timeout_s=5.0,
                 sdk_path=DEFAULT_SDK_PATH, subscribe_status=True,
                 init_channel=True):
        """`init_channel=False`は、呼び出し側が既に
        `ChannelFactoryInitialize`を済ませている場合に使う。
        SDKのチャネル初期化はプロセスに一度しか通せないため、
        `record_dds_to_bag.py --with-mapping`のように同一プロセスで
        購読とRPCを両方やる場合はこちらを渡す。
        """
        if sdk_path and sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.rpc.client import Client

        if init_channel:
            ChannelFactoryInitialize(domain_id, interface)
        client = Client(SERVICE_NAME, False)
        client.SetTimeout(timeout_s)
        # SDK側の綴りが `_SetApiVerson`（Version ではない）。直すとAttributeErrorになる
        client._SetApiVerson(SERVICE_VERSION)
        for api_id in REGISTERED_API_IDS:
            client._RegistApi(api_id, 0)
        self._client = client

        self._lock = threading.Lock()
        self._status_seen = threading.Event()
        self._last_status = {}
        self._subscribers = []
        if subscribe_status:
            self._subscribe_status()

    def _subscribe_status(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        def make_handler(topic):
            def handle(message):
                with self._lock:
                    self._last_status[topic] = message.data
                self._status_seen.set()
            return handle

        for topic in (TOPIC_SLAM_INFO, TOPIC_SLAM_KEY_INFO):
            subscriber = ChannelSubscriber(topic, String_)
            subscriber.Init(make_handler(topic), 1)
            self._subscribers.append(subscriber)

    def wait_for_status(self, timeout_s):
        """slam_info / slam_key_info のどちらかを受け取るまで待つ。"""
        return self._status_seen.wait(timeout_s)

    def latest_status(self):
        with self._lock:
            return dict(self._last_status)

    def call(self, api_id, request):
        """APIを1つ呼ぶ。戻り値は (rpc_code, response_dict_or_None, raw)。

        `rpc_code`はRPC層の状態コードで、レスポンスJSONの`errorCode`とは別物。
        実測では errorCode=507 のとき rpc_code=1 だった。
        """
        rpc_code, raw = self._client._Call(api_id, json.dumps(request))
        response = None
        if raw:
            try:
                response = parse_response(raw)
            except ValueError:
                response = None
        return rpc_code, response, raw


def _report(api_id, rpc_code, response, raw):
    print("[RPC] api_id={} rpc_code={} response={}".format(api_id, rpc_code, raw))
    if rpc_code != 0:
        print("[ERROR] RPCが失敗しました（api-id未登録なら3103、応答なしならタイムアウト）",
              file=sys.stderr)
        return 1
    if response is None:
        print("[ERROR] 応答が空、またはJSONとして読めませんでした", file=sys.stderr)
        return 1
    if not response["succeed"]:
        message = "[ERROR] サービスが失敗を返しました: errorCode={} info={!r}".format(
            response["errorCode"], response["info"])
        if response["errorCode"] == ERROR_LOAD_PCD_FAILED:
            message += "\n        （507はファイル不在・形式不正・権限を区別しない総称エラー）"
        print(message, file=sys.stderr)
        return 1
    print("[OK] 成功しました")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["probe", "start", "stop", "close", "status"])
    parser.add_argument("--map", dest="map_path", default=None,
                        help="1802の保存先。**PC1上のパス**（既定 {}）".format(DEFAULT_MAP_PATH))
    parser.add_argument("--iface", default="eth0", help="PC2のG1側NIC")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=5.0, help="RPC応答待ち[秒]")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_PATH)
    arguments = parser.parse_args(argv)

    if arguments.action == "stop" and not arguments.map_path:
        arguments.map_path = DEFAULT_MAP_PATH

    client = SlamOperateClient(
        interface=arguments.iface, domain_id=arguments.domain_id,
        timeout_s=arguments.timeout, sdk_path=arguments.sdk_path)

    if arguments.action in ("probe", "status"):
        if not client.wait_for_status(arguments.timeout):
            print("[ERROR] {} / {} を受信できませんでした".format(
                TOPIC_SLAM_INFO, TOPIC_SLAM_KEY_INFO), file=sys.stderr)
            return 1
        print("[OK] G1内蔵SLAMサービスの応答を確認しました")
        if arguments.action == "status":
            # ctrl_info が来るまで少し待つと state が見える
            time.sleep(0.5)
            for topic, payload in sorted(client.latest_status().items()):
                try:
                    body = json.loads(payload)
                except ValueError:
                    print("  {}: {}".format(topic, payload[:160]))
                    continue
                machine = body.get("data", {}).get("stateMachine", {})
                print("  {}: type={} state={} ctrName={}".format(
                    topic, body.get("type"), machine.get("state"),
                    machine.get("ctrName")))
        return 0

    if arguments.action == "start":
        print("[1801] 建図を開始します（slam_type={}）".format(SLAM_TYPE_INDOOR))
        return _report(API_START_MAPPING,
                       *client.call(API_START_MAPPING, start_mapping_request()))

    if arguments.action == "stop":
        print("[1802] 建図を終了し保存します: {}".format(arguments.map_path))
        print("       ※ これはPC1(192.168.123.161)上のパスです。PC2ではありません")
        return _report(API_END_MAPPING,
                       *client.call(API_END_MAPPING,
                                    end_mapping_request(arguments.map_path)))

    if arguments.action == "close":
        print("[1901] SLAMを終了します")
        return _report(API_CLOSE_SLAM,
                       *client.call(API_CLOSE_SLAM, close_slam_request()))

    return 2


if __name__ == "__main__":
    sys.exit(main())
