#!/usr/bin/env python3
"""slam_operate へ「機体を動かさない」APIだけを送る。**PC2 で動かす。**

## 安全設計

`unitree_sdk2py` の RPC クライアントは `_RegistApi()` で登録した api-id しか
送れない。**そこで 1801(建図開始) / 1901(SLAM終了) / 1802(建図保存) だけを登録し、
1102(位姿導航=移動) と 1202(再開) は登録しない。** 引数で指定しても送れない。

    python3 send_slam_api.py 1801    # 建図開始
    python3 send_slam_api.py 1901    # SLAM終了
"""
import json
import sys

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

SERVICE_NAME = "slam_operate"
SERVICE_VERSION = "1.0.0.1"

# 機体を動かさない API だけ。1102(移動)/1202(再開)/1201 は意図的に含めない
SAFE_APIS = {
    1801: ("START_MAPPING", {"data": {"slam_type": "indoor"}}),
    1901: ("CLOSE_SLAM", {"data": {}}),
}


def main() -> int:
    api_id = int(sys.argv[1])
    if api_id not in SAFE_APIS:
        print(f"[NG] api-id {api_id} は許可リストに無い。使えるのは {sorted(SAFE_APIS)} だけ")
        return 2
    name, request = SAFE_APIS[api_id]

    ChannelFactoryInitialize(0, "eth0")
    client = Client(SERVICE_NAME, False)
    client.SetTimeout(10.0)
    client._SetApiVerson(SERVICE_VERSION)
    for safe_id in SAFE_APIS:
        client._RegistApi(safe_id, 0)

    print(f"[send] api-id={api_id} ({name}) request={json.dumps(request)}")
    code, data = client._Call(api_id, json.dumps(request))
    print(f"[recv] RPC code={code}")
    print(f"[recv] payload={data}")
    try:
        j = json.loads(data) if data else {}
        print(f"[recv] succeed={j.get('succeed')} errorCode={j.get('errorCode')} info={j.get('info')!r}")
    except Exception as e:
        print(f"[warn] JSON として読めなかった: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
