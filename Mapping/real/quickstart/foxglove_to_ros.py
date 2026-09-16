#!/usr/bin/env python3
"""Foxglove Bridge の WebSocket を受けて、**こちら側の DDS に ROS トピックとして流し直す**。

    # コンテナで（Mac 側）
    python3 foxglove_to_ros.py --host 10.42.0.76
    # 別の端末で RViz2 を開けば、機体のトピックが普通に見える

## なぜ要るのか

機体を無線で動かしたいが、**DDS は AP を越えられない**（2026-09-16 に実測）:

| PC2 の DDS | 同一ホスト内 | PC1 の LiDAR | コンテナから見える |
|---|---|---|---|
| eth0 単独 | OK | OK | ⛔ |
| wlan0 単独 | OK | **⛔ 0 枚** | OK |
| 2 NIC（peer=コンテナのみ）| ⛔ | ⛔ | — |
| 2 NIC（+ localhost）| OK | **⛔ 0 枚** | — |

2 NIC にすると CycloneDDS が `selected interface "wlan0"` となり、**eth0 側に居る PC1 の
LiDAR が見えなくなる**。priority 指定・ExternalNetworkAddress・PC1 を peer に足す、
いずれも効かなかった。⇒ **DDS を無線に出す道は閉じている。**

一方 foxglove_bridge は **WebSocket（TCP）**なので AP を素通りする（実測 0.06 コア）。
ここで受けて DDS に戻せば、**RViz2 は「普通のローカルな ROS」として使える**。

## 仕組み

    PC2: ROS ──> foxglove_bridge ──(WS/TCP・AP 越え)──> ここ ──> DDS ──> RViz2

CDR をそのまま持ってくるので、型ごとの変換コードは要らない。`deserialize_message`
で ROS のメッセージに戻してから publish する（publish 側で QoS を選ぶため）。

⚠️ **一方向である。** ここから機体へは何も送らない。ゴールを投げるのは別の口
（`measure_walk.sh` が `/goal_pose` に publish する）。

## QoS（ここを外すと RViz2 に出ない）

地図とコストマップは **TRANSIENT_LOCAL** でないと、後から開いた RViz2 に届かない。
`/scan` は発行側が BEST_EFFORT なので合わせる。`/tf` は RELIABLE の VOLATILE。
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
from foxglove_ws import MSG_DATA, OP_BINARY, FoxgloveClient  # noqa: E402

# MessageData のバイト構成: [0]=0x01 / [1:5]=購読 id / [5:13]=時刻 / [13:]=CDR 本体
CDR_OFFSET = 13

# 既定で中継するもの。RViz2 で Nav2 を見るのに要る最小限。
# ⚠️ `/map` は入れない。latched（TRANSIENT_LOCAL）なので橋が再送せず 0 件になる
#    （2026-09-16 実測）。地図は**こちら側で map_server を立てる**方が確実で、
#    同じ格子を AP 越しに何度も運ばずに済む。
DEFAULT_TOPICS = [
    "/tf", "/tf_static", "/scan",
    "/global_costmap/costmap", "/local_costmap/costmap",
    "/local_costmap/published_footprint", "/plan",
]

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# ⚠️ `/tf_static` は**静的 TF ごとに 1 通**来る（body->base_link, base_link->livox_frame …）。
#    depth=1 にすると最後の 1 通しか残らず、**後から開いた RViz2 で鎖が切れる**
#    （2026-09-16 に踏んだ。tf2_echo map base_link が無言になる）。
STATIC = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
SENSOR = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)


def qos_for(topic: str) -> QoSProfile:
    """⚠️ 地図系を VOLATILE にすると、後から開いた RViz2 には**何も出ない**。"""
    if topic == "/tf_static":
        return STATIC
    if topic == "/map" or topic.endswith("/costmap"):
        return LATCHED
    if topic == "/scan":
        return SENSOR
    return RELIABLE


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="10.42.0.76", help="foxglove_bridge のホスト")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--topic", action="append", default=None,
                    help="中継するトピック（複数可）。省略すると既定の一式")
    ap.add_argument("--all", action="store_true",
                    help="橋が広告する全トピック。⚠️ 点群まで来るので AP の帯域を食う")
    ap.add_argument("--settle", type=float, default=4.0, help="advertise を集める秒数")
    a = ap.parse_args()

    print(f"[relay] 橋に繋ぐ ws://{a.host}:{a.port}")
    client = FoxgloveClient(a.host, a.port)
    channels = client.collect_channels(settle=a.settle)
    print(f"[relay] 橋が広告するトピック {len(channels)} 件")

    if a.all:
        wanted = sorted(channels)
    else:
        wanted = a.topic if a.topic else DEFAULT_TOPICS

    usable, missing = [], []
    for t in wanted:
        ch = channels.get(t)
        if ch is None:
            missing.append(t)
            continue
        if ch.get("encoding") != "cdr":
            print(f"[relay] ⚠️ {t}: encoding={ch.get('encoding')} は中継できない")
            continue
        usable.append(t)
    if missing:
        print(f"[relay] ⚠️ 橋に無い: {', '.join(missing)}")
    if not usable:
        print("[relay] ⛔ 中継できるトピックが 1 つも無い", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("foxglove_to_ros")
    pubs, types = {}, {}
    for t in usable:
        schema = channels[t]["schemaName"]          # 例: nav_msgs/msg/OccupancyGrid
        try:
            msg_type = get_message(schema)
        except Exception as exc:                     # 型が無いものは黙って飛ばさない
            print(f"[relay] ⚠️ {t}: 型 {schema} を解決できない（{exc}）")
            continue
        types[t] = msg_type
        pubs[t] = node.create_publisher(msg_type, t, qos_for(t))
        print(f"[relay]   {t:38s} {schema}")

    by_id = client.subscribe(list(pubs))
    print(f"[relay] 中継開始（{len(pubs)} トピック）。Ctrl-C で止める")

    counts = {t: 0 for t in pubs}
    last_report = time.time()
    client.sock.settimeout(5.0)
    try:
        while rclpy.ok():
            try:
                opcode, payload = client.recv_frame()
            except TimeoutError:
                continue
            except OSError:
                continue
            if opcode != OP_BINARY or not payload or payload[0] != MSG_DATA:
                continue
            sub_id = struct.unpack_from("<I", payload, 1)[0]
            topic = by_id.get(sub_id)
            if topic is None or topic not in pubs:
                continue
            try:
                msg = deserialize_message(bytes(payload[CDR_OFFSET:]), types[topic])
            except Exception:
                continue                             # 壊れた 1 通で止めない
            try:
                pubs[topic].publish(msg)
            except Exception:
                break                                # 終了中。ここで静かに抜ける
            counts[topic] += 1

            now = time.time()
            if now - last_report >= 10.0:
                alive = [f"{t.rsplit('/', 1)[-1]}={n}" for t, n in counts.items() if n]
                dead = [t for t, n in counts.items() if not n]
                print(f"[relay] 10 秒: {' '.join(alive) if alive else '(0 件)'}"
                      + (f"  ⚠️ 来ない: {', '.join(dead)}" if dead else ""))
                counts = {t: 0 for t in pubs}
                last_report = now
    except KeyboardInterrupt:
        print("\n[relay] 止める")
    finally:
        try:
            client.close()
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
