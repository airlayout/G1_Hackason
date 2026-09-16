#!/usr/bin/env python3
"""こちら側（コンテナ）の ROS トピックを、**Foxglove Bridge 経由で機体へ publish する**。

    # コンテナで。RViz2 の「2D Goal Pose」を機体の /goal_pose へ返す
    python3 ros_to_foxglove.py --host 10.42.0.76

## なぜ要るのか

`foxglove_to_ros.py` は**機体 → こちら**の一方向しかない。それだけだと RViz2 で
地図は見えても、**クリックしたゴールが機体に届かない**（コンテナ側の DDS に
publish されるだけで、DDS は AP を越えられない。2026-09-16 実測）。

foxglove_bridge は `clientPublish` の能力を持つ（serverInfo の capabilities で確認済み）。
こちらから advertise して ClientMessageData を送れば、**橋が機体側で publish し直す**。

    RViz2 --/goal_pose--> [これ] --WS/TCP--> foxglove_bridge --> 機体の /goal_pose --> Nav2

## 何を返すか

既定は `/goal_pose` だけ。**戻す口は最小限にする** —— ここは機体を動かす側なので、
広げるほど事故の幅が広がる。`/initialpose` は測位をやり直す口なので、要るときだけ
`--topic` で明示する。

⚠️ `/cmd_vel` は**絶対にここから流さない**。速度指令は Nav2 → cmd_vel_bridge →
loco_driver の経路にウォッチドッグと発進ゲートが仕込んである。AP 越しの
WebSocket を挟むと、切れたときに止める仕組みが無くなる。

## プロトコル（Foxglove WebSocket v1 のクライアント publish）

    1. client -> server (JSON)   {"op":"advertise","channels":[{id, topic, encoding:"cdr", schemaName}]}
    2. client -> server (binary) [0x01][uint32 channelId][CDR 本体]
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
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
from foxglove_ws import FoxgloveClient  # noqa: E402

CLIENT_MSG_DATA = 0x01                  # クライアント→サーバのバイナリ先頭 1 バイト

# 機体へ返す口。⚠️ 増やすほど事故の幅が広がる。既定は 1 本だけ。
DEFAULT_TOPICS = {
    "/goal_pose": "geometry_msgs/msg/PoseStamped",
}
# 明示したときだけ返すもの
OPTIONAL_TOPICS = {
    "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
}

RELIABLE = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="10.42.0.76")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--topic", action="append", default=None,
                    help="返すトピック（複数可）。省略すると /goal_pose のみ")
    a = ap.parse_args()

    known = {**DEFAULT_TOPICS, **OPTIONAL_TOPICS}
    if a.topic:
        wanted = {}
        for t in a.topic:
            if t not in known:
                print(f"[back] ⛔ {t} は返す口として登録されていない", file=sys.stderr)
                return 2
            if t == "/cmd_vel":                      # 念のための二重の歯止め
                print("[back] ⛔ /cmd_vel はここから流さない", file=sys.stderr)
                return 2
            wanted[t] = known[t]
    else:
        wanted = dict(DEFAULT_TOPICS)

    print(f"[back] 橋に繋ぐ ws://{a.host}:{a.port}")
    client = FoxgloveClient(a.host, a.port)
    client.collect_channels(settle=3.0)
    caps = (client.server_info or {}).get("capabilities", [])
    if "clientPublish" not in caps:
        print(f"[back] ⛔ 橋が clientPublish に対応していない（{caps}）", file=sys.stderr)
        return 2

    channels, ids = [], {}
    for i, (topic, schema) in enumerate(sorted(wanted.items()), start=1):
        channels.append({"id": i, "topic": topic, "encoding": "cdr",
                         "schemaName": schema, "schemaEncoding": "ros2msg", "schema": ""})
        ids[topic] = i
    client.send_json({"op": "advertise", "channels": channels})
    time.sleep(1.0)

    rclpy.init()
    node = Node("ros_to_foxglove")
    sent = {t: 0 for t in wanted}

    def make_cb(topic: str):
        cid = ids[topic]

        def cb(msg):
            body = serialize_message(msg)
            client.send_binary(bytes([CLIENT_MSG_DATA]) + struct.pack("<I", cid) + body)
            sent[topic] += 1
            print(f"[back] {topic} を機体へ返した（累計 {sent[topic]} 件）")
        return cb

    for topic, schema in wanted.items():
        node.create_subscription(get_message(schema), topic, make_cb(topic), RELIABLE)
        print(f"[back]   {topic:16s} {schema}")

    print("[back] 待機中。RViz2 の「2D Goal Pose」でクリックすると機体へ届く")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[back] 止める")
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
