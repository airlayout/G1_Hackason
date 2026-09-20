#!/usr/bin/env python3
"""Nav2 の /cmd_vel を `loco_driver.py` へ渡す。**PC2 の pixi 環境（Python 3.11）で動かす。**

## なぜこちらは SDK を呼ばないのか

PC2 では **rclpy と unitree_sdk2py が同じ Python に同居できない**（2026-09-06 実測。
pixi の 3.11 には rclpy だけ、system の 3.8 には SDK だけがある）。
そこで ROS 側（これ）と SDK 側（`loco_driver.py`）を分け、localhost の UDP で繋ぐ。

    [Nav2] --/cmd_vel--> [これ (rclpy/py3.11)] --UDP 127.0.0.1--> [loco_driver.py (SDK/py3.8)] --> 足

**安全機構はこちらには置かない。**速度クランプもウォッチドッグも発進ゲートも
`loco_driver.py` 側にある。止められるのはあちらだけなので、判断もあちらに寄せる。
このプロセスが落ちれば UDP が途切れ、あちらのウォッチドッグが足を止める。

## 使い方

    ssh g1 'cd ~/g1_humble && ~/.pixi/bin/pixi run python ~/nav_tools/cmd_vel_bridge.py'
"""
import argparse
import json
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

DEFAULT_PORT = 47600


class CmdVelBridge(Node):
    def __init__(self, topic: str, host: str, port: int) -> None:
        super().__init__("cmd_vel_bridge")
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._count = 0
        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self.get_logger().info(
            "[bridge] {} を購読し {}:{} へ中継します（安全機構は loco_driver 側）".format(
                topic, host, port))

    def _on_cmd_vel(self, msg: Twist) -> None:
        payload = json.dumps({
            "vx": msg.linear.x, "vy": msg.linear.y, "vyaw": msg.angular.z,
        }).encode("utf-8")
        # UDP なので取りこぼしうるが、途切れたら向こうのウォッチドッグが止める。
        # 送信で詰まらせないことのほうが大事
        self._sock.sendto(payload, self._addr)
        self._count += 1
        if self._count in (1, 10) or self._count % 200 == 0:
            self.get_logger().info(
                "[bridge] {} 件中継 (最新 vx={:+.3f} vy={:+.3f} vyaw={:+.3f})".format(
                    self._count, msg.linear.x, msg.linear.y, msg.angular.z))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/cmd_vel")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = p.parse_args(argv)

    rclpy.init()
    node = CmdVelBridge(args.topic, args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[bridge] 停止します")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
