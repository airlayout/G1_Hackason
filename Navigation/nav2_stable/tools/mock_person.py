#!/usr/bin/env python3
"""経路上に「人」を置く（モック検証用）。指定の座標に点群のかたまりを配信する。

2026-09-24 の実機で「**人が横切るたびに巡回が終わる**」ことが分かった
（[findings/real_run_20260924.md](../findings/real_run_20260924.md) §5）。
その状況を実機なしで再現するための道具。

    python3 mock_person.py --x -1.5 --y -1.0 --frame map --seconds 20

⚠️ **`fake_sensor_publisher.py` と同じトピックへ出す**（`/utlidar/cloud_livox_mid360`）。
あちらは常に空を流すので、こちらが出している間だけ障害物が見える。

⚠️ **`obstacle_min_range: 0.9` はセンサー原点からの距離**。`--frame` に指定した
フレームの原点から 0.9m 以上離れた場所に置くこと（近すぎるとマークされない）。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(frame: str, stamp, points: list[tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


def main() -> int:
    ap = argparse.ArgumentParser(description="経路上に人を模した点群を置く")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--frame", default="map")
    ap.add_argument("--radius", type=float, default=0.20, help="人の半径[m]")
    ap.add_argument("--z-min", type=float, default=0.30)
    ap.add_argument("--z-max", type=float, default=1.50)
    ap.add_argument("--seconds", type=float, default=20.0, help="何秒間そこに居るか")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--topic", default="/utlidar/cloud_livox_mid360")
    args = ap.parse_args()

    # 円柱状に点を並べる（LiDAR が人を捉えたときの見え方に近い）
    points: list[tuple[float, float, float]] = []
    z = args.z_min
    while z <= args.z_max:
        for i in range(24):
            th = 2 * math.pi * i / 24
            points.append((args.x + args.radius * math.cos(th),
                           args.y + args.radius * math.sin(th), z))
        z += 0.1

    rclpy.init()
    node = Node("mock_person")
    pub = node.create_publisher(PointCloud2, args.topic, 10)
    node.get_logger().info(
        f"({args.x:+.2f}, {args.y:+.2f}) [{args.frame}] に {len(points)} 点を "
        f"{args.seconds:.0f} 秒間置く")

    end = time.time() + args.seconds
    period = 1.0 / args.rate
    while rclpy.ok() and time.time() < end:
        pub.publish(make_cloud(args.frame, node.get_clock().now().to_msg(), points))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.get_logger().info("人が通り過ぎた（配信を止めた）")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
