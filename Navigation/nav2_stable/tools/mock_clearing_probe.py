#!/usr/bin/env python3
"""足元の消し残りが消えるかを確かめる（モック検証用。tools/mock_nav2_proper_test.sh から呼ぶ）。

findings/collision_ahead_cause_20260925.md の現象を、実機なしで再現する。

1. 段階 A（3 秒）: 遠くの「壁」（半径 4m の輪、床から 0.5m）と、
   機体の 1.5m 前の**低い障害物**（床から 0.1〜0.3m）を同時に見せる
2. 段階 B（5 秒）: 障害物を取り除き、壁だけを見せ続ける
3. local costmap の、障害物があった格子の値を読む

センサーは base_link の上 1.21m（実機の MID-360 と同じ高さ。`livox_frame`）に置く。
壁へ向かうレイは、障害物の位置では高さ約 0.94m を通る。だから

- VoxelLayer（3 次元で消す）: 0.1〜0.3m のボクセルを通らない → **残る**
- ObstacleLayer（2 次元で消す）: その格子を通る → **消える**

となるはず。結果を `[clearing] 障害物の格子: 段階A=... 段階B=...` の 1 行で出す。

⚠️ fake_sensor_publisher を止めてから使うこと（同じトピックへ空の点群を流すため）。
⚠️ base_link→livox_frame の TF を先に出しておくこと（呼び出し側が static TF を立てる）。
"""

from __future__ import annotations

import math
import struct
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

SENSOR_HEIGHT = 1.21          # base_link からの高さ[m]
OBSTACLE_X = 1.5              # 障害物の位置（base_link の前方）[m]


def make_cloud(stamp, points: list[tuple[float, float, float]]) -> PointCloud2:
    """livox_frame 基準の点列を PointCloud2 にする。"""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = "livox_frame"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


def wall_points() -> list[tuple[float, float, float]]:
    """半径 4m・床から 0.5m の輪（1° 刻み）。壁の代わり。"""
    z = 0.5 - SENSOR_HEIGHT
    return [(4.0 * math.cos(math.radians(d)), 4.0 * math.sin(math.radians(d)), z)
            for d in range(360)]


def obstacle_points() -> list[tuple[float, float, float]]:
    """機体の 1.5m 前、床から 0.1〜0.3m の小さな箱（20cm 角）。"""
    pts = []
    for z in (0.1, 0.2, 0.3):
        for dx in (-0.1, 0.0, 0.1):
            for dy in (-0.1, 0.0, 0.1):
                pts.append((OBSTACLE_X + dx, dy, z - SENSOR_HEIGHT))
    return pts


def main() -> None:
    rclpy.init()
    node = Node("mock_clearing_probe")
    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(PointCloud2, "/utlidar/cloud_livox_mid360", 10)
    grids: list[OccupancyGrid] = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", grids.append, qos)

    def publish_for(seconds: float, points: list[tuple[float, float, float]]) -> None:
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(make_cloud(node.get_clock().now().to_msg(), points))
            rclpy.spin_once(node, timeout_sec=0.1)

    def obstacle_cell_value() -> int:
        """障害物があった格子（odom 基準）の現在の値。0〜100、-1=未知。"""
        grids.clear()
        end = time.time() + 5.0
        while not grids and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not grids:
            raise SystemExit("[clearing] /local_costmap/costmap が来ない")
        g = grids[-1]
        tf = buf.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
        yaw = 2.0 * math.atan2(tf.rotation.z, tf.rotation.w)
        ox = tf.translation.x + OBSTACLE_X * math.cos(yaw)
        oy = tf.translation.y + OBSTACLE_X * math.sin(yaw)
        a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        ix = int((ox - g.info.origin.position.x) / g.info.resolution)
        iy = int((oy - g.info.origin.position.y) / g.info.resolution)
        # 20cm 角の箱なので、周り ±2 セル（±10cm）の最大値を見る
        return int(a[iy - 2:iy + 3, ix - 2:ix + 3].max())

    for _ in range(40):
        if buf.can_transform("odom", "base_link", rclpy.time.Time()):
            break
        rclpy.spin_once(node, timeout_sec=0.5)

    walls = wall_points()
    publish_for(3.0, walls + obstacle_points())
    a_val = obstacle_cell_value()
    publish_for(5.0, walls)
    b_val = obstacle_cell_value()
    verdict = "消えた" if b_val < 99 <= a_val else ("載らなかった（試験不成立）" if a_val < 99 else "残った")
    print(f"[clearing] 障害物の格子: 段階A={a_val} 段階B={b_val} → {verdict}")
    # 試験の後もセンサー鮮度が切れないよう、呼び出し側が fake_sensor_publisher を戻すまで少し流す
    publish_for(1.0, walls)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
