#!/usr/bin/env python3
"""G1 の地図点群に時刻を打ち直して再配信する。Humble 環境（~/g1_humble）で動かす。

## なぜ必要か

`/unitree/slam_mapping/points` は **header.stamp が 0** で配信されている
（2026-09-03に確認。CDR直読とros2 topic echoの両方で一致）。

    /unitree/slam_mapping/points   stamp: sec=0           <- これ
    /utlidar/cloud_livox_mid360    stamp: sec=1788406334  正常
    /unitree/slam_mapping/odom     stamp: sec=1788406335  正常

同じSLAMが出すodomには正しい時刻が入っているので、点群だけの不具合とみられる。
Foxglove/Lichtblick は「そのメッセージの時刻におけるTF」を引いて描画するため、
時刻0のメッセージは遥か過去とみなされ**描画されない**。

このノードは受信時刻を打ち直して別トピックへ流す。ビューアではこちらを見る。

    ssh g1 'bash ~/mapping_tools/start_restamp.sh'
    -> /unitree/slam_mapping/points_stamped

## 打ち直しの正しさについて

受信時刻で代用しているだけなので、**真の観測時刻ではない**（DDSの伝送遅延ぶん遅れる）。
ただしこの点群は既に`map`座標系へ変換済みで、表示時にTFを掛けないため、
時刻のずれが点の位置に影響することはない。

**時間的な厳密さが要る用途（他センサとの同期、後処理でのLIO再計算）には使わないこと。**
その場合は生LiDAR`/utlidar/cloud_livox_mid360`を使う。こちらは各点に`time`まで付いている。
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Restamp(Node):
    def __init__(self, src: str, dst: str, force: bool) -> None:
        super().__init__("restamp_points")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PointCloud2, dst, qos)
        self.create_subscription(PointCloud2, src, self._on_cloud, qos)
        self._force = force
        self._count = 0
        self._nonzero = 0
        self.get_logger().info("[restamp] {} -> {} （時刻を打ち直す）".format(src, dst))

    def _on_cloud(self, msg: PointCloud2) -> None:
        original_zero = (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0)
        if not original_zero:
            self._nonzero += 1
            if not self._force:
                # 元から時刻が入っているなら触らない（G1側が直った場合に備える）
                self._pub.publish(msg)
                self._report()
                return
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)
        self._report()

    def _report(self) -> None:
        self._count += 1
        if self._count in (1, 10) or self._count % 200 == 0:
            self.get_logger().info(
                "[restamp] {} 件を再配信（うち元から時刻ありだったもの {} 件）".format(
                    self._count, self._nonzero))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", default="/unitree/slam_mapping/points")
    p.add_argument("--dst", default="/unitree/slam_mapping/points_stamped")
    p.add_argument("--force", action="store_true",
                   help="元から時刻が入っていても打ち直す")
    args = p.parse_args(argv)

    rclpy.init()
    node = Restamp(args.src, args.dst, args.force)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[restamp] 停止します")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
