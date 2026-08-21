"""/map の埋まり具合を確認するツール。

地図がどれだけ探索されたかを数値で見る。SLAM の進捗確認に使う。
occupancy grid の値は -1: 未知 / 0: 空き / 100: 障害物。

実行方法:
    source env.sh && python3 src/check_map.py
"""

from __future__ import annotations

import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class MapChecker(Node):
    """/map を 1 回受け取って統計を出すノード。"""

    def __init__(self) -> None:
        super().__init__("map_checker")
        # /map は latched (TRANSIENT_LOCAL) で配信されるため合わせる
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.received = False
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)

    def _on_map(self, msg: OccupancyGrid) -> None:
        """地図を受け取って内訳を表示する。"""
        info = msg.info
        data = msg.data
        total = len(data)
        unknown = sum(1 for v in data if v < 0)
        free = sum(1 for v in data if 0 <= v <= 25)
        occupied = sum(1 for v in data if v > 65)

        width_m = info.width * info.resolution
        height_m = info.height * info.resolution

        print(f"[Map] 大きさ: {info.width} x {info.height} セル "
              f"({width_m:.1f} x {height_m:.1f} m)")
        print(f"[Map] 解像度: {info.resolution:.3f} m/cell")
        print(f"[Map] 原点: ({info.origin.position.x:.2f}, "
              f"{info.origin.position.y:.2f})")
        print(f"[Map] 未知:   {unknown:>8} セル ({unknown / total:.1%})")
        print(f"[Map] 空き:   {free:>8} セル ({free / total:.1%})")
        print(f"[Map] 障害物: {occupied:>8} セル ({occupied / total:.1%})")
        # 探索済み = 未知でない部分。ここが増えていけば巡回が進んでいる。
        explored = total - unknown
        print(f"[Map] 探索済み面積: {explored * info.resolution ** 2:.1f} m2")
        self.received = True


def main() -> None:
    """/map を待って統計を出す。"""
    rclpy.init()
    node = MapChecker()
    # 最大 30 秒待つ
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.received:
            break
    if not node.received:
        print("[NG] /map を受信できませんでした。slam_toolbox が active か確認してください")
        sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
