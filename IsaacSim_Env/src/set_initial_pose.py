"""AMCL の初期位置を Isaac Sim の真値から設定する。

Nav2 は起動時に自己位置を知らないため、人が RViz の「2D Pose Estimate」で
教える必要がある。しかしこの地図は原点が (-58, -53) にあり、画像上のどこが
どの座標か直感的に分からないため、クリックでは大きくずれる
（実測で位置 31 m / 向き 179 度のずれが発生した）。

Isaac Sim は真値の姿勢を持っているので、それをそのまま渡せば確実。

**向きも必ず渡すこと。** 位置だけ合わせて向きを yaw=0 に固定すると、
AMCL は「前を向いている」と誤認し、実際に真後ろを向いていても
「目標へ向くために回れ」と指令し続けて永久に旋回する（実測）。

実行方法:
    source env.sh && python3 src/set_initial_pose.py
"""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# 初期姿勢の不確かさ。真値なので小さくてよい。
# [x, y, z, roll, pitch, yaw] の対角成分のみ設定する。
COV_XY: float = 0.05
COV_YAW: float = 0.02


class InitialPoseSetter(Node):
    """/odom の真値を /initialpose として送るノード。"""

    def __init__(self) -> None:
        super().__init__("set_initial_pose")
        self.odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

    def _on_odom(self, msg: Odometry) -> None:
        """真値を保持する。"""
        self.odom = msg

    def publish(self) -> tuple[float, float, float]:
        """保持した真値を初期姿勢として配信する。

        Returns:
            (x, y, yaw[deg]) 設定した姿勢
        """
        assert self.odom is not None
        src = self.odom.pose.pose

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = src.position.x
        msg.pose.pose.position.y = src.position.y
        msg.pose.pose.position.z = 0.0
        # 向きをそのまま渡す。ここを省くと AMCL が向きを誤認する。
        msg.pose.pose.orientation = src.orientation

        cov = [0.0] * 36
        cov[0] = COV_XY   # x
        cov[7] = COV_XY   # y
        cov[35] = COV_YAW  # yaw
        msg.pose.covariance = cov

        self._pub.publish(msg)

        q = src.orientation
        yaw = math.degrees(
            math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        )
        return src.position.x, src.position.y, yaw


def main() -> None:
    """真値を待って初期姿勢を送る。"""
    rclpy.init()
    node = InitialPoseSetter()

    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is not None:
            break

    if node.odom is None:
        print("[NG] /odom を受信できません。Isaac Sim が動いているか確認してください")
        sys.exit(1)

    # 購読側（AMCL）が接続するのを待ってから複数回送る。
    # 1 回だけだと接続前に投げてしまい届かないことがある。
    for _ in range(5):
        x, y, yaw = node.publish()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)

    print(f"[OK] 初期姿勢を設定しました: ({x:+.2f}, {y:+.2f}) yaw={yaw:+.1f} 度")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
