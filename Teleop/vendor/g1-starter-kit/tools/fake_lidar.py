#!/usr/bin/env python3
"""実機 LiDAR なしで点群表示を試すためのダミー配信。

ロボットが手元に無くても、RViz の設定・bag の記録と再生・PlotJuggler の操作を
練習できるようにするためのもの。**実機データではない**ので、人に見せるときは
必ずその旨を断ること。

    ./scripts/lidar_view.sh --fake    # 通常はこちら経由で起動される
    python3 tools/fake_lidar.py       # 単体起動（ROS 環境を source 済みのこと）

実機に合わせている点:
  - トピック名 /livox/lidar (PointCloud2) と /livox/imu (sensor_msgs/Imu)
  - frame_id は livox_frame、点群 10Hz / IMU 200Hz
  - 上下逆さま搭載を再現して **逆さまのまま** 出す。
    viz_base への 180 度回転 TF が効いていれば RViz で正立して見える。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField

ROOM = (10.0, 8.0, 2.6)  # 部屋の x, y, 高さ [m]
N_POINTS = 12000


def make_room_points(phase):
    """部屋の壁・床・天井・柱をなぞる点群を作る（livox_frame 基準・上下逆さま）。"""
    rng = np.random.default_rng(int(phase * 100) % 10000)
    w, d, h = ROOM
    n = N_POINTS // 5

    def wall(x0, y0, x1, y1):
        t = rng.random(n)
        return np.column_stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, rng.random(n) * h])

    pts = np.vstack([
        wall(-w / 2, -d / 2, w / 2, -d / 2),
        wall(-w / 2, d / 2, w / 2, d / 2),
        wall(-w / 2, -d / 2, -w / 2, d / 2),
        wall(w / 2, -d / 2, w / 2, d / 2),
        np.column_stack([(rng.random(n) - 0.5) * w, (rng.random(n) - 0.5) * d, rng.random(n) * 0.03]),
    ])

    # ゆっくり回る柱（画面が止まって見えないように）
    m = n // 2
    cx, cy = 2.5 * math.cos(phase), 2.5 * math.sin(phase)
    pillar = np.column_stack([cx + (rng.random(m) - 0.5) * 0.3,
                              cy + (rng.random(m) - 0.5) * 0.3,
                              rng.random(m) * h])
    pts = np.vstack([pts, pillar])

    # センサは床から 1.2m の高さ。さらに上下逆さま搭載を再現（y, z を反転）
    pts[:, 2] -= 1.2
    pts[:, 1] *= -1.0
    pts[:, 2] *= -1.0

    intensity = (40 + 200 * rng.random(len(pts))).astype(np.float32)
    return pts.astype(np.float32), intensity


class FakeLivox(Node):
    def __init__(self):
        super().__init__("fake_livox")
        # 実機ドライバ (lddc.cpp: create_publisher(topic, queue_size)) と同じ
        # デフォルト QoS = Reliable / KeepLast(10) を使う。ここを SENSOR_DATA
        # (Best Effort) にすると Reliable 側の購読者に一切届かず、
        # 「RViz は繋がるのに点が出ない」という実機と違う症状になる。
        self.pc_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", 10)
        self.create_timer(0.1, self.publish_cloud)        # 10 Hz
        self.create_timer(0.005, self.publish_imu)        # 200 Hz
        self.t = 0.0
        self.get_logger().info(
            "ダミー配信中: /livox/lidar (10Hz) + /livox/imu (200Hz) frame=livox_frame")

    def publish_cloud(self):
        self.t += 0.1
        xyz, intensity = make_room_points(self.t * 0.3)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "livox_frame"
        msg.height = 1
        msg.width = len(xyz)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.column_stack([xyz, intensity]).astype(np.float32).tobytes()
        self.pc_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "livox_frame"
        # それらしく揺れる値（歩行中の振動を模した正弦 + ノイズ）
        t = self.get_clock().now().nanoseconds * 1e-9
        n = np.random.normal(0, 0.02, 6)
        msg.angular_velocity.x = 0.15 * math.sin(2 * math.pi * 1.2 * t) + n[0]
        msg.angular_velocity.y = 0.10 * math.sin(2 * math.pi * 0.8 * t + 1.0) + n[1]
        msg.angular_velocity.z = 0.05 * math.sin(2 * math.pi * 0.5 * t + 2.0) + n[2]
        msg.linear_acceleration.x = 0.3 * math.sin(2 * math.pi * 2.0 * t) + n[3]
        msg.linear_acceleration.y = 0.3 * math.sin(2 * math.pi * 1.7 * t + 0.5) + n[4]
        msg.linear_acceleration.z = -9.81 + 0.5 * math.sin(2 * math.pi * 2.4 * t) + n[5]
        msg.orientation_covariance[0] = -1.0  # 姿勢は未提供の意味
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLivox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
