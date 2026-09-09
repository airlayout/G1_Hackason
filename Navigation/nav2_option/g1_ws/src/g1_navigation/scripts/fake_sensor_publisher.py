#!/usr/bin/env python3
"""Nav2の疑似データ動作確認用。実センサーの代わりに空のLaserScan/PointCloud2を配信する。

Planning.md「Nav2設定ファイル下書き+疑似データでの動作確認」に対応。
実際の障害物検出は行わない(常に「何も無い」データを流すだけ)。目的は
costmapの各observation_sourceがタイムアウトせず、Nav2のライフサイクルノードが
正常にactivateして経路計画・制御が動くことを確認すること。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate_hz", 10.0)
        self.frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("rate_hz").value

        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_points = self.create_publisher(PointCloud2, "/g1/points_local", 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info("fake_sensor_publisher 起動(常に空のscan/pointcloudを配信する)")

    def on_timer(self):
        now = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id
        scan.angle_min = -3.14159
        scan.angle_max = 3.14159
        scan.angle_increment = 0.0174533  # 1deg
        scan.range_min = 0.1
        scan.range_max = 30.0
        n = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [float("inf")] * n  # 障害物なし(何も検知しない)
        self.pub_scan.publish(scan)

        cloud = PointCloud2()
        cloud.header.stamp = now
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 0
        cloud.data = b""
        cloud.is_dense = True
        self.pub_points.publish(cloud)


def main():
    rclpy.init()
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
