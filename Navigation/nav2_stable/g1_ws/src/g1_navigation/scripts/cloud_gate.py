#!/usr/bin/env python3
"""点の少ない点群フレームを捨てて中継する（Collision Monitor の前段）。

## なぜ要るのか

G1 内蔵の MID-360 は「満杯のフレーム」と「1点だけのフレーム」を交互に出す
（2026-09-25 実測: `1, 41951, 1, 44255, ...`。発行元は1つ。
findings/collision_ahead_cause_20260925.md §6）。

costmap の voxel layer は格子にマークを**溜める**ので、1点のフレームが来ても
困らない。しかし **Collision Monitor は最新の1フレームしか見ない**
（Humble の `PointCloud::getData` は最後に受け取った1通だけを使う）。
そのまま繋ぐと、1点のフレームを受け取った周期では障害物が「消えて」しまい、
停止と発進を約 5Hz で繰り返すことになる。ここで1点のフレームを捨てる。

📌 中身は一切変えない。受け取ったメッセージをそのまま出し直すだけ。

## 使い方

    ros2 run g1_navigation cloud_gate.py --ros-args \\
        -p input_topic:=/utlidar/cloud_livox_mid360 \\
        -p output_topic:=/g1/cloud_dense -p min_points:=100

⚠️ モック（`fake_sensor_publisher.py`）は**常に 0 点**を出すので、
`min_points` を 0 にしないと何も通らない（launch が backend で切り替える）。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudGate(Node):
    """`min_points` 未満のフレームを捨てて、それ以外をそのまま出し直す。"""

    def __init__(self) -> None:
        super().__init__("cloud_gate")
        self.declare_parameter("input_topic", "/utlidar/cloud_livox_mid360")
        self.declare_parameter("output_topic", "/g1/cloud_dense")
        # 満杯のフレームは約 4 万点、空振りは 1 点。100 はその間のどこでもよい値
        self.declare_parameter("min_points", 100)
        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        self.min_points: int = int(self.get_parameter("min_points").value)

        # ⚠️ 入出力とも best effort（SensorDataQoS）。costmap も Collision Monitor も
        # センサー用 QoS で購読するので、reliable で出しても繋がるが、入力側は
        # best effort にしないと実機の publisher と繋がらない。
        self.pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.on_cloud, qos_profile_sensor_data)

        self.passed = 0
        self.dropped = 0
        self.create_timer(30.0, self.report)
        print(f"[cloud_gate] {input_topic} → {output_topic}（{self.min_points} 点未満は捨てる）")

    def on_cloud(self, msg: PointCloud2) -> None:
        """点数を数えて、足りていれば出し直す。"""
        if msg.width * msg.height < self.min_points:
            self.dropped += 1
            return
        self.passed += 1
        self.pub.publish(msg)

    def report(self) -> None:
        """30 秒ごとに通過数と破棄数を出す。0/0 なら入力が来ていない。"""
        print(f"[cloud_gate] 直近30秒: 通過 {self.passed} / 破棄 {self.dropped}")
        if self.passed == 0:
            self.get_logger().warn("1 フレームも通していない。Collision Monitor は何も見えていない")
        self.passed = 0
        self.dropped = 0


def main() -> None:
    rclpy.init()
    node = CloudGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # ⚠️ finally で shutdown を隠さない（例外を握りつぶさないため。CLAUDE.md の方針）
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
