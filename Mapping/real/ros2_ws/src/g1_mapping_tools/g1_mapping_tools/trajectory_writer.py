"""nav_msgs/OdometryをTUM trajectory形式へ逐次保存する。"""

from __future__ import annotations

from pathlib import Path
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TrajectoryWriter(Node):
    def __init__(self) -> None:
        super().__init__("g1_trajectory_writer")
        output_path = Path(
            self.declare_parameter(
                "output_path", "/runs/unknown/trajectory/trajectory.tum"
            ).value
        )
        odom_topic = str(self.declare_parameter("odom_topic", "/g1_mapping/odom").value)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open("a", encoding="utf-8", buffering=1)
        if output_path.stat().st_size == 0:
            self._stream.write("# timestamp tx ty tz qx qy qz qw\n")
        self._lock = threading.Lock()
        self.create_subscription(
            Odometry, odom_topic, self._on_odometry, qos_profile_sensor_data
        )
        self.get_logger().info(f"trajectory: {odom_topic} -> {output_path}")

    def _on_odometry(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        line = (
            f"{stamp:.9f} {position.x:.9f} {position.y:.9f} {position.z:.9f} "
            f"{orientation.x:.9f} {orientation.y:.9f} "
            f"{orientation.z:.9f} {orientation.w:.9f}\n"
        )
        with self._lock:
            self._stream.write(line)

    def destroy_node(self) -> bool:
        with self._lock:
            self._stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TrajectoryWriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
