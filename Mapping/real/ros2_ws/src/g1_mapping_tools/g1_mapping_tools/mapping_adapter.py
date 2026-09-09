"""Backend固有のLIO出力をG1 Mappingの共通契約へ正規化する。"""

from __future__ import annotations

from collections import deque
import copy

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


def _stamp_is_zero(message: PointCloud2) -> bool:
    return message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0


class MappingAdapter(Node):
    """1つのpose providerを共通odom・登録点群・TF・Pathへ変換する。"""

    def __init__(self) -> None:
        super().__init__("g1_mapping_adapter")
        source_odom = str(
            self.declare_parameter("source_odom", "/unitree/slam_mapping/odom").value
        )
        source_cloud = str(
            self.declare_parameter(
                "source_cloud", "/unitree/slam_mapping/points"
            ).value
        )
        self._global_frame = str(
            self.declare_parameter("global_frame", "map").value
        )
        self._fallback_child_frame = str(
            self.declare_parameter("fallback_child_frame", "base_link").value
        )
        self._path_limit = int(self.declare_parameter("path_limit", 5000).value)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._odom_pub = self.create_publisher(Odometry, "/g1_mapping/odom", 20)
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/g1_mapping/cloud_registered", sensor_qos
        )
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(Path, "/g1_mapping/path", path_qos)
        self._tf = TransformBroadcaster(self)
        self._poses: deque[PoseStamped] = deque(maxlen=self._path_limit)
        self._latest_odom: Odometry | None = None
        self._restamped_clouds = 0

        self.create_subscription(Odometry, source_odom, self._on_odom, sensor_qos)
        self.create_subscription(
            PointCloud2, source_cloud, self._on_cloud, sensor_qos
        )
        self.get_logger().info(
            f"normalize odom={source_odom}, cloud={source_cloud} -> /g1_mapping/*"
        )

    def _on_odom(self, source: Odometry) -> None:
        message = copy.deepcopy(source)
        message.header.frame_id = self._global_frame
        if not message.child_frame_id:
            message.child_frame_id = self._fallback_child_frame
        self._latest_odom = message
        self._odom_pub.publish(message)

        transform = TransformStamped()
        transform.header = copy.deepcopy(message.header)
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._tf.sendTransform(transform)

        pose = PoseStamped()
        pose.header = copy.deepcopy(message.header)
        pose.pose = copy.deepcopy(message.pose.pose)
        self._poses.append(pose)
        path = Path()
        path.header = copy.deepcopy(message.header)
        path.poses = list(self._poses)
        self._path_pub.publish(path)

    def _on_cloud(self, source: PointCloud2) -> None:
        message = copy.deepcopy(source)
        message.header.frame_id = self._global_frame
        if _stamp_is_zero(message):
            if self._latest_odom is None:
                self.get_logger().warning(
                    "登録点群のstampが0で、対応するodometryもまだ無いため破棄します"
                )
                return
            message.header.stamp = copy.deepcopy(self._latest_odom.header.stamp)
            self._restamped_clouds += 1
            if self._restamped_clouds == 1:
                self.get_logger().warning(
                    "登録点群のstampが0のため、直近odometryのstampで補正します"
                )
        self._cloud_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = MappingAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
