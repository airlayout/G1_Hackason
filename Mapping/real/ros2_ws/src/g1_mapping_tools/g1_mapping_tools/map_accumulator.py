"""地図座標系の登録済み点群を、表示用のボクセル地図へ蓄積する。"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from .density_grid import DENSITY_POINT_STEP, DensityGrid, pack_density_points


class MapAccumulator(Node):
    """増分点群を一定解像度で蓄積し、backend非依存のライブ地図を配信する。"""

    def __init__(self) -> None:
        super().__init__("g1_map_accumulator")
        self._voxel_size = float(self.declare_parameter("voxel_size", 0.05).value)
        self._max_points = int(
            self.declare_parameter("max_points", 2_000_000).value
        )
        self._target_scan_count = int(
            self.declare_parameter("target_scan_count", 10).value
        )
        publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self._global_frame = str(
            self.declare_parameter("global_frame", "map").value
        )
        if publish_hz <= 0.0:
            raise ValueError("publish_hzは0より大きい必要があります")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            PointCloud2, "/g1_mapping/map", map_qos
        )
        self.create_subscription(
            PointCloud2,
            "/g1_mapping/cloud_registered",
            self._on_cloud,
            sensor_qos,
        )
        self.create_timer(1.0 / publish_hz, self._publish)

        self._grid = DensityGrid(
            voxel_size=self._voxel_size,
            max_points=self._max_points,
            target_scan_count=self._target_scan_count,
        )
        self._latest_stamp = None
        self._dirty = False
        self._limit_reported = False
        self.get_logger().info(
            f"accumulate /g1_mapping/cloud_registered -> /g1_mapping/map "
            f"(voxel={self._voxel_size}m, max={self._max_points}, "
            f"density_target={self._target_scan_count} scans)"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        names = {field.name for field in message.fields}
        if not {"x", "y", "z"}.issubset(names):
            self.get_logger().error("登録点群にx/y/z fieldがありません")
            return
        fields = ("x", "y", "z", "intensity") if "intensity" in names else ("x", "y", "z")
        try:
            points = point_cloud2.read_points(
                message, field_names=fields, skip_nans=True
            )
            def normalized_points():
                for point in points:
                    intensity = float(point[3]) if len(fields) == 4 else 0.0
                    yield float(point[0]), float(point[1]), float(point[2]), intensity

            updated = self._grid.integrate_scan(normalized_points())
            if self._grid.dropped_new_voxels and not self._limit_reported:
                self.get_logger().warning(
                    f"ライブ地図がmax_points={self._max_points}へ到達しました"
                )
                self._limit_reported = True
            self._latest_stamp = message.header.stamp
            self._dirty = self._dirty or updated > 0
        except (AssertionError, IndexError, ValueError) as error:
            self.get_logger().error(f"登録点群を読めません: {error}")

    def _publish(self) -> None:
        if not self._dirty or self._latest_stamp is None:
            return
        values = self._grid.snapshot()

        message = PointCloud2()
        message.header.stamp = self._latest_stamp
        message.header.frame_id = self._global_frame
        message.height = 1
        message.width = len(values)
        message.fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in (
                ("x", 0, PointField.FLOAT32),
                ("y", 4, PointField.FLOAT32),
                ("z", 8, PointField.FLOAT32),
                ("intensity", 12, PointField.FLOAT32),
                ("density", 16, PointField.FLOAT32),
                ("hit_count", 20, PointField.UINT32),
                ("scan_count", 24, PointField.UINT32),
            )
        ]
        message.is_bigendian = False
        message.point_step = DENSITY_POINT_STEP
        message.row_step = DENSITY_POINT_STEP * len(values)
        message.data = pack_density_points(values)
        message.is_dense = True
        self._publisher.publish(message)
        self._dirty = False


def main() -> None:
    rclpy.init()
    node = MapAccumulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
