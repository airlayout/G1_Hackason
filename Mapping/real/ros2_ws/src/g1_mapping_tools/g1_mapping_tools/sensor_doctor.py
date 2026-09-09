"""LiDARとIMUの型・周波数・時刻を短時間だけ観測する。"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, PointCloud2


class SensorDoctor(Node):
    def __init__(
        self,
        points_topic: str,
        imu_topic: str,
        image_topic: str,
        camera_info_topic: str,
    ) -> None:
        super().__init__("g1_sensor_doctor")
        self.points_count = 0
        self.imu_count = 0
        self.point_fields: list[dict[str, object]] = []
        self.point_stamp_first: float | None = None
        self.point_stamp_last: float | None = None
        self.imu_stamp_first: float | None = None
        self.imu_stamp_last: float | None = None
        self.imu_finite = True
        self.image_count = 0
        self.image_stamp_first: float | None = None
        self.image_stamp_last: float | None = None
        self.image_is_jpeg = True
        self.camera_info_count = 0
        self.camera_width = 0
        self.camera_height = 0
        self.camera_calibrated = False
        self.create_subscription(
            PointCloud2, points_topic, self._on_points, qos_profile_sensor_data
        )
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _stamp(message: PointCloud2 | Imu) -> float:
        return message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9

    def _on_points(self, message: PointCloud2) -> None:
        self.points_count += 1
        stamp = self._stamp(message)
        self.point_stamp_first = stamp if self.point_stamp_first is None else self.point_stamp_first
        self.point_stamp_last = stamp
        if not self.point_fields:
            self.point_fields = [
                {
                    "name": field.name,
                    "offset": field.offset,
                    "datatype": field.datatype,
                    "count": field.count,
                }
                for field in message.fields
            ]

    def _on_imu(self, message: Imu) -> None:
        self.imu_count += 1
        stamp = self._stamp(message)
        self.imu_stamp_first = stamp if self.imu_stamp_first is None else self.imu_stamp_first
        self.imu_stamp_last = stamp
        values = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        self.imu_finite = self.imu_finite and all(math.isfinite(value) for value in values)

    def _on_image(self, message: CompressedImage) -> None:
        self.image_count += 1
        stamp = self._stamp(message)
        self.image_stamp_first = stamp if self.image_stamp_first is None else self.image_stamp_first
        self.image_stamp_last = stamp
        self.image_is_jpeg = self.image_is_jpeg and (
            "jpeg" in message.format.lower() and message.data[:2] == b"\xff\xd8"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info_count += 1
        self.camera_width = int(message.width)
        self.camera_height = int(message.height)
        self.camera_calibrated = message.k[0] > 0.0 and message.k[4] > 0.0


def _rate(count: int, first: float | None, last: float | None) -> float:
    if count < 2 or first is None or last is None or last <= first:
        return 0.0
    return (count - 1) / (last - first)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--points-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--image-topic", default="/g1_camera/color/image/compressed"
    )
    parser.add_argument(
        "--camera-info-topic", default="/g1_camera/color/camera_info"
    )
    parser.add_argument("--require-camera", action="store_true")
    arguments = parser.parse_args()

    rclpy.init()
    node = SensorDoctor(
        arguments.points_topic,
        arguments.imu_topic,
        arguments.image_topic,
        arguments.camera_info_topic,
    )
    deadline = time.monotonic() + arguments.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    point_names = {str(field["name"]) for field in node.point_fields}
    required_fields = {"x", "y", "z"}
    timestamp_fields = {"timestamp", "time", "t", "offset_time"}
    points_rate = _rate(
        node.points_count, node.point_stamp_first, node.point_stamp_last
    )
    imu_rate = _rate(node.imu_count, node.imu_stamp_first, node.imu_stamp_last)
    image_rate = _rate(
        node.image_count, node.image_stamp_first, node.image_stamp_last
    )
    stamp_delta = None
    if node.point_stamp_last is not None and node.imu_stamp_last is not None:
        stamp_delta = node.point_stamp_last - node.imu_stamp_last

    errors: list[str] = []
    warnings: list[str] = []
    if node.points_count == 0:
        errors.append("PointCloud2を受信できません")
    if node.imu_count == 0:
        errors.append("Imuを受信できません")
    if not required_fields.issubset(point_names):
        errors.append("PointCloud2にx/y/z fieldsがありません")
    if not timestamp_fields.intersection(point_names):
        warnings.append("各点時刻fieldがありません。時刻推定の縮退モードになります")
    if points_rate and points_rate < 5.0:
        warnings.append(f"点群周波数が低いです: {points_rate:.1f} Hz")
    if imu_rate and imu_rate < 50.0:
        warnings.append(f"IMU周波数が低いです: {imu_rate:.1f} Hz")
    if not node.imu_finite:
        errors.append("IMUにNaNまたはInfがあります")
    if arguments.require_camera and node.image_count == 0:
        errors.append("CompressedImageを受信できません")
    if arguments.require_camera and node.camera_info_count == 0:
        errors.append("CameraInfoを受信できません")
    if node.image_count and not node.image_is_jpeg:
        errors.append("カメラ画像がJPEGとして不正です")
    if node.camera_info_count and not node.camera_calibrated:
        warnings.append("CameraInfoに有効な焦点距離がありません")
    if stamp_delta is not None and abs(stamp_delta) > 0.2:
        warnings.append(f"LiDARとIMUの最新header時刻差が大きいです: {stamp_delta:.3f} s")

    report = {
        "success": not errors,
        "points": {
            "topic": arguments.points_topic,
            "messages": node.points_count,
            "rate_hz": points_rate,
            "fields": node.point_fields,
        },
        "imu": {
            "topic": arguments.imu_topic,
            "messages": node.imu_count,
            "rate_hz": imu_rate,
            "finite": node.imu_finite,
        },
        "latest_stamp_delta_seconds": stamp_delta,
        "camera": {
            "image_topic": arguments.image_topic,
            "camera_info_topic": arguments.camera_info_topic,
            "messages": node.image_count,
            "rate_hz": image_rate,
            "jpeg": node.image_is_jpeg,
            "camera_info_messages": node.camera_info_count,
            "width": node.camera_width,
            "height": node.camera_height,
            "calibrated": node.camera_calibrated,
        },
        "warnings": warnings,
        "errors": errors,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
