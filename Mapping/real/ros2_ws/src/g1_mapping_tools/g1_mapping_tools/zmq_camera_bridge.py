"""LeRobot公式G1 camera serverを標準ROSカメラトピックへ変換する。"""

from __future__ import annotations

import base64
import json
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
import zmq


def jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """JPEGのSOF markerから(width, height)を取得する。"""
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return None
    index = 2
    sof = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while index + 4 <= len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        while index < len(data) and data[index] == 0xFF:
            index += 1
        if index >= len(data):
            break
        marker = data[index]
        index += 1
        if marker in {0xD8, 0xD9}:
            continue
        if index + 2 > len(data):
            break
        length = int.from_bytes(data[index : index + 2], "big")
        if length < 2 or index + length > len(data):
            break
        if marker in sof and length >= 7:
            height = int.from_bytes(data[index + 3 : index + 5], "big")
            width = int.from_bytes(data[index + 5 : index + 7], "big")
            return width, height
        index += length
    return None


def decode_lerobot_message(
    wire_data: bytes, camera_name: str
) -> tuple[bytes, int] | None:
    """LeRobot ImageServer JSONからJPEGと撮影時刻nsを取り出す。"""
    payload = json.loads(wire_data.decode("utf-8"))
    images = payload.get("images", {})
    timestamps = payload.get("timestamps", {})
    if camera_name not in images or camera_name not in timestamps:
        return None
    jpeg = base64.b64decode(images[camera_name], validate=True)
    capture_ns = int(float(timestamps[camera_name]) * 1_000_000_000)
    return jpeg, capture_ns


class ZmqCameraBridge(Node):
    """撮影元timestampを保持したままLeRobot画像をROSへ橋渡しする。"""

    def __init__(self) -> None:
        super().__init__("g1_zmq_camera_bridge")
        host = str(self.declare_parameter("host", "192.168.123.164").value)
        port = int(self.declare_parameter("port", 5555).value)
        self._camera_name = str(
            self.declare_parameter("camera_name", "head_camera").value
        )
        self._frame_id = str(
            self.declare_parameter("frame_id", "camera_color_optical_frame").value
        )
        self._width = int(self.declare_parameter("width", 0).value)
        self._height = int(self.declare_parameter("height", 0).value)
        self._fx = float(self.declare_parameter("fx", 0.0).value)
        self._fy = float(self.declare_parameter("fy", 0.0).value)
        self._cx = float(self.declare_parameter("cx", 0.0).value)
        self._cy = float(self.declare_parameter("cy", 0.0).value)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._image_pub = self.create_publisher(
            CompressedImage, "/g1_camera/color/image/compressed", sensor_qos
        )
        self._info_pub = self.create_publisher(
            CameraInfo, "/g1_camera/color/camera_info", info_qos
        )
        self._metadata_pub = self.create_publisher(
            String, "/g1_camera/frame_metadata", sensor_qos
        )

        self._zmq_context = zmq.Context.instance()
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self._zmq_socket.setsockopt(zmq.RCVHWM, 1)
        self._zmq_socket.setsockopt(zmq.LINGER, 0)
        self._zmq_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._zmq_socket.connect(f"tcp://{host}:{port}")
        self._sequence = 0
        self._last_frame_monotonic = time.monotonic()
        self._stale_reported = False
        self.create_timer(0.002, self._poll)
        self.create_timer(1.0, self._watchdog)
        self.get_logger().info(
            f"LeRobot camera {self._camera_name}@tcp://{host}:{port} を購読します"
        )

    def _poll(self) -> None:
        try:
            wire_data = self._zmq_socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        try:
            decoded = decode_lerobot_message(wire_data, self._camera_name)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            self.get_logger().warning(f"カメラmessageを解釈できません: {error}")
            return
        if decoded is None:
            return
        jpeg, capture_ns = decoded
        dimensions = jpeg_dimensions(jpeg)
        if dimensions is None:
            self.get_logger().warning("カメラpayloadがJPEGではありません")
            return
        width, height = dimensions
        stamp = Time()
        stamp.sec = capture_ns // 1_000_000_000
        stamp.nanosec = capture_ns % 1_000_000_000

        image = CompressedImage()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.format = "rgb8; jpeg compressed rgb8"
        image.data = jpeg
        self._image_pub.publish(image)

        info = self._camera_info(width, height)
        info.header = image.header
        self._info_pub.publish(info)

        metadata = String()
        metadata.data = json.dumps(
            {
                "schema_version": 1,
                "sequence": self._sequence,
                "stamp_ns": capture_ns,
                "timestamp_source": "lerobot_camera_capture_realtime",
                "camera_name": self._camera_name,
                "width": width,
                "height": height,
                "calibration_complete": self._fx > 0.0 and self._fy > 0.0,
            },
            separators=(",", ":"),
        )
        self._metadata_pub.publish(metadata)
        self._sequence += 1
        self._last_frame_monotonic = time.monotonic()
        if self._stale_reported:
            self.get_logger().info("カメラフレームの受信が復旧しました")
            self._stale_reported = False

    def _camera_info(self, width: int, height: int) -> CameraInfo:
        if self._width and width != self._width:
            self.get_logger().warning(f"画像幅{width} != 設定値{self._width}")
        if self._height and height != self._height:
            self.get_logger().warning(f"画像高{height} != 設定値{self._height}")
        message = CameraInfo()
        message.width = width
        message.height = height
        message.distortion_model = "plumb_bob"
        message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.k = [self._fx, 0.0, self._cx, 0.0, self._fy, self._cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            self._fx, 0.0, self._cx, 0.0,
            0.0, self._fy, self._cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def _watchdog(self) -> None:
        if not self._stale_reported and time.monotonic() - self._last_frame_monotonic > 3.0:
            self.get_logger().warning("camera serverから3秒以上フレームを受信していません")
            self._stale_reported = True

    def destroy_node(self) -> bool:
        self._zmq_socket.close(linger=0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ZmqCameraBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
