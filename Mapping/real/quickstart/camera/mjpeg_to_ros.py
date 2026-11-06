#!/usr/bin/env python3
"""PC2 の `serve_view.py`（頭の IR カメラの MJPEG）を ROS の画像トピックにして RViz2 に出す。

    # コンテナで（live/camera.sh start が起こす）
    python3 mjpeg_to_ros.py --url http://192.168.123.164:8080/stream.mjpg

    [PC2: serve_view.py] --HTTP MJPEG--> [これ（コンテナ）] --/g1_camera/ir/image--> [RViz2]

## なぜこの形なのか

- **PC2 側は実績のある `serve_view.py` をそのまま使う**（09-15 に実機で 9〜28 fps）。
  PC2 で ROS の画像を出すには jammy の Python に cv2 が要り、そちらは確かめていない
- **生の `sensor_msgs/Image`（mono8）で出す。** コンテナには `compressed_image_transport`
  が入っていないので、RViz2 は `CompressedImage` を映せない
- **JPEG は PIL で戻す。** コンテナの cv2 は numpy 2 と噛み合わず import できない
  （`container-scipy-numpy2-mismatch`）。apt の PIL 9.0.1 は numpy なしで動く

## 出すもの

| トピック | 型 | 中身 |
|---|---|---|
| `/g1_camera/ir/image` | `sensor_msgs/Image`（mono8）| RViz2 の Image 表示用 |
| `/g1_camera/ir/image/compressed` | `sensor_msgs/CompressedImage` | 受け取った JPEG そのまま（記録用） |

⚠️ `--max-hz`（既定 10）で間引く。RViz2 は重く、MOLA の頃に測位を痩せさせた（09-11）。
⚠️ 接続が切れたら 2 秒おきに繋ぎ直す（PC2 の再起動・serve_view の再起動に耐える）。
"""
from __future__ import annotations

import argparse
import io
import time
import urllib.request

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

RECONNECT_S = 2.0
READ_TIMEOUT_S = 5.0


def read_frames(url: str):
    """MJPEG（`--boundary` / ヘッダ / Content-Length 分の JPEG）を 1 枚ずつ返す。"""
    with urllib.request.urlopen(url, timeout=READ_TIMEOUT_S) as stream:
        while True:
            line = stream.readline()
            if not line:
                return
            if not line.startswith(b"--"):
                continue
            length = None
            while True:
                header = stream.readline()
                if not header or header in (b"\r\n", b"\n"):
                    break
                name, _, value = header.decode("latin-1").partition(":")
                if name.strip().lower() == "content-length":
                    length = int(value.strip())
            if length is None:
                continue
            data = stream.read(length)
            if len(data) != length:
                return
            yield data


class MjpegBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("g1_ir_mjpeg_bridge")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.image_pub = self.create_publisher(Image, "/g1_camera/ir/image", qos)
        self.jpeg_pub = self.create_publisher(
            CompressedImage, "/g1_camera/ir/image/compressed", qos)
        self.frame_id = args.frame_id
        self.min_period = 1.0 / args.max_hz
        self.last_sent = 0.0
        self.sent = 0

    def publish(self, jpeg: bytes) -> None:
        now = time.monotonic()
        if now - self.last_sent < self.min_period:
            return
        self.last_sent = now
        stamp = self.get_clock().now().to_msg()
        picture = PilImage.open(io.BytesIO(jpeg)).convert("L")
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.width, image.height = picture.size
        image.encoding = "mono8"
        image.step = image.width
        image.data = picture.tobytes()
        self.image_pub.publish(image)
        compressed = CompressedImage()
        compressed.header = image.header
        compressed.format = "jpeg"
        compressed.data = jpeg
        self.jpeg_pub.publish(compressed)
        self.sent += 1
        if self.sent == 1 or self.sent % 300 == 0:
            self.get_logger().info("[cam] %d 枚目 %dx%d" % (self.sent, image.width, image.height))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://192.168.123.164:8080/stream.mjpg")
    parser.add_argument("--max-hz", type=float, default=10.0)
    parser.add_argument("--frame-id", default="head_ir_camera")
    args = parser.parse_args()

    rclpy.init()
    node = MjpegBridge(args)
    node.get_logger().info("[cam] %s を購読する（最大 %.0f Hz）" % (args.url, args.max_hz))
    try:
        while rclpy.ok():
            try:
                for jpeg in read_frames(args.url):
                    node.publish(jpeg)
                    if not rclpy.ok():
                        break
                node.get_logger().warn("[cam] 配信が終わった。繋ぎ直す")
            except OSError as error:
                node.get_logger().warn("[cam] 繋がらない（%s）。%.0f 秒後に繋ぎ直す"
                                       % (error, RECONNECT_S))
            time.sleep(RECONNECT_S)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
