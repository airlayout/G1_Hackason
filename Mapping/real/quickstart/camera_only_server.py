#!/usr/bin/env python3
"""G1 PC2でLeRobotの画像serverだけを起動する。"""

from __future__ import annotations

import argparse
import base64

import cv2
import lerobot.cameras.zmq.image_server as image_server


def _encode_rgb_image(image, quality: int = 80) -> str:
    """OpenCVCameraのRGB配列を色順を保ったJPEGへ変換する。"""

    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    success, buffer = cv2.imencode(
        ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    )
    if not success:
        raise RuntimeError("JPEG encodeに失敗しました")
    return base64.b64encode(buffer).decode("ascii")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=4)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--camera-name", default="head_camera")
    arguments = parser.parse_args()
    config = {
        "fps": arguments.fps,
        "cameras": {
            arguments.camera_name: {
                "device_id": arguments.device,
                "shape": [arguments.height, arguments.width],
            }
        },
    }
    # LeRobot ImageServerはOpenCVCameraをRGBで読みながらcv2.imencodeへそのまま
    # 渡すため、標準JPEG viewerではR/Bが反転する。camera-only serverでは補正する。
    image_server.encode_image = _encode_rgb_image
    image_server.ImageServer(config, port=arguments.port).run()


if __name__ == "__main__":
    main()
