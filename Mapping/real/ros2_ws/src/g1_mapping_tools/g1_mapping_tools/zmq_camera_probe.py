"""LeRobot camera serverをROS graphへ変更を加えずに診断する。"""

from __future__ import annotations

import argparse
import json
import sys
import time

import zmq

from .zmq_camera_bridge import decode_lerobot_message, jpeg_dimensions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--camera-name", default="head_camera")
    parser.add_argument("--duration", type=float, default=5.0)
    arguments = parser.parse_args()

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt(zmq.RCVTIMEO, 200)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(f"tcp://{arguments.host}:{arguments.port}")
    deadline = time.monotonic() + arguments.duration
    frames = 0
    first_timestamp = None
    last_timestamp = None
    dimensions = None
    errors: list[str] = []
    try:
        while time.monotonic() < deadline:
            try:
                wire_data = socket.recv()
            except zmq.Again:
                continue
            try:
                decoded = decode_lerobot_message(wire_data, arguments.camera_name)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                errors.append(str(error))
                continue
            if decoded is None:
                continue
            jpeg, timestamp_ns = decoded
            dimensions = jpeg_dimensions(jpeg)
            if dimensions is None:
                errors.append("JPEG markerを確認できません")
                continue
            frames += 1
            first_timestamp = timestamp_ns if first_timestamp is None else first_timestamp
            last_timestamp = timestamp_ns
    finally:
        socket.close(linger=0)
        context.term()

    rate = 0.0
    if frames > 1 and first_timestamp is not None and last_timestamp > first_timestamp:
        rate = (frames - 1) / ((last_timestamp - first_timestamp) / 1.0e9)
    report = {
        "success": frames > 0 and dimensions is not None,
        "camera_name": arguments.camera_name,
        "frames": frames,
        "rate_hz": rate,
        "dimensions": dimensions,
        "timestamp_source": "lerobot_camera_capture_realtime",
        "errors": errors[-5:],
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
