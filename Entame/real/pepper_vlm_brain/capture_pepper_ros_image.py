"""Capture exactly one ROS 2 sensor_msgs/Image frame to disk."""

import argparse
import json
import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState


class LatestFrame(Node):
    def __init__(self, topic: str, output: Path, continuous: bool,
                 joint_state_output: Path | None):
        super().__init__("pepper_one_frame_capture")
        self.output = output
        self.bridge = CvBridge()
        self.continuous = continuous
        self.joint_state_output = joint_state_output
        self.done = False
        self.subscription = self.create_subscription(Image, topic, self.capture, 1)
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self.capture_joints, 1)

    def capture_joints(self, message: JointState):
        if self.joint_state_output is None:
            return
        positions = dict(zip(message.name, message.position))
        if "HeadYaw" not in positions or "HeadPitch" not in positions:
            return
        self.joint_state_output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.joint_state_output.with_suffix(".tmp")
        temporary.write_text(json.dumps({"yaw": positions["HeadYaw"],
                                         "pitch": positions["HeadPitch"]}))
        os.replace(temporary, self.joint_state_output)

    def capture(self, message: Image):
        if self.done:
            return
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_name(self.output.stem + ".tmp" + self.output.suffix)
        if not cv2.imwrite(str(temporary), frame):
            raise RuntimeError(f"Could not save {self.output}")
        os.replace(temporary, self.output)
        if not self.continuous:
            print(f"captured={self.output} width={message.width} height={message.height} encoding={message.encoding}")
            self.done = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/front/image_raw")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--continuous", action="store_true")
    parser.add_argument("--joint-state-output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = LatestFrame(args.topic, args.output, args.continuous, args.joint_state_output)
    try:
        while rclpy.ok() and (args.continuous or not node.done):
            rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if args.continuous or node.done else 1


if __name__ == "__main__":
    raise SystemExit(main())
