#!/usr/bin/env python3
"""mapping 実行時の軌跡(TUM形式)を nav_msgs/Path として配信する。

`view_map_rviz.sh` から使う。地図の自由空間の上にロボットが実際に歩いた経路が
乗っているかを RViz で目視確認するためのもの(A-7 の連結性レポートの視覚版)。

    python3 publish_trajectory_path.py <trajectory.tum> [--frame map] [--topic /mapping_trajectory]

Transient Local で 1 回だけ配信し続けるので、後から RViz を開いても表示される。
"""
from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_tum(path: str) -> list[tuple[float, float, float, tuple[float, float, float, float]]]:
    poses = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                quat = (float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]))
            except ValueError:
                continue
            poses.append((x, y, z, quat))
    return poses


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--topic", default="/mapping_trajectory")
    args = parser.parse_args()

    poses = load_tum(args.trajectory)
    if not poses:
        raise SystemExit(f"軌跡を1件も読めなかった: {args.trajectory}")

    rclpy.init()
    node = Node("mapping_trajectory_publisher")
    # RViz を後から開いても見えるように Transient Local で保持する
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Path, args.topic, qos)

    msg = Path()
    msg.header.frame_id = args.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    for x, y, z, quat in poses:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x, ps.pose.orientation.y = quat[0], quat[1]
        ps.pose.orientation.z, ps.pose.orientation.w = quat[2], quat[3]
        msg.poses.append(ps)

    pub.publish(msg)
    node.get_logger().info(f"{len(msg.poses)} 姿勢を {args.topic} ({args.frame}) へ配信した")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
