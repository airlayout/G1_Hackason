#!/usr/bin/env python3
"""生の LiDAR 点群を 1 枚だけ .npy に落とす（読むだけ。何も publish しない）。"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


def fields_xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": ["<f4"] * 3,
                      "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                      "itemsize": msg.point_step})
    flat = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    return np.stack((flat["x"], flat["y"], flat["z"]), axis=1).astype(np.float64)


class Once(Node):
    def __init__(self):
        super().__init__("dump_cloud_once")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360",
                                 self.on_cloud, qos)
        self.done = False

    def on_cloud(self, msg):
        if self.done:
            return
        points = fields_xyz(msg)
        np.save("/tmp/raw_cloud.npy", points)
        print("frame=%s 点数=%d 保存=/tmp/raw_cloud.npy" % (msg.header.frame_id, len(points)),
              flush=True)
        self.done = True


rclpy.init()
node = Once()
for _ in range(200):
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.done:
        break
if not node.done:
    print("点群が来なかった", flush=True)
node.destroy_node()
rclpy.try_shutdown()
