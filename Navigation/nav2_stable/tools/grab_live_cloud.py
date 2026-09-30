"""生点群を数スキャン取得して .npy に落とす(購読のみ)。"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10
out = sys.argv[2] if len(sys.argv) > 2 else "/work/cloud.npy"

rclpy.init()
node = Node("grab_cloud")
scans = []

def cb(msg):
    # フィールドの型が混在(x/y/z=float32、intensity/tag/line=uint8等)するため
    # read_points_numpy は使えない。構造化配列で受けて x/y/z だけ取り出す
    rec = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float32)
    scans.append(xyz)
    node.get_logger().info(f"{len(scans)}/{N} 取得 ({xyz.shape[0]}点) frame={msg.header.frame_id}")

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360", cb, qos)
while rclpy.ok() and len(scans) < N:
    rclpy.spin_once(node, timeout_sec=5.0)
    if not scans and len(scans) == 0:
        pass
if scans:
    np.save(out, np.concatenate(scans, axis=0))
    print(f"[OK] {out} に {sum(s.shape[0] for s in scans)} 点を保存({len(scans)}スキャン)")
else:
    print("[NG] 1スキャンも取れなかった")
