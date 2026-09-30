"""静止状態の点群とIMU重力ベクトルを取得する(購読のみ)。

重力で水平化すれば、ロボットの姿勢(座位/直立)に依存せず地図と同じ向きに揃えられる。
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2

N_SCAN = int(sys.argv[1]) if len(sys.argv) > 1 else 60
OUT = sys.argv[2] if len(sys.argv) > 2 else "/work/sit"

rclpy.init()
node = Node("grab_cloud_imu")
scans, accs, gyros = [], [], []

def cb_cloud(msg):
    rec = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    scans.append(np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float32))

def cb_imu(msg):
    a = msg.linear_acceleration
    g = msg.angular_velocity
    accs.append([a.x, a.y, a.z])
    gyros.append([g.x, g.y, g.z])

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360", cb_cloud, qos)
node.create_subscription(Imu, "/utlidar/imu_livox_mid360", cb_imu, qos)

while rclpy.ok() and len(scans) < N_SCAN:
    rclpy.spin_once(node, timeout_sec=5.0)

pts = np.concatenate(scans, axis=0)
acc = np.asarray(accs); gyro = np.asarray(gyros)
np.save(f"{OUT}_cloud.npy", pts)
np.save(f"{OUT}_acc.npy", acc)
print(f"[OK] {len(scans)}スキャン {pts.shape[0]}点 / IMU {len(accs)}サンプル")
print(f"     重力(平均) = ({acc[:,0].mean():+.4f}, {acc[:,1].mean():+.4f}, {acc[:,2].mean():+.4f})  ノルム {np.linalg.norm(acc.mean(axis=0)):.4f}")
print(f"     重力の標準偏差 = ({acc[:,0].std():.4f}, {acc[:,1].std():.4f}, {acc[:,2].std():.4f})")
print(f"     角速度(平均) = ({gyro[:,0].mean():+.4f}, {gyro[:,1].mean():+.4f}, {gyro[:,2].mean():+.4f}) rad/s  → 静止しているか確認")
