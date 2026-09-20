"""LocalCostmap が機体の周りを塗っている理由を切り分ける(購読のみ)。

床の水平が出ていないなら「全方位に一様なリング」として出る。
人や物が近くに居るなら「特定の方位に固まる」。この2つを区別する。
"""
import math

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

MIN_H, MAX_H = 0.05, 1.8   # nav2_params.yaml の voxel_layer と同じ
MAX_R = 5.0                # obstacle_max_range と同じ

rclpy.init()
node = Node("why_costmap")
buf = Buffer()
TransformListener(buf, node)
clouds = []
qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360",
                         lambda m: clouds.append(m), qos)
while rclpy.ok() and len(clouds) < 12:
    rclpy.spin_once(node, timeout_sec=5.0)

msg = clouds[-1]
# TF は購読を張ってから溜まるまで少し待つ(静的TFの再送を拾う)
for _ in range(60):
    if buf.can_transform("odom", msg.header.frame_id, rclpy.time.Time()):
        break
    rclpy.spin_once(node, timeout_sec=0.5)
else:
    raise SystemExit("odom<-点群 の TF が来ない")
tf = buf.lookup_transform("odom", msg.header.frame_id, rclpy.time.Time())
t = tf.transform.translation
q = tf.transform.rotation
# クォータニオン → 回転行列
x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])
rec = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
p = np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float64)
p = p @ R.T + np.array([t.x, t.y, t.z])
print(f"odom<-{msg.header.frame_id}: 並進 z={t.z:+.3f} m")

r = np.linalg.norm(p[:, :2], axis=1)
near = p[r < MAX_R]
rn = r[r < MAX_R]
print(f"5m以内 {len(near)} 点")
print("z のヒストグラム(床が水平なら 0 付近に鋭い山が1本):")
hist, edges = np.histogram(near[:, 2], bins=np.arange(-0.4, 2.0, 0.1))
for h, e in zip(hist, edges):
    if h:
        print(f"  z {e:+.1f}〜{e + 0.1:+.1f} m : {h:7d} {'#' * min(60, h // 400)}")

band = (near[:, 2] > MIN_H) & (near[:, 2] < MAX_H)
obs = near[band]
print(f"\n障害物として立つ点(z {MIN_H}〜{MAX_H} m) = {len(obs)} 点 / {len(near)} 点 "
      f"({100 * len(obs) / max(1, len(near)):.1f}%)")
if len(obs):
    az = np.degrees(np.arctan2(obs[:, 1] - t.y, obs[:, 0] - t.x))
    ro = np.linalg.norm(obs[:, :2] - np.array([t.x, t.y]), axis=1)
    print("  方位ごとの分布(一様ならリング=水平の狂い / 偏っていれば人や物):")
    for lo in range(-180, 180, 30):
        m = (az >= lo) & (az < lo + 30)
        if m.sum():
            print(f"    {lo:+4d}〜{lo + 30:+4d}° : {m.sum():6d} 点  最短 {ro[m].min():.2f} m")
    print(f"  障害物点の最短距離 = {ro.min():.2f} m")
print(f"\n参考: 5m以内の最短水平距離 = {rn.min():.2f} m(死角の半径。立位の実測は 0.91〜1.12 m)")
