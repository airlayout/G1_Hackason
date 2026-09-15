"""LocalCostmap の障害物セルを数える(購読のみ)。

obstacle_min_range を入れる前後で比較するためのもの。
機体の footprint(0.5m x 0.4m)の内側に lethal があると Nav2 は経路を出せない。
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("count_costmap")
buf = Buffer()
TransformListener(buf, node)
grids = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: grids.append(m), qos)
for _ in range(80):
    if grids and buf.can_transform("odom", "base_link", rclpy.time.Time()):
        break
    rclpy.spin_once(node, timeout_sec=0.5)
if not grids:
    raise SystemExit("/local_costmap/costmap が来ない")

g = grids[-1]
a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
res = g.info.resolution
tf = buf.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
bx, by = tf.translation.x, tf.translation.y

# ⚠️ /local_costmap/costmap は OccupancyGrid なので 0〜100(-1=未知)。
# 生の costmap 値(0〜255)ではない。100=lethal / 99=inscribed / -1=未知。
lethal = int((a >= 99).sum())
inflated = int(((a > 0) & (a < 99)).sum())
unknown = int((a < 0).sum())
print(f"local_costmap {g.info.width}x{g.info.height} @{res}m  原点=({g.info.origin.position.x:.2f},{g.info.origin.position.y:.2f})")
print(f"  lethal(253以上) = {lethal} セル ({100*lethal/a.size:.1f}%)")
print(f"  inflation(1〜98) = {inflated} セル ({100*inflated/a.size:.1f}%)")
print(f"  未知(-1) = {unknown} セル ({100*unknown/a.size:.1f}%)")

# 機体まわりの半径ごとの lethal 数
ys, xs = np.nonzero(a >= 99)
if lethal:
    wx = g.info.origin.position.x + (xs + 0.5) * res
    wy = g.info.origin.position.y + (ys + 0.5) * res
    d = np.hypot(wx - bx, wy - by)
    print(f"  機体からの距離: 最短 {d.min():.2f} m / 中央 {np.median(d):.2f} m")
    for lo, hi in ((0, 0.3), (0.3, 0.6), (0.6, 0.9), (0.9, 1.5), (1.5, 3.0)):
        n = int(((d >= lo) & (d < hi)).sum())
        print(f"    {lo:.1f}〜{hi:.1f} m : {n:5d} セル {'#' * min(50, n // 4)}")
    inside = int((d < 0.32).sum())   # footprint の外接円(0.25,0.20)
    print(f"  ⚠️ footprint の内側(0.32m 以内)の lethal = {inside} セル"
          f"{'  ← ここが 0 でないと経路が出ない' if inside else '  ← 0。経路計画できる'}")
