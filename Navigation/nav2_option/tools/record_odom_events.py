"""U-10: odometry の符号・単位を確認する(購読のみ)。

内蔵SLAM の odom を記録し、動いた区間ごとに
「x/y の変位」と「yaw の変化」を報告する。
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/work/u10_odom.npy"

def yaw_of(o):
    return math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))

rclpy.init(); n = Node("record_u10"); rows = []
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(Odometry, "/unitree/slam_mapping/odom",
    lambda m: rows.append([time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                           m.pose.pose.position.z, yaw_of(m.pose.pose.orientation)]), qos)

print(f"[rec] {DUR:.0f} 秒記録する", flush=True)
t0 = time.time(); last = 0.0
while rclpy.ok() and time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.2)
    el = time.time() - t0
    if el - last >= 3.0 and len(rows) > 5:
        a = np.array(rows)
        print(f"[rec] t={el:5.1f}s  x={a[-1,1]:+7.3f} y={a[-1,2]:+7.3f} "
              f"yaw={math.degrees(a[-1,4]):+7.2f}°", flush=True)
        last = el

a = np.array(rows); np.save(OUT, a)
print(f"\n[rec] {len(a)} サンプル -> {OUT}", flush=True)

# 動いた区間を検出して報告
t = a[:,0] - a[0,0]
v = np.zeros(len(a))
for i in range(2, len(a)):
    dt = t[i] - t[i-2]
    if dt > 0:
        v[i] = math.hypot(a[i,1]-a[i-2,1], a[i,2]-a[i-2,2]) / dt
w = np.zeros(len(a))
for i in range(2, len(a)):
    dt = t[i] - t[i-2]
    if dt > 0:
        d = a[i,4] - a[i-2,4]
        d = (d + math.pi) % (2*math.pi) - math.pi
        w[i] = d / dt
mv = (v > 0.05) | (np.abs(w) > 0.10)
if not mv.any():
    print("動いた区間は検出されなかった"); raise SystemExit(0)
idx = np.where(mv)[0]
groups = np.split(idx, np.where(np.diff(t[idx]) > 2.5)[0] + 1)
print(f"\n=== 動いた区間: {len(groups)} 個 ===")
for gi, gr in enumerate(groups):
    i0, i1 = gr[0], gr[-1]
    dx, dy = a[i1,1]-a[i0,1], a[i1,2]-a[i0,2]
    dyaw = (a[i1,4]-a[i0,4] + math.pi) % (2*math.pi) - math.pi
    dist = math.hypot(dx, dy)
    dur = t[i1]-t[i0]
    # 移動方向が開始時の機体前方(yaw)からどれだけずれているか
    head = math.degrees((math.atan2(dy, dx) - a[i0,4] + math.pi) % (2*math.pi) - math.pi) if dist > 0.02 else float('nan')
    print(f"\n  区間{gi+1}: t={t[i0]:.2f}〜{t[i1]:.2f}s (継続 {dur:.2f}s)")
    print(f"    Δx={dx:+.3f} m  Δy={dy:+.3f} m  距離 {dist:.3f} m  平均速度 {dist/max(dur,1e-3):.3f} m/s")
    print(f"    Δyaw={math.degrees(dyaw):+.2f}°  平均角速度 {math.degrees(dyaw)/max(dur,1e-3):+.2f}°/s "
          f"({dyaw/max(dur,1e-3):+.3f} rad/s)")
    if not math.isnan(head):
        print(f"    移動方向は開始時の機体前方から {head:+.1f}° ずれている(0°=真っ直ぐ前進)")
