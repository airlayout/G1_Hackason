"""重力ベクトルをTFで map 系へ変換し、(0,0,-1) に一致するかで二重計上を判定する。"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import tf2_ros

rclpy.init(); n = Node("check_gravity_tf")
buf = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(buf, n)
accs = []
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(Imu, "/utlidar/imu_livox_mid360",
                      lambda m: accs.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]), qos)
t0 = time.time()
while rclpy.ok() and (len(accs) < 200 or not buf.can_transform("map", "livox_frame", rclpy.time.Time())) and time.time()-t0 < 25:
    rclpy.spin_once(n, timeout_sec=0.5)

g = np.array(accs).mean(axis=0); g /= np.linalg.norm(g)
print(f"重力(livox系) = ({g[0]:+.4f}, {g[1]:+.4f}, {g[2]:+.4f})")

def quat_to_R(q):
    x,y,z,w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

for target, src in (("map","livox_frame"), ("map","base_link"), ("base_link","livox_frame")):
    try:
        tr = buf.lookup_transform(target, src, rclpy.time.Time())
    except Exception as e:
        print(f"{target}<-{src}: 取得できない ({e})"); continue
    q = tr.transform.rotation
    R = quat_to_R([q.x,q.y,q.z,q.w])
    gm = R @ g
    ang = math.degrees(math.acos(max(-1,min(1, gm[2]))))  # +z に来れば0°
    print(f"{target}<-{src}: 重力を変換 → ({gm[0]:+.4f}, {gm[1]:+.4f}, {gm[2]:+.4f})  +Z軸からのずれ {ang:.2f}°")
print("\n判定: 'map<-livox_frame' で加速度計の値(上向き)が (0,0,+1) に来れば正しい。")
print("⚠️ 2026-09-09: 当初 (0,0,-1) を正解として検証していたが、それはバグの側を検証していた")
