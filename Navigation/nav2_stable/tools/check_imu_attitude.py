import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
rclpy.init(); n=Node("imu_now"); a=[]
qos=QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(Imu,"/utlidar/imu_livox_mid360",
  lambda m: a.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]), qos)
t0=time.time()
while rclpy.ok() and (len(a)<300 and time.time()-t0<15): rclpy.spin_once(n, timeout_sec=0.5)
g=np.array(a).mean(axis=0); nrm=np.linalg.norm(g); gh=g/nrm
tilt=math.degrees(math.acos(max(-1,min(1,-gh[2]))))
print(f"サンプル {len(a)} 件")
print(f"重力 = ({g[0]:+.4f}, {g[1]:+.4f}, {g[2]:+.4f})  ノルム {nrm:.4f}")
print(f"**-Z軸からの傾き = {tilt:.2f}°**")
# 傾きの向き(どちらに倒れているか)
if tilt > 1:
    az = math.degrees(math.atan2(gh[1], gh[0]))
    print(f"傾きの方位 = {az:+.1f}° (センサーX軸を0°として、重力の水平成分の向き)")
    print(f"  → センサーの上向き軸(+Z)は、方位 {az+180 if az<0 else az-180:+.1f}° の側へ {tilt:.2f}° 倒れている")
