"""点群を TF で base_link へ変換し、床が z≈0 に来るかを検証する。

これが「上下逆フレーム」が本当に直ったかの最終確認になる。
base_link は床面に置く前提なので、床は z≈0、天井は z≈+2.8 に来るのが正しい。
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

rclpy.init(); n = Node("verify_frame")
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
clouds = []
qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
def cb(m):
    rec = pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)
    clouds.append(np.stack([rec["x"],rec["y"],rec["z"]],axis=-1).astype(np.float64))
n.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360", cb, qos)

t0=time.time()
while rclpy.ok() and time.time()-t0 < 25 and (len(clouds) < 20 or not buf.can_transform("base_link","livox_frame",rclpy.time.Time())):
    rclpy.spin_once(n, timeout_sec=0.3)

tr = buf.lookup_transform("base_link","livox_frame", rclpy.time.Time())
q = tr.transform.rotation; t = tr.transform.translation
x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([
 [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
 [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
 [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
T = np.array([t.x,t.y,t.z])
print(f"base_link<-livox_frame: 並進 ({t.x:+.3f},{t.y:+.3f},{t.z:+.3f})")

p = np.concatenate(clouds, axis=0)
r = np.linalg.norm(p,axis=1)
p = p[(r>0.4)&(r<15)]
b = p @ R.T + T
print(f"点数 {len(b)}  base_link 系での z 範囲: {b[:,2].min():+.2f} .. {b[:,2].max():+.2f}")
h,e = np.histogram(b[:,2], bins=np.arange(-1.0, 4.0, 0.05))
ctr=(e[:-1]+e[1:])/2
top = np.argsort(h)[::-1][:4]
print("\n=== z のピーク上位4(base_link 系) ===")
for i in sorted(top, key=lambda k:-h[k]):
    print(f"  z={ctr[i]:+5.2f} m  {h[i]:>7} 点")
print("\n判定: **床が z≈0.0 に大きなピークを作れば正しい**（base_link は床面に置く前提）")
print("      z≈-1.2 にピークが出るなら並進が未適用、上下が逆なら床が z≈+2.4 付近に出る")
