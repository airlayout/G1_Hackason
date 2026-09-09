"""内蔵SLAMのodom/points/状態を監視する(購読のみ)。"""
import json, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
rclpy.init(); n = Node("watch_slam")
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
odom, pts, ctrl = [], [], []

def cb_odom(m):
    p = m.pose.pose.position; o = m.pose.pose.orientation
    odom.append((time.time(), m.header.frame_id, m.child_frame_id,
                 p.x, p.y, p.z, o.x, o.y, o.z, o.w))
def cb_pts(m):
    pts.append((time.time(), m.header.frame_id, m.width * m.height))
def cb_ctrl(m):
    try:
        j = json.loads(m.data)
        if j.get("type") == "ctrl_info":
            sm = j.get("data", {}).get("stateMachine", {})
            ctrl.append((sm.get("ctrName"), sm.get("state"), j.get("info")))
    except Exception:
        pass

n.create_subscription(Odometry, "/unitree/slam_mapping/odom", cb_odom, qos)
n.create_subscription(PointCloud2, "/unitree/slam_mapping/points", cb_pts, qos)
n.create_subscription(String, "/slam_info", cb_ctrl, 10)

t0 = time.time()
while rclpy.ok() and time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=1.0)
el = time.time() - t0

print(f"\n===== 監視 {el:.0f} 秒 =====")
print(f"/unitree/slam_mapping/odom  : {len(odom)} 件 ({len(odom)/el:.2f} Hz)")
if odom:
    a = np.array([[r[3], r[4], r[5]] for r in odom])
    print(f"  frame_id='{odom[0][1]}'  child_frame_id='{odom[0][2]}'")
    print(f"  最初の位置 ({a[0,0]:+.4f}, {a[0,1]:+.4f}, {a[0,2]:+.4f})")
    print(f"  最後の位置 ({a[-1,0]:+.4f}, {a[-1,1]:+.4f}, {a[-1,2]:+.4f})")
    d = np.linalg.norm(a - a[0], axis=1)
    print(f"  静止時のドリフト: 最大 {d.max()*100:.1f} cm / 最終 {d[-1]*100:.1f} cm")
print(f"/unitree/slam_mapping/points: {len(pts)} 件 ({len(pts)/el:.2f} Hz)")
if pts:
    w = [r[2] for r in pts]
    print(f"  frame_id='{pts[0][1]}'  点数 中央{int(np.median(w))} 最小{min(w)} 最大{max(w)}")
uniq = sorted(set(ctrl))
print(f"/slam_info ctrl_info: {len(ctrl)} 件 / 状態の種類 {len(uniq)}")
for c in uniq[:6]:
    print(f"  ctrName={c[0]!r} state={c[1]!r} info={c[2]!r}")
