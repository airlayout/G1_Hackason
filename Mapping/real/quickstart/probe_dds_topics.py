"""PC2上で動かす非破壊プローブ。

ROS 2 CLI(`ros2 topic list`)はPC2のfoxyからUnitreeのトピックを見つけられない
（Navigation/README.md の実測記録）。型を明示した直接DDS購読なら見えるので、
LiDARとSLAMが実際に配信しているかをこちらで確かめる。

購読するだけで、ロボットには何も指令しない。
"""
import sys, time, threading, json

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
IFACE = sys.argv[2] if len(sys.argv) > 2 else "eth0"

lock = threading.Lock()
stats = {}

def cloud_handler(name):
    def handle(msg):
        with lock:
            s = stats.setdefault(name, {"n": 0, "first": time.time(), "detail": None})
            s["n"] += 1
            s["last"] = time.time()
            if s["detail"] is None:
                fields = " ".join(f.name for f in msg.fields)
                s["detail"] = (
                    f"frame_id={msg.header.frame_id} points={msg.width * msg.height} "
                    f"point_step={msg.point_step} fields=[{fields}]"
                )
    return handle

def string_handler(name):
    def handle(msg):
        with lock:
            s = stats.setdefault(name, {"n": 0, "first": time.time(), "detail": None})
            s["n"] += 1
            s["last"] = time.time()
            try:
                payload = json.loads(msg.data)
                t = payload.get("type")
                sm = payload.get("data", {}).get("stateMachine", {})
                s["detail"] = f"type={t} state={sm.get('state')} ctrName={sm.get('ctrName')}"
            except Exception:
                s["detail"] = f"raw={msg.data[:120]}"
    return handle

ChannelFactoryInitialize(0, IFACE)

TOPICS = [
    ("rt/utlidar/cloud_livox_mid360",     PointCloud2_, cloud_handler),
    ("rt/unitree/slam_mapping/points",    PointCloud2_, cloud_handler),
    ("rt/unitree/slam_relocation/points", PointCloud2_, cloud_handler),
    ("rt/slam_info",                      String_,      string_handler),
]

subs = []
for topic, msg_type, factory in TOPICS:
    sub = ChannelSubscriber(topic, msg_type)
    sub.Init(factory(topic), 10)
    subs.append(sub)

print(f"{DURATION:.0f}秒購読します (iface={IFACE}, domain=0) ...\n")
time.sleep(DURATION)

print(f"{'topic':<38} {'count':>6} {'Hz':>7}  detail")
print("-" * 110)
for topic, _, _ in TOPICS:
    with lock:
        s = stats.get(topic)
    if not s or s["n"] == 0:
        print(f"{topic:<38} {0:>6} {'-':>7}  受信なし")
        continue
    span = max(s["last"] - s["first"], 1e-6)
    hz = (s["n"] - 1) / span if s["n"] > 1 else 0.0
    print(f"{topic:<38} {s['n']:>6} {hz:>7.2f}  {s['detail']}")
