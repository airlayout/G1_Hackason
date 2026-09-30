import json, time
from collections import Counter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
n = Node("read_slam_info")
got = []
n.create_subscription(String, "/slam_info", lambda m: got.append(m.data), 10)
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=1.0)
print("受信 %d 件 / 20秒 = 約 %.1f Hz" % (len(got), len(got)/20.0))

types, best = [], {}
for d in got:
    try:
        j = json.loads(d)
    except Exception:
        continue
    t = j.get("type", "?")
    types.append(t)
    best[t] = j
print("type の内訳:", dict(Counter(types)))

for t, j in best.items():
    if t == "robot_data":
        d = j.get("data", {})
        print("\n--- robot_data(抜粋) battery=%s%% %smV / cpu=%.0f%% %.0fC / sportMode=%s gaitType=%s" % (
            d.get("batteryPower"), d.get("batteryVol"), d.get("cpuUsage", 0), d.get("cpuTemp", 0),
            d.get("sportMode"), d.get("gaitType")))
        continue
    s = json.dumps(j, indent=2, ensure_ascii=False)
    print("\n===== type=%s (%d文字) =====" % (t, len(s)))
    print(s[:1400])
