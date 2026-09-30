#!/usr/bin/env python3
"""いま機体が立っている場所から Nav2 が**出発できるか**を判定する（購読のみ・機体は動かない）。

    python3 can_start_here.py

## なぜ要るのか（2026-09-25 にこれで詰まった）

巡回を start しても `planner_server: GridBased: failed to create plan` が延々出て、
**復帰動作の spin だけして止まる**ことがあった。原因は**ゴールではなく出発点**で、
機体が地図上の LETHAL セルの上に立っていた（§7 の照合は 98% 通っていたので、
自己位置ではなく**地図と現況の食い違い**）。

Nav2 のログは「経路が引けない」としか言わないので、**出発点が原因だと分からない**。
この道具は足元の値をそのまま出すので、1回で分かる。

出力の読み方:

| | |
|---|---|
| `足元の値 = 100` | **LETHAL**。ここからは絶対に出発できない → 機体を動かす |
| `足元の値 = 99` | 内接膨張。機体中心がここにあると衝突扱い → 動かす |
| `周囲0.35m の最悪値 >= 99` | 出発はできるが余裕が無い → できれば動かす |
| `足元の値 = 0` かつ `最悪値 = 0` | ✅ 出発できる |

⚠️ `/global_costmap/costmap` は **OccupancyGrid の 0〜100 スケール**で出る:
    -1=未知 / 100=LETHAL / 99=内接膨張 / 0〜98=膨張のコスト
   **nav2 内部の 0〜255 と混同しないこと**（同日にこれで「free」と誤判定した）。
⚠️ **TF リスナーは購読を始めてから溜まるまで数秒かかる。** costmap が来た時点で
   待機ループを抜けると `two or more unconnected trees` になる（同日にこれで誤診した）。

📌 **`costmap か TF が取れなかった` と出たら、まず内蔵SLAM が動いているかを見る**
   （止まっていると TF が繋がらない）。`ros2 topic hz /unitree/slam_mapping/odom`。

⚠️ **この版そのものは実機で未検証。** 2026-09-25 の切り分けは同じ処理を
   その場限りのスクリプトで回して得たもので、`tools/` へ整理したあと
   （巡回路を yaml から読むように変えたあと）は内蔵SLAM を止めてしまい、
   **実機で1度も通していない。** 次の実機セッションで最初に叩いて確かめること。
"""
import math, sys
from pathlib import Path

import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

LETHAL, INSCRIBED = 100, 99

# 巡回路は yaml から読む（点を書き換えてもこの道具を直さなくて済むように）。
PATROL_YAML = Path("/home/unitree/g1_nav2/g1_ws/install/g1_navigation/share/"
                   "g1_navigation/config/patrol_room_a.yaml")


def load_goals() -> dict:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else PATROL_YAML
    try:
        import yaml
        wps = yaml.safe_load(path.read_text(encoding="utf-8"))["waypoints"]
        return {w["name"]: (float(w["x"]), float(w["y"])) for w in wps}
    except Exception as e:
        print(f"⚠️ 巡回路を読めなかった({path}: {e})。各点の見通しは出さない")
        return {}


GOALS = load_goals()

rclpy.init(); node = Node("here_ok")
buf = Buffer(); TransformListener(buf, node)
got = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: got.append(m), qos)
# ⚠️ costmap が来ても抜けない。TF が引けるまで回す
t = None
for _ in range(80):
    rclpy.spin_once(node, timeout_sec=0.5)
    if got:
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time()); break
        except Exception:
            pass
if not got or t is None:
    print("costmap か TF が取れなかった"); sys.exit(1)

m = got[-1]; w, h = m.info.width, m.info.height; res = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
a = np.array(m.data, np.int16).reshape(h, w)
rx, ry = t.transform.translation.x, t.transform.translation.y
q = t.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def at(x, y):
    c, r = int((x-ox)/res), int((y-oy)/res)
    return int(a[r, c]) if 0 <= r < h and 0 <= c < w else -1

v = at(rx, ry)
k = int(0.35/res)
rr, rc_ = int((ry-oy)/res), int((rx-ox)/res)
blk = a[max(0,rr-k):rr+k+1, max(0,rc_-k):rc_+k+1]
worst = int(blk.max()); unknown = int((blk < 0).sum())

print(f"機体(map) = ({rx:+.2f}, {ry:+.2f})  向き {math.degrees(yaw):+.0f}°")
print(f"足元の値 = {v}  ({'未知' if v<0 else 'LETHAL' if v>=LETHAL else '内接膨張' if v>=INSCRIBED else 'cost '+str(v)})")
print(f"周囲0.35m の最悪値 = {worst}  未知セル = {unknown}")
if v < 0 or v >= INSCRIBED:
    print("→ ❌ **ここからは出発できない**（足元が走行不可）。もう少し動かすこと")
elif worst >= INSCRIBED or unknown:
    print("→ ⚠️ 足元は出られるが、**0.35m 以内に走行不可か未知がある**。余裕がほしい")
else:
    print("→ ✅ **ここから出発できる**")

print("\n--- 各点までの見通し（直線上に走行不可があるか。実際の経路は迂回できる） ---")
for nm, (gx, gy) in GOALS.items():
    d = math.hypot(gx-rx, gy-ry)
    N = max(2, int(d/0.05))
    bad = sum(1 for i in range(N+1)
              if at(rx+(gx-rx)*i/N, ry+(gy-ry)*i/N) >= INSCRIBED
              or at(rx+(gx-rx)*i/N, ry+(gy-ry)*i/N) < 0)
    rel = (math.degrees(math.atan2(gy-ry, gx-rx) - yaw) + 180) % 360 - 180
    side = "正面" if abs(rel) < 25 else ("左" if rel > 0 else "右")
    print(f"  {nm}({gx:+5.2f},{gy:+5.2f}) 値={at(gx,gy):4d}  距離{d:5.2f}m  {side}{abs(rel):3.0f}°  "
          f"直線上の走行不可 {bad}/{N+1}")
