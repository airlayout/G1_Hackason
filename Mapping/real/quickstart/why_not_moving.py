#!/usr/bin/env python3
"""**経路も `/cmd_vel` も出ているのに機体が動かない**とき、機体の周りを見る。

`why_no_plan.py` は「経路が引けない」を切り分ける。こちらは
「経路は引けているのに進まない」を切り分ける。local costmap を機体の
**胴体座標系**で見て、正面がどれだけ空いているかを出す。

## ⚠️ 方角は胴体座標系で見る

map の +x を「前」と読むと誤診する。2026-09-08 の実測では、機体は
`yaw −173.4 度`（map の −x をほぼ向いていた＝進行方向は合っていた）で、
周りには自由空間が **31.4 m² / 最遠 4.24 m** も在ったのに、
**正面だけ 0.80 m しか無かった**。「囲まれている」ではなく
「**正面が詰まっている**」が正しい読みである。

## ⚠️ しきい値と QoS

`/local_costmap/costmap` も **0〜100 にスケールされた版**（99=inscribed /
100=lethal）。`why_no_plan.py` と同じ罠。

`/plan` は **VOLATILE** で出ている。コストマップと同じ TRANSIENT_LOCAL で
購読すると QoS が噛み合わず**何も来ない**。それを「経路が無い」と読み違える
（2026-09-08 に踏んだ）。コストマップだけが TRANSIENT_LOCAL である。

    source IsaacSim_Env/env.sh
    python3 Mapping/real/quickstart/why_not_moving.py
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

COSTMAP_BLOCK = 99           # 0〜100 スケール。99 以上は機体の中心が入れない
PROBE_MAX_M = 3.0            # 各方角に何 m まで見るか
NEIGHBORS = ((1, 0), (-1, 0), (0, 1), (0, -1))
# 胴体座標系での方角（度）。0 = 正面
BEARINGS = (("正面", 0), ("左斜め前", 45), ("左", 90), ("左斜め後ろ", 135),
            ("真後ろ", 180), ("右斜め後ろ", 225), ("右", 270), ("右斜め前", 315))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=20.0)
    args = ap.parse_args()

    qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = Node("why_not_moving")
    buf = Buffer()
    TransformListener(buf, node)
    # ⚠️ **最新**を持つ（setdefault で最初のものを掴むと、機体が動いた後には
    # 古い窓を見ることになり「機体が窓の外」になる。2026-09-08 に踏んだ）。
    got: dict[str, object] = {}

    def keep(key: str):
        def cb(m):
            got[key] = m
        return cb

    node.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                             keep("local"), qos)
    # ⚠️ /plan は VOLATILE で出ている。qos（TRANSIENT_LOCAL）で購読すると
    # QoS が噛み合わず**何も来ない**。それを「経路が無い」と読み違える。
    # コストマップだけが TRANSIENT_LOCAL である。
    node.create_subscription(Path, "/plan", keep("plan"), 10)
    node.create_subscription(Twist, "/cmd_vel", keep("cmd"), 10)

    # ⚠️ /plan は約 1 Hz なので、local costmap が来た時点で抜けると間に合わない。
    # 3 つ揃うか、--wait を使い切るまで回す。
    end = time.monotonic() + args.wait
    while rclpy.ok() and time.monotonic() < end and len(got) < 3:
        rclpy.spin_once(node, timeout_sec=0.2)

    pose = None
    for _ in range(20):
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            q = tr.transform.rotation
            pose = (tr.transform.translation.x, tr.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)))
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.5)
    if "local" not in got or pose is None:
        print("[NG] /local_costmap/costmap か map -> base_link が取れない")
        return 1

    cmd = got.get("cmd")
    plan = got.get("plan")
    print(f"[pose] ({pose[0]:+.2f}, {pose[1]:+.2f}) yaw "
          f"{math.degrees(pose[2]):+.1f} 度")
    if cmd is not None:
        print(f"[cmd ] vx {cmd.linear.x:+.3f} m/s  yaw {cmd.angular.z:+.3f} rad/s"
              + ("  ← その場旋回（並進 0）" if abs(cmd.linear.x) < 0.02
                 and abs(cmd.angular.z) > 0.02 else ""))
    else:
        print("[cmd ] /cmd_vel が来ない（誰も指令していない）")
    if plan is not None and plan.poses:
        p = plan.poses
        print(f"[plan] {len(p)} 点  末尾 ({p[-1].pose.position.x:+.2f}, "
              f"{p[-1].pose.position.y:+.2f})")
    else:
        print("[plan] /plan が来ない。⚠️ まず QoS を疑う（/plan は VOLATILE）。"
              "本当に無いなら why_no_plan.py を使う")

    m = got["local"]
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    blocked = (g >= COSTMAP_BLOCK) | (g < 0)
    h, w = g.shape
    print(f"[local] {w}x{h} / {res:.2f} m = {w * res:.1f} × {h * res:.1f} m の窓。"
          f"通行不可 {100 * blocked.mean():.1f} %")

    def cell(x: float, y: float) -> tuple[int, int]:
        return (int((y - oy) / res), int((x - ox) / res))

    def inside(c: tuple[int, int]) -> bool:
        return 0 <= c[0] < h and 0 <= c[1] < w

    c0 = cell(pose[0], pose[1])
    if not inside(c0):
        print("[NG] 機体が窓の外")
        return 1
    print(f"[local] 機体セル 値 {int(g[c0]):3d}  "
          f"{'**通行不可**' if blocked[c0] else '通行可'}")

    if not blocked[c0]:
        seen = {c0}
        q2 = deque([c0])
        far = 0.0
        while q2:
            c = q2.popleft()
            far = max(far, math.hypot((c[1] - c0[1]) * res,
                                      (c[0] - c0[0]) * res))
            for d in NEIGHBORS:
                n = (c[0] + d[0], c[1] + d[1])
                if inside(n) and not blocked[n] and n not in seen:
                    seen.add(n)
                    q2.append(n)
        print(f"[local] 機体から届く自由空間 {len(seen) * res * res:.2f} m² / "
              f"最遠 {far:.2f} m")
        print("        ⚠️ ここが広くても「動ける」ではない。見るのは正面である")

    print("\n[local] 胴体座標系で、自由なまま進める距離:")
    for name, rel in BEARINGS:
        a = pose[2] + math.radians(rel)
        d = 0.0
        while d < PROBE_MAX_M:
            c = cell(pose[0] + math.cos(a) * (d + res),
                     pose[1] + math.sin(a) * (d + res))
            if not inside(c) or blocked[c]:
                break
            d += res
        mark = ""
        if rel == 0 and d < 0.5:
            mark = "  ← **正面が詰まっている**。ここが動かない理由"
        elif d < 0.3:
            mark = "  ← 塞がっている"
        print(f"   {name:10s} {d:4.2f} m{mark}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
