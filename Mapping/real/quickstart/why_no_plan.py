#!/usr/bin/env python3
"""`planner_server` が "Failed to create plan" を出したとき、
**事前地図とライブのコストマップのどちらが廊下を閉じているか**を切り分ける。

同じ 4 連結 BFS を `/map`（事前地図）と `/global_costmap/costmap`（静的層 +
障害物層 + 膨張層）に当てる。届かない側があれば、それが原因である。
届かない場合は、機体側の連結成分のうち**ゴールに最も近づける点**を出す
（そこが詰まり所である）。

## ⚠️ しきい値を間違えると「通行不可 0 セル」に見える

`/global_costmap/costmap` は **0〜100 にスケールされた版**である。
99 = inscribed（機体の中心が入れない）/ 100 = lethal。
0〜255 の生の値は `costmap_raw` に出るが、**この構成では配信されていない**
（`nav2.yaml` の `plugins` に含まれていない）。
253 で数えると 0 セルになり、「コストマップは空だ」と読み違える。

## 使い方

    # ゴールの座標を渡す（Nav2 が動いている機械の上で）
    source IsaacSim_Env/env.sh
    python3 Mapping/real/quickstart/why_no_plan.py 1.15 3.12

⚠️ 経路計画の失敗は**間欠的**である。1 回の観測で「詰まっている」と結論しない。
2026-09-08 の実測では 17 回失敗した区間でも、後から測ると 11.80 m の経路が在った。
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

# 0〜100 スケール側のしきい値。99 以上は機体の中心が入れない
COSTMAP_BLOCK = 99
# /map（0〜100 の占有確率）側のしきい値
MAP_BLOCK = 100
NEIGHBORS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def analyze(name: str, m: OccupancyGrid, thresh: int,
            pose: tuple[float, float], goal: tuple[float, float]) -> None:
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    # 未知（-1）も通れないものとして数える（NavFn の allow_unknown とは別に、
    # 「どこが閉じているか」を見るには未知を塞ぐ方が読みやすい）
    blocked = (g >= thresh) | (g < 0)
    h, w = g.shape

    def cell(x: float, y: float) -> tuple[int, int]:
        return (int((y - oy) / res), int((x - ox) / res))

    def inside(c: tuple[int, int]) -> bool:
        return 0 <= c[0] < h and 0 <= c[1] < w

    print(f"\n[{name}] {w}x{h} / {res:.2f} m / origin ({ox:.2f}, {oy:.2f})  "
          f"通行不可 {int(blocked.sum()):,} セル（{100 * blocked.mean():.1f} %、"
          f"しきい値 {thresh}）")
    a, b = cell(*pose), cell(*goal)
    for label, c in (("機体", a), ("ゴール", b)):
        if not inside(c):
            print(f"  {label}: 地図の外")
            return
        print(f"  {label}セル 値 {int(g[c]):4d}  "
              f"{'**通行不可**' if blocked[c] else '通行可'}")
    if blocked[a] or blocked[b]:
        print("  → 端点が塞がっているので経路は引けない")
        return

    dist = {a: 0}
    q = deque([a])
    while q:
        c = q.popleft()
        if c == b:
            print(f"  → 4 連結で **届く**（{dist[c] * res:.2f} m）")
            return
        for d in NEIGHBORS:
            n = (c[0] + d[0], c[1] + d[1])
            if inside(n) and not blocked[n] and n not in dist:
                dist[n] = dist[c] + 1
                q.append(n)

    best, best_d = None, math.inf
    for r, cc in dist:
        x, y = ox + cc * res, oy + r * res
        d = math.hypot(x - goal[0], y - goal[1])
        if d < best_d:
            best_d, best = d, (x, y)
    print(f"  → 4 連結で **届かない**。機体側の成分は {len(dist):,} セル。"
          f"ゴールに最も近づける点 ({best[0]:+.2f}, {best[1]:+.2f}) で残り "
          f"{best_d:.2f} m ← ここが詰まり所")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("gx", type=float, help="ゴールの map x [m]")
    ap.add_argument("gy", type=float, help="ゴールの map y [m]")
    ap.add_argument("--wait", type=float, default=25.0,
                    help="トピックを待つ秒数")
    args = ap.parse_args()

    qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = Node("why_no_plan")
    buf = Buffer()
    TransformListener(buf, node)
    topics = ("/map", "/global_costmap/costmap")
    got: dict[str, OccupancyGrid] = {}
    for t in topics:
        node.create_subscription(
            OccupancyGrid, t, lambda m, k=t: got.setdefault(k, m), qos)

    end = time.monotonic() + args.wait
    while rclpy.ok() and time.monotonic() < end and len(got) < len(topics):
        rclpy.spin_once(node, timeout_sec=0.2)

    pose = None
    for _ in range(20):
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            pose = (tr.transform.translation.x, tr.transform.translation.y)
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.5)
    if pose is None:
        print("[NG] map -> base_link が引けない。Nav2 と TF を先に確かめる")
        return 1

    goal = (args.gx, args.gy)
    print(f"[pose] 機体 ({pose[0]:+.2f}, {pose[1]:+.2f})   "
          f"ゴール ({goal[0]:+.2f}, {goal[1]:+.2f})   "
          f"直線 {math.hypot(goal[0] - pose[0], goal[1] - pose[1]):.2f} m")
    for t, name, thresh in (("/map", "事前地図", MAP_BLOCK),
                            ("/global_costmap/costmap", "global costmap",
                             COSTMAP_BLOCK)):
        if t in got:
            analyze(name, got[t], thresh, pose, goal)
        else:
            print(f"\n[{name}] {t} が来ない"
                  + ("（always_send_full_costmap: true か確かめる）"
                     if "costmap" in t else ""))
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
