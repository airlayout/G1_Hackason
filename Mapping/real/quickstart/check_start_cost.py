#!/usr/bin/env python3
"""**機体の立ち位置の global costmap の値を見る。**（up.sh の段 7・PC2 で動かす）

    python3 check_start_cost.py            # 0 = 経路を作れる / 1 = lethal / 2 = 測れない

## なぜ要るのか（2026-09-28 実機）

立ち位置が lethal だと、2D Goal Pose をクリックしても planner は経路を作らない
（`Smac2D: ... Starting point in lethal space!`）。すると Nav2 の立て直しが
**その場で 90° 回って 5 秒待つ**を 2 回繰り返してからゴールを諦める。
見た目は「回って止まった」だけで、原因がログにしか出ない。
クリックする**前に**分かるよう、起動の最後にここで見る。

## 判定

- 立ち位置のセルの値が **99 以上**（99 = 機体が触れる / 100 = 障害物）なら NG
  （Smac2D は内接 = 99 以上をスタートに使えない）
- あわせて、機体から最も近い 100 のセルまでの距離を出す（直す場所の手がかり）

⚠️ `/global_costmap/costmap` は Nav2 が transient_local で出す。
`always_send_full_costmap: true`（g1_nav2.yaml）が前提（部分更新だけだと全体が来ない）。
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

INSCRIBED = 99
LETHAL = 100
SEARCH_M = 1.5          # 最寄りの障害物を探す範囲（半径）


class Probe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("check_start_cost")
        self.grid: "OccupancyGrid | None" = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, topic, self._on_grid, qos)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self.grid = msg


def nearest_lethal(g: OccupancyGrid, col: int, row: int) -> float:
    w, h, res = g.info.width, g.info.height, g.info.resolution
    reach = int(math.ceil(SEARCH_M / res))
    best = float("inf")
    for r in range(max(0, row - reach), min(h, row + reach + 1)):
        base = r * w
        for c in range(max(0, col - reach), min(w, col + reach + 1)):
            if g.data[base + c] >= LETHAL:
                best = min(best, math.hypot(c - col, r - row) * res)
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/global_costmap/costmap")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--timeout", type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    node = Probe(a.topic)
    t0 = time.time()
    pose = None
    while time.time() - t0 < a.timeout and (node.grid is None or pose is None):
        rclpy.spin_once(node, timeout_sec=0.2)
        if pose is None:
            try:
                tr = node.buf.lookup_transform(a.frame, a.base, Time())
                pose = (tr.transform.translation.x, tr.transform.translation.y)
            except TransformException:
                pass
    g = node.grid
    node.destroy_node()
    rclpy.shutdown()

    if g is None or pose is None:
        what = "costmap" if g is None else f"{a.frame} -> {a.base}"
        print(f"[start] ⚠️ {what} が {a.timeout:.0f} 秒で取れなかった。立ち位置は確かめていない")
        return 2

    res = g.info.resolution
    col = int(math.floor((pose[0] - g.info.origin.position.x) / res))
    row = int(math.floor((pose[1] - g.info.origin.position.y) / res))
    if not (0 <= col < g.info.width and 0 <= row < g.info.height):
        print(f"[start] ⛔ 立ち位置 ({pose[0]:+.2f}, {pose[1]:+.2f}) が costmap の外")
        return 1
    cost = g.data[row * g.info.width + col]
    dist = nearest_lethal(g, col, row)
    near = f"{dist:.2f} m" if math.isfinite(dist) else f"{SEARCH_M:.1f} m 以内に無い"
    print(f"[start] 立ち位置 ({pose[0]:+.2f}, {pose[1]:+.2f}) の cost {cost} / 最寄りの障害物 {near}")
    if cost >= INSCRIBED:
        print("[start] ⛔ 立ち位置が lethal。**クリックしても経路が作れず、その場で回って止まる**")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
