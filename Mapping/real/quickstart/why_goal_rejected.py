#!/usr/bin/env python3
"""ゴールが**そのセルの値のせいで**拒否されるのかを切り分ける。

`why_no_plan.py` は「廊下が閉じているか」を BFS で見る。こちらは
**ゴール 1 セルの値**だけを見る。プランナによってはこれだけで即失敗するため。

## なぜ 1 セルで決まるのか（2026-09-09 に実測で踏んだ）

ThetaStar は `createPlan()` の冒頭でゴールセルを見て、**厳密に 254
（`LETHAL_OBSTACLE`）と一致したら即例外**を投げる（`theta_star_planner.cpp`）。
探索は 1 度も走らない。しかも **ThetaStar には `tolerance` パラメータが無い**
（`clearStart()` で開始セルは救済するのにゴールは救済しない）。
一方 NavFn と Smac2D は `tolerance: 0.5` を持ち、ゴールが塞がっていても
その近傍の自由セルへ逃がす。**同じ地図・同じゴールで ThetaStar だけが落ちる。**

## しきい値（⚠️ ここを間違えると読み違える）

`/global_costmap/costmap` は **0〜100 にスケールされた版**である
（生の 0〜255 は `costmap_raw` だがこの構成では配信されていない）。

| 0〜100 | 生の値 | 意味 | ThetaStar |
|---|---|---|---|
| 100 | 254 | lethal（**障害物として直接マークされた**） | **即拒否** |
| 99 | 253 | inscribed（膨張層。機体中心が入れない） | 通す |
| -1 | 255 | unknown | 通す |

**膨張層は最大 253 までしか書かない。** つまり 100 を見たら、それは
静的層（地図の占有セル）か `obstacle_layer`（LiDAR のマーキング）のどちらかである。
事前地図が自由なら、**犯人は obstacle_layer しかいない**。

## 使い方

    source IsaacSim_Env/env.sh
    python3 why_goal_rejected.py 15.85 -2.28            # 1 回見る
    python3 why_goal_rejected.py 15.85 -2.28 --watch 60 # 60 秒間 5 秒ごと
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LETHAL = 100        # 0〜100 スケール。生の 254
INSCRIBED = 99      # 生の 253。膨張層が書ける上限
PLANNER_TOLERANCE_M = 0.5   # NavFn / Smac2D の tolerance。ThetaStar には無い


def latched(depth: int = 1) -> QoSProfile:
    """コストマップも /map も TRANSIENT_LOCAL で配信される。"""
    return QoSProfile(
        depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class GoalInspector(Node):
    def __init__(self) -> None:
        super().__init__("why_goal_rejected")
        self.costmap: OccupancyGrid | None = None
        self.map: OccupancyGrid | None = None
        self.scan: LaserScan | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # ⚠️ 最新を持つこと。最初の 1 件を掴むと古い窓を見る
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._on_costmap, latched())
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched())
        # ⚠️ /scan は BEST_EFFORT で配信される。既定（RELIABLE）で購読すると
        # QoS 不一致で **1 件も届かない**（警告は出るが黙って空になる）。
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)

    def _on_costmap(self, m: OccupancyGrid) -> None:
        self.costmap = m

    def _on_map(self, m: OccupancyGrid) -> None:
        self.map = m

    def _on_scan(self, m: LaserScan) -> None:
        self.scan = m

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def grid_of(m: OccupancyGrid) -> tuple[np.ndarray, float, float, float]:
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    return (g, m.info.resolution,
            m.info.origin.position.x, m.info.origin.position.y)


def value_at(m: OccupancyGrid, x: float, y: float) -> int | None:
    g, res, ox, oy = grid_of(m)
    c, r = int((x - ox) / res), int((y - oy) / res)
    if not (0 <= r < g.shape[0] and 0 <= c < g.shape[1]):
        return None
    return int(g[r, c])


def describe(v: int | None) -> str:
    if v is None:
        return "地図の外"
    if v == LETHAL:
        return f"{v}（生 254 = lethal。**障害物として直接マークされている**）"
    if v == INSCRIBED:
        return f"{v}（生 253 = inscribed。膨張層。機体中心は入れないが lethal ではない）"
    if v < 0:
        return f"{v}（unknown）"
    return f"{v}（通行可）"


def nearest_free(m: OccupancyGrid, x: float, y: float,
                 max_r_m: float) -> tuple[float, float, float] | None:
    """半径 max_r_m 以内で最も近い「lethal でない」セルを返す（距離, x, y）。"""
    g, res, ox, oy = grid_of(m)
    c0, r0 = int((x - ox) / res), int((y - oy) / res)
    rad = int(math.ceil(max_r_m / res))
    best = None
    for dr in range(-rad, rad + 1):
        for dc in range(-rad, rad + 1):
            r, c = r0 + dr, c0 + dc
            if not (0 <= r < g.shape[0] and 0 <= c < g.shape[1]):
                continue
            if g[r, c] == LETHAL or g[r, c] < 0:
                continue
            d = math.hypot(dr, dc) * res
            if d <= max_r_m and (best is None or d < best[0]):
                best = (d, ox + (c + 0.5) * res, oy + (r + 0.5) * res)
    return best


def neighborhood(m: OccupancyGrid, x: float, y: float, rad: int) -> str:
    g, res, ox, oy = grid_of(m)
    c0, r0 = int((x - ox) / res), int((y - oy) / res)
    lines = []
    for dr in range(rad, -rad - 1, -1):       # 上が +y になるように
        row = []
        for dc in range(-rad, rad + 1):
            r, c = r0 + dr, c0 + dc
            if not (0 <= r < g.shape[0] and 0 <= c < g.shape[1]):
                row.append("  x")
                continue
            v = int(g[r, c])
            row.append(" **" if (dr == 0 and dc == 0) else f"{v:3d}")
        lines.append("  " + " ".join(row))
    return "\n".join(lines)


def scan_hits_near(node: GoalInspector, x: float, y: float,
                   radius_m: float) -> list[tuple[float, float, float]]:
    """ゴール近傍に落ちている LiDAR 反射点を map 系で返す（距離, x, y）。"""
    if node.scan is None:
        return []
    try:
        tr = node.tf_buffer.lookup_transform(
            "map", node.scan.header.frame_id, rclpy.time.Time())
    except Exception:
        return []
    t, q = tr.transform.translation, tr.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    out = []
    for i, rng in enumerate(node.scan.ranges):
        if not math.isfinite(rng) or rng <= node.scan.range_min:
            continue
        a = node.scan.angle_min + i * node.scan.angle_increment + yaw
        px, py = t.x + rng * math.cos(a), t.y + rng * math.sin(a)
        d = math.hypot(px - x, py - y)
        if d <= radius_m:
            out.append((d, px, py))
    return sorted(out)


def report(node: GoalInspector, x: float, y: float, rad: int) -> None:
    print(f"\n===== {time.strftime('%H:%M:%S')} ゴール ({x:+.2f}, {y:+.2f}) =====")
    if node.map is not None:
        print(f"  事前地図 /map            : {describe(value_at(node.map, x, y))}")
    else:
        print("  事前地図 /map            : 届いていない")

    if node.costmap is None:
        print("  /global_costmap/costmap  : 届いていない")
        return
    v = value_at(node.costmap, x, y)
    print(f"  /global_costmap/costmap  : {describe(v)}")

    if v == LETHAL:
        print("\n  → **ThetaStar はこのゴールを即拒否する**"
              "（getCost(goal) == 254 で例外。探索は走らない）")
        near = nearest_free(node.costmap, x, y, PLANNER_TOLERANCE_M)
        if near:
            print(f"  → NavFn / Smac2D は tolerance {PLANNER_TOLERANCE_M} m で "
                  f"{near[0]:.2f} m 先の ({near[1]:+.2f}, {near[2]:+.2f}) へ逃がせる"
                  "（だから同じ状況でも落ちない）")
        else:
            print(f"  → tolerance {PLANNER_TOLERANCE_M} m 以内に逃げ場が無い"
                  "（NavFn / Smac2D も失敗するはず）")
        mv = value_at(node.map, x, y) if node.map else None
        if mv is not None and mv != LETHAL:
            print("  → **事前地図では塞がっていない。書いたのは obstacle_layer**"
                  "（膨張層は 253 までしか書けないため）")
    elif v == INSCRIBED:
        print("\n  → inscribed なので ThetaStar は拒否しない"
              "（拒否は 254 と厳密一致のときだけ）")

    hits = scan_hits_near(node, x, y, 0.6)
    if hits:
        print(f"\n  いま /scan がゴールの 0.6 m 以内に見ている点: {len(hits)} 個"
              f"（最寄り {hits[0][0]:.2f} m @ ({hits[0][1]:+.2f}, {hits[0][2]:+.2f})）")
    else:
        print("\n  いま /scan はゴールの 0.6 m 以内に何も見ていない"
              "（＝過去にマークされたまま消えていない可能性）")

    print(f"\n  周辺 {rad * 2 + 1}x{rad * 2 + 1} セル（** がゴール。上が +y）:")
    print(neighborhood(node.costmap, x, y, rad))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--radius", type=int, default=4, help="周辺表示のセル数")
    ap.add_argument("--watch", type=float, default=0.0,
                    help="この秒数のあいだ 5 秒ごとに見続ける（間欠かどうかの判定用）")
    args = ap.parse_args()

    rclpy.init()
    node = GoalInspector()
    node.spin_for(5.0)
    if node.costmap is None:
        print("[NG] /global_costmap/costmap が届かない。"
              "always_send_full_costmap: true と Nav2 の起動を確認すること")
        node.destroy_node(); rclpy.shutdown(); return 1

    report(node, args.x, args.y, args.radius)
    end = time.monotonic() + args.watch
    while rclpy.ok() and time.monotonic() < end:
        node.spin_for(5.0)
        report(node, args.x, args.y, args.radius)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
