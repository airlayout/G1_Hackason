#!/usr/bin/env python3
"""コストマップの機体まわりを数える。**消えたことを確かめるための目盛り。**

`clear_costmaps.sh` が消す前と後に呼ぶ。単体でも使える:

    python3 costmap_stats.py /global_costmap/costmap --radius 3.0 --no-sim-time

## なぜ要るのか（2026-09-10 に踏んだ）

障害物層（voxel_layer）の印は、レイキャストが通らない所では消えない。
30 分ほど立ち上げたままにしたら、**機体まわり ±3 m の LETHAL 1,305 セルのうち
384 セルが「事前地図にも無く、その時 LiDAR が見てもいない」消え残り**になっていた。
そのうちの 1 つがちょうどゴールのセルで、Smac2D がゴールへ行けず、
`tolerance` の中で一番近い点＝**機体の現在地**を返し、コントローラが 1.6 ms で
「Reached the goal!」と言った。**落ちないし転倒もしない。1 度も歩かないまま到達と出る。**

⚠️ numpy はコンテナに無いので使わない。素の Python で数える。
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

LETHAL = 100        # OccupancyGrid に落ちた時の値。costmap_2d の 254 に対応
INSCRIBED = 99      # 同 253。**Smac2D はここから先を「通れない」と見る**
SPIN_TIMEOUT_S = 20.0


class Stats(Node):
    def __init__(self, topic: str, use_sim_time: bool) -> None:
        super().__init__("costmap_stats")
        self.set_parameters([Parameter("use_sim_time", value=use_sim_time)])
        # コストマップは transient_local で latch される。これを合わせないと
        # **購読しても 1 通も来ない**（rolling window でない global 側で特に効く）
        qos = QoSProfile(depth=1,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.grid: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, topic, self._on_grid, qos)
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)

    def _on_grid(self, m: OccupancyGrid) -> None:
        self.grid = m

    def robot_xy(self) -> tuple[float, float] | None:
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return tr.transform.translation.x, tr.transform.translation.y


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic", help="/global_costmap/costmap など")
    ap.add_argument("--radius", type=float, default=3.0,
                    help="機体を中心に何 m 四方を数えるか（既定 3.0）")
    ap.add_argument("--no-sim-time", action="store_true",
                    help="実機で使うときに付ける。**/clock が無い環境では必須**")
    a = ap.parse_args()

    rclpy.init()
    node = Stats(a.topic, use_sim_time=not a.no_sim_time)
    end = node.get_clock().now().nanoseconds * 1e-9 + SPIN_TIMEOUT_S
    xy = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        xy = xy or node.robot_xy()
        if node.grid is not None and xy is not None:
            break
        if node.get_clock().now().nanoseconds * 1e-9 > end:
            break

    grid, ok = node.grid, True
    if grid is None:
        print(f"[stats] {a.topic} が来ない", file=sys.stderr)
        ok = False
    if xy is None:
        print("[stats] map -> base_link が引けない", file=sys.stderr)
        ok = False
    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        return 1

    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    k = int(math.ceil(a.radius / res))
    c0 = int((xy[0] - ox) / res)
    r0 = int((xy[1] - oy) / res)

    lethal = inscribed = total = 0
    for r in range(max(0, r0 - k), min(h, r0 + k + 1)):
        base = r * w
        for c in range(max(0, c0 - k), min(w, c0 + k + 1)):
            v = grid.data[base + c]
            total += 1
            if v >= LETHAL:
                lethal += 1
            elif v >= INSCRIBED:
                inscribed += 1

    pct = (100.0 * (lethal + inscribed) / total) if total else 0.0
    print(f"[stats] {a.topic}  機体({xy[0]:+.2f}, {xy[1]:+.2f}) ±{a.radius:.1f} m / "
          f"{total} セル: LETHAL {lethal} / INSCRIBED {inscribed} "
          f"（通れないセル {pct:.1f} %）")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
