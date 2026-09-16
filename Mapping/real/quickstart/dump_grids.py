#!/usr/bin/env python3
"""RViz に出ている 2D の層を、そのまま **map_server 形式の PGM + yaml** に落とす。

## なぜ要るのか

2026-09-11 に「RViz の 2D 地図のざらつきはどの層か」を調べようとして、
`import -window root` でスクリーンショットを撮ったら**デスクトップのアイコンしか
写らなかった**（RViz のウィンドウが root に載っていない。`xdotool` も未導入）。

層は latched な topic として流れているので、**画面を撮らずに topic を落として描く**。
落とす形式を map_server と同じ PGM + yaml にしておくと、既にある読み手
（`check_map_clearance.load_map` / `measure_overlay.read_map` / `render_nav_map.py`）
が**無改造で**読める。

## 使い方（コンテナの中で。Mac には rclpy が無い）

    docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 rviz bash -c \\
        'source /opt/ros/humble/setup.bash && python3 /work/.../dump_grids.py --out /work/.../layers'

    # 出るもの: <out>/static.{pgm,yaml} global_costmap.{pgm,yaml} projected.{pgm,yaml}
    #           <out>/counts.json（層ごとの内訳。描かずに数字だけ見たいとき用）

そのあと Mac 側で `render_layers.py` が 1 枚の PNG にする。

⚠️ **来ない層は「来なかった」と書いて飛ばす。**黙って空の層を描かない
（`/projected_map` は 2026-09-11 から既定 off なので、普通は来ないのが正しい）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

# map_server の PGM 規約（nav2_map_server/map_io.cpp と同じ）
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 205
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25

# 既定で落とす層。名前は出力ファイル名になる
DEFAULT_LAYERS = (
    ("static", "/map"),
    ("global_costmap", "/global_costmap/costmap"),
    ("local_costmap", "/local_costmap/costmap"),
    ("projected", "/projected_map"),
)


def map_qos(transient_local: bool = True, depth: int = 5) -> QoSProfile:
    """地図の層を購読する QoS。

    `map_server` も costmap も `octomap_server`（`latch:=true`）も TRANSIENT_LOCAL で出す。
    ⚠️ **1 回だけ latched で出る層を VOLATILE で購読すると永遠に来ない**（2026-09-07）。

    逆向きの罠もある。`octomap_server` を **`latch:=false`** で起こすと
    パブリッシャが **VOLATILE** になり、**TRANSIENT_LOCAL で購読すると
    QoS 不一致で 1 通も来ない**（2026-09-16 実測）。しかも `octomap_server` は
    「購読者が居る層だけ出す」ので、不一致だと購読者 0 と数えられ、
    `/projected_map` は**そもそも計算されない**。二重に来なくなる。
    ⇒ `latch:=false` で回すときは `--volatile` を付けること。
    """
    return QoSProfile(depth=depth,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if transient_local
                                  else QoSDurabilityPolicy.VOLATILE))


class GridDumper(Node):
    """層ごとに**最後に来た**メッセージを持つ。

    ⚠️ 2026-09-16 まで `setdefault` で**最初の 1 通**を握っていた。`/map` のように
    1 回だけ latched で出る層では同じだが、`octomap_server` の `/projected_map` は
    点群が来るたびに出し直すので、育った後の姿が取れず「1 セルも動いていない」と
    誤診した（TRANSIENT_LOCAL の履歴から古い標本が先に届く）。
    """

    def __init__(self, layers, transient_local: bool = True):
        super().__init__("dump_grids")
        self.grids = {}
        self.counts = {}
        qos = map_qos(transient_local)
        for name, topic in layers:
            self.create_subscription(OccupancyGrid, topic, self._keep(name), qos)

    def _keep(self, name: str):
        def callback(message: OccupancyGrid) -> None:
            self.grids[name] = message
            self.counts[name] = self.counts.get(name, 0) + 1
        return callback


def to_pgm_bytes(grid: OccupancyGrid) -> "tuple[bytes, int, int]":
    """OccupancyGrid を map_server と同じ 3 値の PGM にする。

    -1(未知) -> 205 / occ >= 65 -> 0(占有) / occ <= 25 -> 254(空き) / その間 -> 205。
    ⚠️ 膨張帯（costmap の 1..98）は**この 3 値では潰れる**。内訳は counts.json に残す。
    """
    data = np.asarray(grid.data, dtype=np.int16).reshape(
        grid.info.height, grid.info.width)
    image = np.full(data.shape, PGM_UNKNOWN, dtype=np.uint8)
    image[(data >= 0) & (data <= int(FREE_THRESH * 100))] = PGM_FREE
    image[data >= int(OCCUPIED_THRESH * 100)] = PGM_OCCUPIED
    # ROS の格子は左下が原点、PGM は左上。map_server と同じく上下を返す
    image = np.flipud(image)
    header = "P5\n{} {}\n255\n".format(data.shape[1], data.shape[0]).encode()
    return header + image.tobytes(), data.shape[1], data.shape[0]


def counts_of(grid: OccupancyGrid) -> dict:
    data = np.asarray(grid.data, dtype=np.int16)
    return {
        "cells": int(data.size),
        "unknown": int((data < 0).sum()),
        "free": int(((data >= 0) & (data <= 25)).sum()),
        # 膨張帯。PGM では未知に潰れるのでここに残す
        "inflated_26_64": int(((data > 25) & (data < 65)).sum()),
        "occupied_65_98": int(((data >= 65) & (data < 99)).sum()),
        "lethal_99_100": int((data >= 99).sum()),
        "frame_id": grid.header.frame_id,
        "resolution": float(grid.info.resolution),
        "origin": [float(grid.info.origin.position.x),
                   float(grid.info.origin.position.y)],
    }


def write_layer(out: Path, name: str, grid: OccupancyGrid) -> dict:
    blob, w, h = to_pgm_bytes(grid)
    (out / (name + ".pgm")).write_bytes(blob)
    (out / (name + ".yaml")).write_text(
        "image: {}.pgm\n"
        "resolution: {}\n"
        "origin: [{:.4f}, {:.4f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: {}\n"
        "free_thresh: {}\n".format(
            name, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            OCCUPIED_THRESH, FREE_THRESH))
    info = counts_of(grid)
    info["size"] = [w, h]
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True, help="書き出し先のディレクトリ")
    ap.add_argument("--timeout", type=float, default=12.0,
                    help="latched が全部来るまで待つ上限 [s]")
    ap.add_argument("--layer", action="append", default=None,
                    metavar="NAME=/topic", help="層を指定（繰り返し可）")
    ap.add_argument("--volatile", action="store_true",
                    help="VOLATILE で購読する。octomap_server を latch:=false で"
                         "起こしたときはこちら（TRANSIENT_LOCAL だと 1 通も来ない）")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="全層がそろってから、さらに待つ時間[s]。"
                         "出し直される層（/projected_map 等）の最後の姿を取るため")
    a = ap.parse_args()

    layers = DEFAULT_LAYERS
    if a.layer:
        layers = tuple(tuple(spec.split("=", 1)) for spec in a.layer)

    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = GridDumper(layers, transient_local=not a.volatile)
    now = lambda: node.get_clock().now().nanoseconds / 1e9
    deadline = now() + a.timeout
    while rclpy.ok() and len(node.grids) < len(layers) and now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    # ⚠️ 全層が 1 通そろってからも少し回す。出し直される層は**最後の姿**が要る
    settle = now() + a.settle
    while rclpy.ok() and now() < settle:
        rclpy.spin_once(node, timeout_sec=0.2)

    report = {}
    for name, topic in layers:
        grid = node.grids.get(name)
        if grid is None:
            # ⚠️ 黙って空の層を描かせない。来なかったことを残す
            report[name] = {"topic": topic, "received": False}
            print("--  {:<16} {:<28} 来なかった".format(name, topic))
            continue
        info = write_layer(a.out, name, grid)
        info.update({"topic": topic, "received": True,
                     "messages": node.counts.get(name, 0)})
        report[name] = info
        print("OK  {:<16} {:<28} {}x{} / res {:.3f} / 占有(>=65) {:,} / "
              "空き {:,} / 未知 {:,} / 受信 {} 通".format(
                  name, topic, info["size"][0], info["size"][1], info["resolution"],
                  info["occupied_65_98"] + info["lethal_99_100"],
                  info["free"], info["unknown"], info["messages"]))

    (a.out / "counts.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    got = sum(1 for v in report.values() if v.get("received"))
    print("\n{} / {} 層を {} に書いた".format(got, len(layers), a.out))
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
