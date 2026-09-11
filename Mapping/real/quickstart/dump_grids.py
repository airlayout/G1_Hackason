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


def latched(depth: int = 1) -> QoSProfile:
    """map_server も costmap も octomap_server も TRANSIENT_LOCAL で出す。

    ⚠️ VOLATILE で購読すると**永遠に来ない**（2026-09-07 に踏んだ型）。
    """
    return QoSProfile(depth=depth,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class GridDumper(Node):
    def __init__(self, layers):
        super().__init__("dump_grids")
        self.grids = {}
        for name, topic in layers:
            self.create_subscription(
                OccupancyGrid, topic,
                lambda m, n=name: self.grids.setdefault(n, m), latched())


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
    a = ap.parse_args()

    layers = DEFAULT_LAYERS
    if a.layer:
        layers = tuple(tuple(spec.split("=", 1)) for spec in a.layer)

    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = GridDumper(layers)
    deadline = node.get_clock().now().nanoseconds / 1e9 + a.timeout
    while rclpy.ok() and len(node.grids) < len(layers):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds / 1e9 > deadline:
            break

    report = {}
    for name, topic in layers:
        grid = node.grids.get(name)
        if grid is None:
            # ⚠️ 黙って空の層を描かせない。来なかったことを残す
            report[name] = {"topic": topic, "received": False}
            print("--  {:<16} {:<28} 来なかった".format(name, topic))
            continue
        info = write_layer(a.out, name, grid)
        info.update({"topic": topic, "received": True})
        report[name] = info
        print("OK  {:<16} {:<28} {}x{} / res {} / 占有(>=65) {} / 膨張(26..64) {}".format(
            name, topic, info["size"][0], info["size"][1], info["resolution"],
            info["occupied_65_98"] + info["lethal_99_100"], info["inflated_26_64"]))

    (a.out / "counts.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    got = sum(1 for v in report.values() if v.get("received"))
    print("\n{} / {} 層を {} に書いた".format(got, len(layers), a.out))
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
