#!/usr/bin/env python3
"""地図ファイル（map_server 形式の .yaml + .pgm）を `/map` に流す（map_server の代わり）。

    python3 publish_map_file.py <地図の.yaml>

## なぜ要るのか

`bench_localizer.sh` で `map_localizer.py` を記録済み bag で動かすとき、`/map` が要る。
**このPC の Humble コンテナには `nav2_map_server` が入っていない**ので、
lifecycle も Nav2 も要らない最小の代わりを置いた（PC2 でも同じ手順で動かすため）。

⚠️ 変換は map_server の `trinary` に合わせる（`negate: 0` のみ対応）:
明るさ p=(255-画素)/255 が `occupied_thresh` 超→100、`free_thresh` 未満→0、その間→-1。
⚠️ PGM は**上の行から**、OccupancyGrid は**下の行から**並ぶので上下を反転する。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def read_pgm(path: Path) -> np.ndarray:
    """P5（8bit バイナリ）の PGM を読む。PIL が無い環境（PC2 の pixi）でも動くように自前で読む。"""
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        # 空白とコメント（# から行末）を飛ばす
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5" or maxval > 255:
        raise ValueError(f"P5 / 8bit の PGM だけ扱う: {magic!r} maxval={maxval}")
    pos += 1  # ヘッダ末尾の空白1文字
    return np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos).reshape(height, width)


def load_grid(map_yaml: Path) -> OccupancyGrid:
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    if int(meta.get("negate", 0)) != 0:
        raise ValueError("negate: 1 の地図には対応していない")
    img = read_pgm(map_yaml.parent / meta["image"])
    p = (255.0 - img.astype(np.float32)) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p > float(meta["occupied_thresh"])] = 100
    grid[p < float(meta["free_thresh"])] = 0
    grid = np.flipud(grid)

    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = float(meta["resolution"])
    msg.info.width = img.shape[1]
    msg.info.height = img.shape[0]
    msg.info.origin.position.x = float(meta["origin"][0])
    msg.info.origin.position.y = float(meta["origin"][1])
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.ravel().tolist()
    return msg


def main() -> None:
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) != 1:
        print("使い方: publish_map_file.py <地図の.yaml>", file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    node = Node("map_file_publisher")
    msg = load_grid(Path(args[0]))
    # map_server と同じ transient local。後から起動した購読者にも届く
    qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(OccupancyGrid, "/map", qos)
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    occupied = sum(1 for v in msg.data if v == 100)
    print(f"[publish_map_file] {args[0]} {msg.info.width}x{msg.info.height} "
          f"占有セル {occupied} を /map に出した", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
