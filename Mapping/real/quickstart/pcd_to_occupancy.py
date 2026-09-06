#!/usr/bin/env python3
"""地図の PCD を Nav2 が読む 2D 占有格子（.pgm + .yaml）にする。

## なぜ要るか

Nav2 の global costmap の静的レイヤは `nav_msgs/OccupancyGrid` を要求する。
`octomap_server` の `/projected_map` でも足りるが、**あれは走りながら育つ**ので
部屋の全体像を最初から持てない。過去に作った地図を先に読ませておけば、
まだ見ていない場所も含めて経路を引ける。

## 何を占有とみなすか

床から `--band` の高さ帯にある点だけを障害物とする。天井と床そのものは入れない
（入れると部屋全体が壁になる）。床の高さは点群の低い側の分位で決める。

**観測できた範囲だけを「空き」にする。**格子を全部空きにすると、
地図の外まで通れることになってしまう。床が測れたセルと障害物セルの
どちらでもない所は**未知(-1)**にする。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/pcd_to_occupancy.py \\
        runs/<id>/map/map_octomap_r4_s5.pcd runs/<id>/map/nav_map
    # -> nav_map.pgm と nav_map.yaml ができる
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage

FLOOR_PERCENTILE = 5.0      # 床の高さに使う分位。低い側から取る
OBSTACLE_BAND = (0.15, 1.80)  # 床上のこの帯を障害物とする
FLOOR_BAND = (-0.20, 0.15)    # 床面とみなす帯
# pgm の慣習。Nav2 の map_server がこの値で読む
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pcd", type=Path)
    p.add_argument("output", type=Path, help="拡張子なしの出力名（.pgm と .yaml を作る）")
    p.add_argument("--resolution", type=float, default=0.10, help="格子の一辺[m]")
    p.add_argument("--band", type=float, nargs=2, default=OBSTACLE_BAND,
                   help="床上の障害物とみなす高さ帯[m]")
    p.add_argument("--dilate-free", type=float, default=0.30,
                   help="床セルをこの距離だけ広げて空きにする[m]。測り漏れを埋める")
    args = p.parse_args()

    cloud = o3d.io.read_point_cloud(str(args.pcd))
    points = np.asarray(cloud.points)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise SystemExit("有限な点が 0 です: {}".format(args.pcd))

    floor_z = float(np.percentile(points[:, 2], FLOOR_PERCENTILE))
    height = points[:, 2] - floor_z
    print("点 {:,} / 床の高さ z={:.3f}m".format(len(points), floor_z))

    obstacle = points[(height >= args.band[0]) & (height <= args.band[1])]
    floor = points[(height >= FLOOR_BAND[0]) & (height <= FLOOR_BAND[1])]
    print("  障害物帯 {:,} 点 / 床帯 {:,} 点".format(len(obstacle), len(floor)))

    lo = points[:, :2].min(axis=0) - args.resolution
    hi = points[:, :2].max(axis=0) + args.resolution
    size = np.ceil((hi - lo) / args.resolution).astype(int)
    width, height_cells = int(size[0]), int(size[1])
    print("  格子 {} x {} セル（{:.1f} x {:.1f} m）".format(
        width, height_cells, width * args.resolution, height_cells * args.resolution))

    def to_cells(xy: np.ndarray) -> np.ndarray:
        idx = np.floor((xy - lo) / args.resolution).astype(int)
        idx[:, 0] = np.clip(idx[:, 0], 0, width - 1)
        idx[:, 1] = np.clip(idx[:, 1], 0, height_cells - 1)
        return idx

    occupied = np.zeros((height_cells, width), dtype=bool)
    free = np.zeros_like(occupied)
    if len(obstacle):
        c = to_cells(obstacle[:, :2]); occupied[c[:, 1], c[:, 0]] = True
    if len(floor):
        c = to_cells(floor[:, :2]); free[c[:, 1], c[:, 0]] = True

    # 床は grazing 角でしか測れないので穴が空く。少しだけ広げて埋める
    if args.dilate_free > 0:
        radius = max(1, int(round(args.dilate_free / args.resolution)))
        free = ndimage.binary_dilation(free, iterations=radius)
    free &= ~occupied      # 障害物が勝つ

    image = np.full((height_cells, width), PGM_UNKNOWN, dtype=np.uint8)
    image[free] = PGM_FREE
    image[occupied] = PGM_OCCUPIED
    print("  占有 {:,} / 空き {:,} / 未知 {:,} セル".format(
        int(occupied.sum()), int(free.sum()),
        image.size - int(occupied.sum()) - int(free.sum())))

    # pgm は左上が原点。ROS の格子は左下が原点なので上下を反転して書く
    pgm_path = args.output.with_suffix(".pgm")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pgm_path, "wb") as handle:
        handle.write("P5\n{} {}\n255\n".format(width, height_cells).encode("ascii"))
        handle.write(np.flipud(image).tobytes())

    yaml_path = args.output.with_suffix(".yaml")
    yaml_path.write_text(
        "image: {}\n"
        "resolution: {}\n"
        "origin: [{:.4f}, {:.4f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n".format(pgm_path.name, args.resolution, lo[0], lo[1]))

    print("\n[OK] {} と {}".format(pgm_path, yaml_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
