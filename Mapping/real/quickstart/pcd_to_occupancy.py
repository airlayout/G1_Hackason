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

作った地図は必ず `check_map_clearance.py` にかける。**軌跡が自分の歩いた道を
塞いでいないか**は、地図を眺めても分からない（2026-09-08 に実際に嵌まった）。

    quickstart/check_map_clearance.py runs/<id>/map/nav_map runs/<id>/mola_floor0/traj.txt

⚠️ 出力名にドットを入れない（`with_suffix` が `band0.15` を `band0.pgm` にする）。
⚠️ `--band 0.15 1.80` が 2026-09-06 までの既定だった。当時の nav_map を再現するときは
   明示的に渡すこと（既定は `min_obstacle_height` に合わせて 0.23 にした）。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage

FLOOR_PERCENTILE = 5.0      # 床の高さに使う分位。低い側から取る
# ⚠️ 下限は `Navigation/nav2/g1_nav2.yaml` の `min_obstacle_height` と**同じ値**にする。
# 静的地図の帯だけ低いと、静的レイヤが床を障害物として撃つ一方、costmap の
# 観測レイヤは撃たない、という食い違いが起きる。2026-09-08 の実測:
# 下限 0.15 の地図では軌跡の 60.4% が占有セルから 0.30m 未満だったが、
# 0.23 にすると 1.9% になった（消えたのは全部床だった）。
# なぜ 0.15 では足りないか: FLOOR_PERCENTILE の床推定は真の床（最頻ビン）より
# 0.06〜0.07m 低く出る。さらに床自体が最大 +0.126m うねっている
# （check_calibration.py で実測）。0.15 は実質 0.08m しか確保できていなかった。
OBSTACLE_BAND = (0.23, 1.80)  # 床上のこの帯を障害物とする
# ⚠️ **帯の中の点数**がこれ未満のセルは占有にしない。
# 1（＝1 点で占有）だと、床のうねりが帯の下端をかすめた所が障害物になる。
# 2026-09-08 の実測: 長距離の経路を塞いでいたセルは点 25 個のうち
# **帯の中が 2 点だけ**で、その 2 点は 0.23 / 0.24 m ＝ 帯の下端ちょうど。
# 残りは床（0.10〜0.24 m が 14 点）と天井（2.78〜2.95 m が 11 点）だった。
# ⚠️ **全高さの点数で数えると見つからない**（このセルは合計 25 点あり「濃い」）。
# 数えるのは帯の中だけである。
# ⚠️ **既定は 1（＝従来の挙動）にしてある。**上げるのは実測してからにすること。
# 2026-09-09 に 1 / 2 / 3 を比べた結果、上げると副作用が勝った:
#
# | min_points | 占有 | sim に在って地図に無い（危険） | 経路が壁を抜ける | infl 0.55 の最狭部 |
# |---|---|---|---|---|
# | 1（既定） | 19,464 | **119 件** | 0 | **0.57 m** |
# | 2 | 13,672 | 196 件 | 0 | 0.45 m |
# | 3 | 11,307 | 307 件 | 3 セル | 0.50 m |
#
# 上げても最狭部は良くならず、**「Nav2 が空きだと思う所に物が在る」食い違いが増える**。
# 3 では機体が実際に当たった 0.75 m の物体まで消えた。
# 経路を広い側へ寄せたいなら、これではなく `inflation_radius` を上げる
# （tune_inflation.py。現状の地図でも 0.55 で最狭部 0.57 m になる）。
MIN_POINTS = 1
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
    p.add_argument("--min-points", type=int, default=MIN_POINTS,
                   help=f"帯の中の点数がこれ未満のセルは占有にしない（既定 {MIN_POINTS}）。"
                        "1 にすると 2026-09-08 以前の挙動")
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
        c = to_cells(obstacle[:, :2])
        # ⚠️ 「1 点でも在れば占有」にしない。帯の中の点数で足切りする
        counts = np.zeros((height_cells, width), dtype=np.int32)
        np.add.at(counts, (c[:, 1], c[:, 0]), 1)
        occupied = counts >= args.min_points
        thin = int(((counts > 0) & ~occupied).sum())
        print("  帯の中の点数が {} 未満で落としたセル {:,}".format(args.min_points, thin))
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
