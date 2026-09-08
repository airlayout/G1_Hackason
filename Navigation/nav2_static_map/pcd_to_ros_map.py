#!/usr/bin/env python3
"""静的地図(PCD)をROS2 Nav2の`map_server`が読める形式(`.yaml`+`.pgm`)に変換する。

## なぜ必要か

`Navigation/nav2/g1_nav2.yaml`のglobal_costmapには既に`static_layer`が設定済み
（`map_topic: /map`から読む前提のコメント付き）だが、それを配信する`map_server`は
`Mapping/real/quickstart/nav_stack.sh`で一度も起動されていない。地図ファイル自体も
存在しない。本ツールはその地図ファイルを作る（`nav2_static_map/README.md`の設計参照）。

## `nav/occupancy.py`を呼ばず、3値で自分で作る理由

`nav/occupancy.py`の`build_grid()`は「通れない」を1つのbool（障害物 or 未観測の外側）に
まとめている。これはA*経路探索(`nav/route.py`)が「未知」という概念を持たないための
安全側の設計判断であり、正しい。

しかしROSの`map_server`形式は本来**自由・占有・未知の3値**を区別でき、
`g1_nav2.yaml`の`track_unknown_space: true`もこの3値を前提にしている。
2値に潰すと「まだ見ていないだけの場所」と「実際の壁」の区別が失われ、
Nav2自身が持つ「未知空間の扱い方」の設定（プランナーが未知を通ってよいか等）を
生かせなくなる。そこで本ツールは`nav/occupancy.py`を**変更せず**、
高さ帯・解像度の定数だけ再利用して、3値の判定を独自に行う。

## 座標系についての前提（⚠️ 未検証）

`g1_nav2.yaml`は`map -> odom`を恒等変換にしている（内蔵SLAMの`odom`をそのまま
map系として使う）。したがって**この地図が、実際にNav2を走らせるセッションの
odomフレームと同じ原点を共有していること**が前提になる。

`map_20260907.pcd`は`room_a`セッション自身のSLAM出力から作られているため、
`room_a`の生rosbagを再生する場合は原点が一致するはず。**別セッションの再生や
実機ライブでは座標系がずれる可能性があり、まだ確認していない。**

## ⚠️ 高さは「床からの相対高さ」で判定する（2026-09-07に修正）

`nav/occupancy.py`の`DEFAULT_OBSTACLE_Z_MIN/MAX`（0.30〜1.80m）は**絶対座標のZ**への
閾値で、Z=0が床にある前提（コメントに「床（実測でz≒0）」とある）。しかし
`map_20260907.pcd`は`Mapping`班のSLAM出力をそのまま使っており、**床は絶対Z≈-1.30m、
天井は絶対Z≈+1.50m**（実測: 床±0.07mの範囲に全体の36.7%、天井付近に16.5%が集中）。

この2つの閾値をそのまま適用すると、「床から0.3〜1.8m」ではなく**「床から1.6〜3.1m」
（天井を突き抜けた範囲）を見てしまい、机・椅子はほとんど拾えず、
代わりに天井の点(全体の約16%)を「障害物」として大量に誤検出していた。**
初回の検証画像・閾値実験(README「占有判定の閾値実験」節)はこのバグの影響下で
行ったものなので、傾向（1点ノイズが多いこと自体）は正しいが、
「何を障害物として数えていたか」は誤っていた。

本ツールは以後、点群自体から床を検出して**床からの相対高さ**で判定する
（`find_floor()`、他のMapping班ツールと同じヒストグラムのピーク検出）。
`--z-min`/`--z-max`は**床からの高さ[m]**として扱う。

## 使い方

    python3 pcd_to_ros_map.py map_20260907.pcd room_a_map
    # -> room_a_map.pgm / room_a_map.yaml ができる

    # 点群の外れ値（今回の計測は x が最大52mまで伸びる外れ値を含む）で
    # 地図が無駄に巨大にならないよう、範囲を手動で絞れる
    python3 pcd_to_ros_map.py map_20260907.pcd room_a_map --bounds -6 20 -18 14
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav.occupancy import DEFAULT_RESOLUTION_M, load_points  # noqa: E402

# 床からの相対高さ[m]。絶対Zではない（下の「高さは床からの相対高さで判定する」参照）。
# 下限0.10: 床に落ちた小物・ケーブル等の厚みは拾わない。什器の脚はここから掛かる
# 上限1.50: 人の頭・天井際を除く。実測の天井は床上約2.8mなので余裕を持たせてある
DEFAULT_OBSTACLE_HEIGHT_MIN = 0.10
DEFAULT_OBSTACLE_HEIGHT_MAX = 1.50


def find_floor(z: np.ndarray) -> float:
    """Zヒストグラムの下半分の最頻ビンを床とみなす（Mapping班の各ツールと同じ考え方）。"""

    low, high = np.percentile(z, [1.0, 99.0])
    core = z[(z >= low) & (z <= high)]
    if len(core) < 100:
        core = z
    hist, edges = np.histogram(core, bins=80)
    centers = (edges[:-1] + edges[1:]) / 2.0
    middle = (centers[0] + centers[-1]) / 2.0
    lower = centers < middle
    return float(centers[lower][np.argmax(hist[lower])])

# ROS map_serverの標準的な値（turtlebot3/nav2のサンプル地図と同じ配色）。
PIXEL_FREE = 254
PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("output_stem", type=Path,
                        help="出力の拡張子なしパス（<stem>.pgm / <stem>.yaml を作る）")
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION_M,
                        help=f"1セルの一辺[m]（既定{DEFAULT_RESOLUTION_M}、"
                             "global_costmapの解像度と揃えてある）")
    parser.add_argument("--z-min", type=float, default=DEFAULT_OBSTACLE_HEIGHT_MIN,
                        help=f"障害物とみなす床からの最小高さ[m]（既定{DEFAULT_OBSTACLE_HEIGHT_MIN}）")
    parser.add_argument("--z-max", type=float, default=DEFAULT_OBSTACLE_HEIGHT_MAX,
                        help=f"障害物とみなす床からの最大高さ[m]（既定{DEFAULT_OBSTACLE_HEIGHT_MAX}）")
    parser.add_argument("--bounds", type=float, nargs=4, default=None,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
                        help="地図の範囲を手動指定する。省略すると点群のbounding box全体を使うが、"
                             "外れ値混入時は地図が無駄に巨大かつスカスカになるので注意")
    parser.add_argument("--fill-interior-unknown", action="store_true",
                        help="内部の孤立した未観測（死角）も自由として塗りつぶす。"
                             "既定では未知のまま残し、Nav2自身の判断に委ねる")
    parser.add_argument("--occupied-min-points", type=int, default=1,
                        help="高さ帯内の点がこの数以上あるセルだけを占有とみなす（既定1=1点でも占有）。"
                             "床に落ちた小物・什器の縁のノイズ1点で塞がるのを防ぐ")
    return parser.parse_args()


def build_three_state_grid(
    points: np.ndarray, *, resolution: float, z_min: float, z_max: float,
    bounds: "tuple[float, float, float, float] | None",
    fill_interior_unknown: bool, occupied_min_points: int = 1,
) -> "tuple[np.ndarray, float, float]":
    """点群から (状態配列, origin_x, origin_y) を返す。

    状態配列の値は PIXEL_FREE / PIXEL_OCCUPIED / PIXEL_UNKNOWN のいずれか。
    配列の行0が画像の最上段＝world座標のy最大に対応する（ROSのPGM規約）。
    """

    if bounds is not None:
        x_min, x_max, y_min, y_max = bounds
    else:
        x_min, x_max = float(points[:, 0].min()), float(points[:, 0].max())
        y_min, y_max = float(points[:, 1].min()), float(points[:, 1].max())
        extent = max(x_max - x_min, y_max - y_min)
        if extent > 40.0:
            print(f"[warn] 点群のbounding boxが{extent:.1f}mと非常に大きい。"
                  "外れ値が混じっている可能性が高い。--boundsで範囲を絞ることを推奨", file=sys.stderr)

    width = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1
    print(f"[grid] 範囲 x[{x_min:.2f},{x_max:.2f}] y[{y_min:.2f},{y_max:.2f}] "
          f"-> {width}x{height}セル（{resolution}m/セル）")

    in_bounds = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max))
    pts = points[in_bounds]
    print(f"[grid] 範囲内の点: {len(pts)}/{len(points)}")
    if len(pts) == 0:
        raise SystemExit("指定範囲に点が1つもありません")

    floor_z = find_floor(pts[:, 2])
    print(f"[grid] 床 Z(絶対座標)={floor_z:+.3f}m と推定。障害物判定は床上{z_min}〜{z_max}m（相対高さ）")

    col = np.floor((pts[:, 0] - x_min) / resolution).astype(np.int64)
    row = np.floor((y_max - pts[:, 1]) / resolution).astype(np.int64)  # 行0=y最大
    col = np.clip(col, 0, width - 1)
    row = np.clip(row, 0, height - 1)

    observed = np.zeros((height, width), dtype=bool)
    observed[row, col] = True  # 高さに関係なく点があった＝観測済み（床の点がここで効く）

    height_above_floor = pts[:, 2] - floor_z
    is_obstacle_pt = (height_above_floor >= z_min) & (height_above_floor <= z_max)
    # セルごとの点数を数え、閾値以上のセルだけを占有にする（1点のノイズで塞がらないように）。
    obstacle_count = np.zeros((height, width), dtype=np.int32)
    np.add.at(obstacle_count, (row[is_obstacle_pt], col[is_obstacle_pt]), 1)
    obstacle = obstacle_count >= occupied_min_points

    unknown = ~observed
    if fill_interior_unknown:
        # 外周とつながる未観測だけ残し、内部の孤立した死角は自由に倒す。
        # nav/occupancy.pyの「外周だけ塞ぐ」とは向きが逆（あちらは占有側を塗る）。
        outside = ~ndimage.binary_fill_holes(observed)
        unknown = outside

    grid = np.full((height, width), PIXEL_UNKNOWN, dtype=np.uint8)
    grid[observed & ~obstacle] = PIXEL_FREE
    grid[obstacle] = PIXEL_OCCUPIED
    grid[unknown] = PIXEL_UNKNOWN  # 最後にunknownで確定（obstacle判定より優先）

    free_n = int((grid == PIXEL_FREE).sum())
    occ_n = int((grid == PIXEL_OCCUPIED).sum())
    unk_n = int((grid == PIXEL_UNKNOWN).sum())
    total = grid.size
    print(f"[grid] 自由 {free_n} ({100*free_n/total:.1f}%) / "
          f"占有 {occ_n} ({100*occ_n/total:.1f}%) / "
          f"未知 {unk_n} ({100*unk_n/total:.1f}%)")

    return grid, x_min, y_min


def write_pgm(path: Path, grid: np.ndarray) -> None:
    """P5 (binary) PGMをstdlibだけで書く。"""

    height, width = grid.shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + grid.tobytes())


def write_yaml(path: Path, pgm_name: str, resolution: float,
               origin_x: float, origin_y: float) -> None:
    """map_serverが読むYAML。originは最下段(行=height-1)左端セルのworld座標。"""

    text = (
        f"image: {pgm_name}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: {OCCUPIED_THRESH}\n"
        f"free_thresh: {FREE_THRESH}\n"
    )
    path.write_text(text, encoding="ascii")


def main() -> None:
    args = parse_args()
    points = load_points(args.pcd)
    print(f"[pcd] {args.pcd} -> {len(points)}点")

    bounds = tuple(args.bounds) if args.bounds is not None else None
    grid, origin_x, origin_y = build_three_state_grid(
        points, resolution=args.resolution, z_min=args.z_min, z_max=args.z_max,
        bounds=bounds, fill_interior_unknown=args.fill_interior_unknown,
        occupied_min_points=args.occupied_min_points,
    )

    pgm_path = args.output_stem.with_suffix(".pgm")
    yaml_path = args.output_stem.with_suffix(".yaml")
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name, args.resolution, origin_x, origin_y)
    print(f"[OUTPUT] {pgm_path}")
    print(f"[OUTPUT] {yaml_path}")


if __name__ == "__main__":
    main()
