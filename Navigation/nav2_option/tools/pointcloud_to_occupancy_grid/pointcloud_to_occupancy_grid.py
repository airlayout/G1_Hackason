"""点群地図(.pcd/.ply) から Nav2 map_server 形式(.pgm + .yaml)へ変換する。

Planning.md A-7・D-21 に対応。頭部LiDAR(MID-360)は足元〜約10m先の床が死角のため、
床面が点群に写らないことを前提に、高さフィルタで壁・柱・家具の側面だけを抽出して
Global costmap用の2D地図にする(Local costmapは別途3D点群ベースで運用する、D-21)。

未観測領域を「自由」と誤判定すると経路が地図の外側を回る問題が起きうる
(高さに関係なく点が1つも無いセルは「未知」であって「自由」ではない)。
これを避けるため、以下の3値で出力する:

    occupied(黒=0)   : 高さフィルタ範囲内に点があるセル
    free(白=254)     : 高さに関係なく点はあるが、フィルタ範囲内には無いセル
    unknown(灰=205)  : どの高さにも点が無いセル(観測されていない)

使い方:
    python pointcloud_to_occupancy_grid.py map.ply --resolution 0.05 \
        --min-height 0.3 --max-height 1.8 --out map

    → map.pgm と map.yaml を生成する(Nav2 map_server がそのまま読める)。
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


@dataclass
class GridResult:
    grid: np.ndarray  # shape (height, width), uint8。行0が最も小さいyに対応(後で反転して出力)
    resolution: float
    origin_x: float
    origin_y: float


def load_points(path: Path) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError(f"点群の読み込みに失敗した、または空だった: {path}")
    return np.asarray(pcd.points)


def build_occupancy_grid(
    points: np.ndarray,
    resolution: float,
    min_height: float,
    max_height: float,
    padding_m: float,
) -> GridResult:
    if points.shape[0] == 0:
        raise ValueError("点群が空")

    xy = points[:, :2]
    z = points[:, 2]

    min_x, min_y = xy.min(axis=0) - padding_m
    max_x, max_y = xy.max(axis=0) + padding_m

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    if width <= 0 or height <= 0:
        raise ValueError("グリッドサイズが不正(点群の広がりを確認すること)")

    def to_index(pts_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = np.floor((pts_xy[:, 0] - min_x) / resolution).astype(np.int64)
        iy = np.floor((pts_xy[:, 1] - min_y) / resolution).astype(np.int64)
        np.clip(ix, 0, width - 1, out=ix)
        np.clip(iy, 0, height - 1, out=iy)
        return ix, iy

    observed = np.zeros((height, width), dtype=bool)
    ix_all, iy_all = to_index(xy)
    observed[iy_all, ix_all] = True

    height_mask = (z >= min_height) & (z <= max_height)
    occ_mask = np.zeros((height, width), dtype=bool)
    if np.any(height_mask):
        ix_occ, iy_occ = to_index(xy[height_mask])
        occ_mask[iy_occ, ix_occ] = True

    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)
    grid[observed] = FREE
    grid[occ_mask] = OCCUPIED  # 観測済みより優先(occupied が free を上書きする)

    return GridResult(grid=grid, resolution=resolution, origin_x=min_x, origin_y=min_y)


def write_pgm(path: Path, grid: np.ndarray) -> None:
    """P5(binary)形式で書き出す。map_server慣例(原点=左下)に合わせ、行を上下反転して保存する。"""
    height, width = grid.shape
    flipped = np.flipud(grid)  # 画像は上が大きいy、mapのyamlはorigin=左下を仮定するため反転
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(flipped.tobytes())


def write_yaml(path: Path, pgm_name: str, result: GridResult) -> None:
    content = (
        f"image: {pgm_name}\n"
        f"resolution: {result.resolution}\n"
        f"origin: [{result.origin_x}, {result.origin_y}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    path.write_text(content, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="入力点群ファイル(.pcd / .ply 等、open3dが読める形式)")
    parser.add_argument("--out", type=Path, default=Path("map"), help="出力ファイル名の接頭辞(既定: map)")
    parser.add_argument("--resolution", type=float, default=0.05, help="グリッド解像度[m/cell](既定: 0.05)")
    parser.add_argument("--min-height", type=float, default=0.3, help="障害物とみなす最低高さ[m](既定: 0.3、D-21)")
    parser.add_argument("--max-height", type=float, default=1.8, help="障害物とみなす最高高さ[m](既定: 1.8、D-21)")
    parser.add_argument("--padding", type=float, default=1.0, help="点群の外周に足す余白[m](既定: 1.0)")
    args = parser.parse_args()

    points = load_points(args.input)
    print(f"[info] 読み込んだ点数: {points.shape[0]}")
    print(f"[info] z範囲: {points[:, 2].min():.3f} .. {points[:, 2].max():.3f}")

    result = build_occupancy_grid(
        points,
        resolution=args.resolution,
        min_height=args.min_height,
        max_height=args.max_height,
        padding_m=args.padding,
    )

    pgm_path = args.out.with_suffix(".pgm")
    yaml_path = args.out.with_suffix(".yaml")
    write_pgm(pgm_path, result.grid)
    write_yaml(yaml_path, pgm_path.name, result)

    n_occ = int(np.sum(result.grid == OCCUPIED))
    n_free = int(np.sum(result.grid == FREE))
    n_unknown = int(np.sum(result.grid == UNKNOWN))
    total = result.grid.size
    print(f"[info] グリッドサイズ: {result.grid.shape[1]} x {result.grid.shape[0]} " f"({result.resolution} m/cell)")
    print(f"[info] occupied={n_occ} ({100*n_occ/total:.1f}%) " f"free={n_free} ({100*n_free/total:.1f}%) " f"unknown={n_unknown} ({100*n_unknown/total:.1f}%)")
    print(f"[info] 書き出し: {pgm_path}, {yaml_path}")


if __name__ == "__main__":
    main()
