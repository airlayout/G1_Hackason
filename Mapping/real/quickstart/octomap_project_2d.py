#!/usr/bin/env python3
"""`.bt` の OctoMap を、`octomap_server` の `/projected_map` と**同じ規則で** 2D に落とす。

## なぜ要るか

段 1〜4 の合否は「`/projected_map` が既存の `nav_map_run` と同じ品質か」である。
実機や再生を回さずに Mac だけで測れるようにする。段 2 で実ノードの
`/projected_map` を PGM に落として、こちらと突き合わせれば規則の写しが正しいか分かる。

## octomap_server の規則（2.3.1 の `update2DMap` / `handlePostNodeTraversal`）

- 葉ごとに **z が帯と重なるか**で判定する。中心が帯に入るかではない:
  `z + size/2 > occupancy_min_z` かつ `z - size/2 < occupancy_max_z`
- 重なった葉が**占有なら 100**。空きなら、そのセルがまだ未知(-1)のときだけ 0
  ⇒ **占有が必ず勝つ**
- prune で葉が解像度より粗くなっていることがあるので、覆う全セルに書く
- 格子は木の metric bbox から作る。セル境界は解像度の整数倍
  （`keyToCoord` が `(key - 32768 + 0.5) * res` を返すため）

⚠️ `occupancy_min_z` / `occupancy_max_z` の**既定は無制限**である。そのままだと
床と天井が入って部屋全体が障害物になる。必ず帯を渡す。

## 使い方

    G1_Hackason/.venv/bin/python quickstart/octomap_project_2d.py \\
        runs/<id>/map/seed.bt runs/<id>/map/seed_projected

    # 帯を変える（段 4 の掃引）
    ... --band 1.30 1.82

既定の帯 0.23〜1.80 m は `pcd_to_occupancy.py` の `OBSTACLE_BAND` と
`g1_nav2.yaml` の `min_obstacle_height` に合わせてある。**床が z≈0 の
`floor0` 系の地図にだけそのまま使える**（`make_octomap_seed.py` の既定はその系）。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import octomap

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pcd_to_occupancy import (OBSTACLE_BAND, PGM_FREE, PGM_OCCUPIED,  # noqa: E402
                             PGM_UNKNOWN, write_map)

UNKNOWN, FREE, OCCUPIED = -1, 0, 100      # nav_msgs/OccupancyGrid の値


def read_leaves(tree: octomap.OcTree) -> "tuple[np.ndarray, ...]":
    """葉を (x, y, z, size, occupied) の配列で取り出す。"""
    xs, ys, zs, ss, oc = [], [], [], [], []
    for node in tree.begin_leafs():
        xs.append(node.getX())
        ys.append(node.getY())
        zs.append(node.getZ())
        ss.append(node.getSize())
        oc.append(tree.isNodeOccupied(node))
    return (np.asarray(xs), np.asarray(ys), np.asarray(zs),
            np.asarray(ss), np.asarray(oc, dtype=bool))


def project(tree: octomap.OcTree, band: "tuple[float, float]",
            verbose: bool = True) -> "tuple[np.ndarray, np.ndarray, float]":
    """2D に落とす。戻り値は (OccupancyGrid の値の 2 次元配列, 左下の原点[m], 解像度)。

    配列は行 0 が y の最小側（ROS の格子と同じ向き）。`write_map` が上下を反転して書く。
    """
    res = float(tree.getResolution())
    low, high = tree.getMetricMin(), tree.getMetricMax()
    ix0, iy0 = int(np.floor(low[0] / res)), int(np.floor(low[1] / res))
    ix1, iy1 = int(np.floor(high[0] / res)), int(np.floor(high[1] / res))
    width, height = ix1 - ix0 + 1, iy1 - iy0 + 1
    grid = np.full((height, width), UNKNOWN, dtype=np.int16)

    x, y, z, size, occupied = read_leaves(tree)
    if verbose:
        print(f"葉 {len(x):,} / 格子 {width} x {height} セル "
              f"（{width*res:.1f} x {height*res:.1f} m）解像度 {res} m")

    # ⚠️ 中心ではなく「帯と重なるか」。粗い葉は中心が帯の外でも一部が帯に入る
    overlap = (z + size / 2 > band[0]) & (z - size / 2 < band[1])
    if verbose:
        print(f"帯 {band[0]}〜{band[1]} m に重なる葉 {int(overlap.sum()):,}")

    # 空き → 占有 の順に書くと「占有が勝つ」規則と同じ結果になる
    for want_occupied, value in ((False, FREE), (True, OCCUPIED)):
        pick = overlap & (occupied == want_occupied)
        if not pick.any():
            continue
        for leaf_size in np.unique(size[pick]):
            same = pick & (size == leaf_size)
            span = max(1, int(round(leaf_size / res)))
            base_x = np.round((x[same] - leaf_size / 2) / res).astype(int) - ix0
            base_y = np.round((y[same] - leaf_size / 2) / res).astype(int) - iy0
            # 葉 1 個が span x span セルを覆う。x と y の**全組み合わせ**を作る
            # （tile と repeat を取り違えると対角しか埋まらない）
            steps = np.arange(span)
            cols = (base_x[:, None] + np.tile(steps, span)[None, :]).ravel()
            rows = (base_y[:, None] + np.repeat(steps, span)[None, :]).ravel()
            inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            grid[rows[inside], cols[inside]] = value

    if verbose:
        for label, value in (("占有", OCCUPIED), ("空き", FREE), ("未知", UNKNOWN)):
            print(f"  {label} {int((grid == value).sum()):,} セル")
    return grid, np.array([ix0 * res, iy0 * res]), res


def to_pgm_image(grid: np.ndarray) -> np.ndarray:
    """OccupancyGrid の値を pgm の画素値にする。"""
    image = np.full(grid.shape, PGM_UNKNOWN, dtype=np.uint8)
    image[grid == FREE] = PGM_FREE
    image[grid == OCCUPIED] = PGM_OCCUPIED
    return image


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bt", type=Path, help="OctoMap の .bt")
    p.add_argument("output", type=Path, help="拡張子なしの出力名（.pgm と .yaml を作る）")
    p.add_argument("--band", type=float, nargs=2, default=OBSTACLE_BAND,
                   metavar=("MIN_Z", "MAX_Z"),
                   help=f"occupancy_min_z / occupancy_max_z[m]（既定 {OBSTACLE_BAND}）")
    p.add_argument("--resolution", type=float, default=0.1,
                   help="読むときの解像度。.bt のヘッダにある値と一致させる")
    a = p.parse_args()

    tree = octomap.OcTree(a.resolution)
    if not tree.readBinary(str(a.bt).encode()):
        raise SystemExit(f"読めません: {a.bt}")

    grid, lo, res = project(tree, tuple(a.band))
    pgm, yaml_path = write_map(to_pgm_image(grid), lo, res, a.output)
    print(f"\n[OK] {pgm} と {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
