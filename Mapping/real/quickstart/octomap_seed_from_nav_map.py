#!/usr/bin/env python3
"""**検証済みの 2D 地図**（`nav_map_run`）から `octomap_server` の種（`.bt`）を作る。

## なぜスキャンから作らないのか（2026-09-16 に実測して分かった）

計画（`docs/plan/2026-09-16-growing-map-octomap.md` 段 1）は「既存の 3D 地図
`map_octomap_r4_s5_floor0.pcd` から種を作る」としていた。**どちらも落ちる:**

| 種の作り方 | 占有 | クリア中央 | <0.30m | 壁の帯 | 机の帯 | 通れる wp |
|---|---|---|---|---|---|---|
| `nav_map_run`（現行・基準） | 20,404 | 0.728 m | 2.2 % | 97.3 % | 78.3 % | 14/14 |
| 事前地図 `r4_s5_floor0` をそのまま | 23,140 | **0.300 m** | **48.7 %** | 98.3 % | 80.2 % | **11/14** |
| 姿勢つきスキャンから作る（`octomap_seed_from_scans.py`）| 12,110 | **0.224 m** | **56.7 %** | **55.7 %** | 62.7 % | **11/14** |

- **測位が使っている事前地図には追従者が焼き込まれている。**ICP には無害だが
  2D に落とすと**自分の歩いた道を塞ぐ**（`g1_nav2.yaml` 冒頭の 09-08 の注記と同じ型）
- スキャンから作ると追従者に加えて**遠方の構造が入らない**（レイが `maxRange` 4 m
  しか届かないので壁の帯が 55.7 %。`nav_map_run` は SLAM 地図の全域を持つ）

⇒ **2D の系統（`nav_map_clean` → `nav_map_run`）だけが検証を通っている。**
09-08 から 09-14 にかけて、近距離除去・可視性除去・持続性フィルタ・ゴースト除去と
4 段の掃除と合否を積んだのはこちら側である。種はここから作る。

## どう作るか

`nav_map_run` の占有セルを足場にして、**高さだけ基準の 3D 点群から取る**。

- 占有: 基準点群のうち、帯の中にあり、かつ `nav_map_run` が占有と言うセルに落ちる点
  ⇒ 机は 0.23〜0.75 m の柱、壁は 0.23〜1.80 m の柱になる（**一様な押し出しにしない**）
- 基準点群が帯の中に点を持たないセルは、帯いっぱいに押し出す（数を印字する）
- 空き: `nav_map_run` の空きセルを帯いっぱいに空きで埋める
- 未知: `nav_map_run` の未知セルは木に入れない（＝未知のまま。地図の外へ経路を引かせない）

⚠️ **高さを持たせる理由。**一様な押し出しにすると、机が動いた跡を消すのに
**柱の全高さ**でレイが通る必要がある。実際の高さだけ立てておけば、机の高さを
通る光線だけで消える（段 6 の「60 秒以内」に直接効く）。

## 座標系（⚠️ 2 流派ある）

| 点群 | 床の z |
|---|---|
| `map_octomap_r4_s5_floor0.pcd`（測位が読む＝runtime の `map` 系） | **-0.002** |
| `map/ref/map_full_raw.txt`・ゴースト除去の点列 | **+0.076** |
| `map_octomap_clean.pcd`・`map_octomap_sim.pcd` | **-1.251** |

種は runtime の `map` 系＝**床が z≈0** で作る。基準点群はどれを渡しても、
この道具が最頻ビンで床を測って z≈0 に揃える（`--floor` で明示もできる）。
xy はどの流派でも一致している（`nav_map_run` と同じ格子で数えられる）。

## 合否の実測（2026-09-16・段 1 と段 3）

| | 占有 | クリア中央 | <0.30m | 壁の帯 | 机の帯 | 通れる wp |
|---|---|---|---|---|---|---|
| `nav_map_run`（基準） | 20,404 | 0.728 m | 2.2 % | 97.3 % | 78.3 % | 14/14 |
| **この種の 2D 投影** | **19,997** | **0.762 m** | **2.0 %** | **93.5 %** | **72.7 %** | **14/14** |

セル単位の重畳は **±1 セル許容で 99.9 %**（種→基準 19,986/19,997・基準→種 20,374/20,404）。
計画の段 1 合否「95 % 以上」を通る。

⚠️ **完全一致は 0 % になる。**`octomap` の格子は世界原点から解像度の整数倍に固定
（セル中心が `(k-32768+0.5)*res`）だが、`nav_map_run` の原点は
`(-12.696, -17.9279)` で整数倍ではない。**位相が (-0.004, +0.0279) m ずれる。**
これは静的レイヤを `map_server` から `octomap_server` に移す時点で避けられない。
壁の帯が 97.3 → 93.5 % に落ちるのはこの位相差が主因である
（種を y に ±0.10 m 動かすと 96.0 / 96.9 % になり、**片側に寄っていない**＝
ずれではなく位相の刻みの影響。この指標はこの地図では ±3 pt の鈍さがある）。

## 使い方

    G1_Hackason/.venv/bin/python quickstart/octomap_seed_from_nav_map.py \\
        runs/20260906T135940_UiS_room_v3

    # 作ったら必ず採点する
    G1_Hackason/.venv/bin/python quickstart/octomap_project_2d.py \\
        runs/<id>/map/seed.bt runs/<id>/map/seed_projected
    G1_Hackason/.venv/bin/python quickstart/score_nav_map.py \\
        runs/<id>/map/nav_map_run.yaml runs/<id>/map/seed_projected.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import octomap
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_map_clearance import load_map                               # noqa: E402
from pcd_to_occupancy import OBSTACLE_BAND                             # noqa: E402
from run_octomap import (DEFAULT_PROB_HIT, DEFAULT_PROB_MISS, DEFAULT_RESOLUTION,  # noqa: E402
                         DEFAULT_THRES_MAX, DEFAULT_THRES_MIN, build_tree,
                         estimate_floor)

DEFAULT_NAV_MAP = "nav_map_run.yaml"
# 高さを取る基準点群。**`nav_map_run` の元になった点群を渡すこと。**
# 2026-09-16 実測: 占有セル 20,404 のうち、帯の中に点があったセルは
#   `map_octomap_clean.pcd`（nav_map_clean の元）      19,216（94.2 %）← これを既定に
#   `ref/map_full_raw.txt`（mola_floor0 の別 SLAM 実行）10,768（52.8 %）
# 別の SLAM 実行の点群を渡すと半分のセルが高さを持てず、帯いっぱいの押し出しに落ちる。
DEFAULT_REFERENCE = "map_octomap_clean.pcd"


def read_points(path: Path) -> np.ndarray:
    """.txt（x y z のヘッダ付き）か .pcd を読む。"""
    if path.suffix == ".pcd":
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from g1_mapping.pcd_io import read_pcd
        return read_pcd(path).points
    return pd.read_csv(path, sep=r"\s+")[["x", "y", "z"]].to_numpy(np.float64)


def cell_of(xy: np.ndarray, grid) -> np.ndarray:
    """世界座標を nav_map の (行, 列) にする。範囲外は -1 を入れる。"""
    height, width = grid.shape
    col = np.floor((xy[:, 0] - grid.origin[0]) / grid.resolution).astype(np.int64)
    row = np.floor((xy[:, 1] - grid.origin[1]) / grid.resolution).astype(np.int64)
    outside = (col < 0) | (col >= width) | (row < 0) | (row >= height)
    col[outside] = row[outside] = -1
    return np.c_[row, col]


def column(cells: np.ndarray, grid, tops: np.ndarray, band: "tuple[float, float]",
           resolution: float) -> np.ndarray:
    """セルごとに、帯の下端から `tops` までの柱のボクセル中心を並べる。"""
    steps = np.arange(band[0], band[1] + resolution / 2, resolution)
    x = grid.origin[0] + (cells[:, 1] + 0.5) * grid.resolution
    y = grid.origin[1] + (cells[:, 0] + 0.5) * grid.resolution
    keep = steps[None, :] <= tops[:, None]
    keep[:, 0] = True                      # 下端の 1 枚は必ず立てる
    rows = np.repeat(np.arange(len(cells)), keep.sum(axis=1))
    return np.c_[x[rows], y[rows], np.tile(steps, len(cells))[keep.ravel()]]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("--nav-map", default=DEFAULT_NAV_MAP,
                   help=f"足場にする 2D 地図（<session>/map 配下。既定 {DEFAULT_NAV_MAP}）")
    p.add_argument("--reference", default=DEFAULT_REFERENCE,
                   help=f"高さを取る 3D 点群（<session>/map 配下。既定 {DEFAULT_REFERENCE}）")
    p.add_argument("--output", default="seed.bt", help="<session>/map 配下の出力名")
    p.add_argument("--band", type=float, nargs=2, default=OBSTACLE_BAND,
                   metavar=("MIN_Z", "MAX_Z"),
                   help=f"床上の帯[m]（既定 {OBSTACLE_BAND}＝min_obstacle_height と同じ）")
    p.add_argument("--floor", type=float, default=None,
                   help="基準点群の床の z[m]。省略すると最頻ビンで測る")
    p.add_argument("--no-free", action="store_true",
                   help="空きセルを木に入れない（占有だけの種にする）")
    p.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    a = p.parse_args()

    map_dir = a.session_dir / "map"
    grid = load_map(map_dir / a.nav_map)
    occupied_cells = np.argwhere(grid.occupied)
    print(f"足場 {a.nav_map}: 格子 {grid.shape[1]} x {grid.shape[0]} / "
          f"原点 {grid.origin} / 占有 {len(occupied_cells):,} セル")

    points = read_points(map_dir / a.reference)
    floor = a.floor if a.floor is not None else estimate_floor(points)
    points = points - np.array([0.0, 0.0, floor])
    print(f"基準 {a.reference}: {len(points):,} 点 / 床 z={floor:+.3f} m を 0 に揃えた")

    # 帯の中の点を nav_map のセルに落とし、占有セルの分だけ残す
    band = tuple(a.band)
    in_band = points[(points[:, 2] >= band[0]) & (points[:, 2] <= band[1])]
    cells = cell_of(in_band[:, :2], grid)
    on_occupied = (cells[:, 0] >= 0) & grid.occupied[cells[:, 0], cells[:, 1]]
    voxels = in_band[on_occupied]
    print(f"帯 {band[0]}〜{band[1]} m の点 {len(in_band):,} / "
          f"占有セルに落ちた点 {len(voxels):,}")

    # 基準点群が帯の中に点を持たない占有セルは、帯いっぱいに押し出す
    have = set(map(tuple, cells[on_occupied].tolist()))
    missing = np.array([c for c in occupied_cells.tolist() if tuple(c) not in have])
    print(f"占有セル {len(occupied_cells):,} のうち "
          f"基準に点があった {len(have):,} / 無くて押し出す {len(missing):,}")

    tree = build_tree(a.resolution, DEFAULT_PROB_HIT, DEFAULT_PROB_MISS,
                      DEFAULT_THRES_MIN, DEFAULT_THRES_MAX)
    if len(missing):
        filled = column(missing, grid, np.full(len(missing), band[1]), band, a.resolution)
        tree.updateNodes(np.ascontiguousarray(filled), True)
        print(f"  押し出したボクセル {len(filled):,}")

    if not a.no_free:
        free_cells = np.argwhere(grid.free)
        filled = column(free_cells, grid, np.full(len(free_cells), band[1]),
                        band, a.resolution)
        tree.updateNodes(np.ascontiguousarray(filled), False)
        print(f"空き {len(free_cells):,} セル → ボクセル {len(filled):,}")

    # ⚠️ 占有は**最後**に入れる。空きの柱と重なるセルでは占有を勝たせる
    tree.updateNodes(np.ascontiguousarray(voxels), True)
    tree.updateInnerOccupancy()

    out = map_dir / a.output
    tree.writeBinary(str(out).encode())
    print(f"\n出力: {out}（{out.stat().st_size/1e6:.2f} MB）")

    back = octomap.OcTree(a.resolution)
    if not back.readBinary(str(out).encode()):
        raise SystemExit(f"[NG] 書いた .bt を読み戻せない: {out}")
    occ = free = 0
    zs = []
    for node in back.begin_leafs():
        weight = int(round((node.getSize() / a.resolution) ** 3))
        if back.isNodeOccupied(node):
            occ += weight
            zs.append(node.getZ())
        else:
            free += weight
    low, high = back.getMetricMin(), back.getMetricMax()
    print(f"読み戻し: 占有 {occ:,} / 空き {free:,} セル / "
          f"z {low[2]:.2f}〜{high[2]:.2f} m")

    zs = np.asarray(zs)
    checks = [
        ("占有ボクセルが帯の中だけ",
         bool(len(zs)) and zs.min() >= band[0] - a.resolution and zs.max() <= band[1] + a.resolution),
        ("空きセルがある（--no-free でないとき）", a.no_free or free > 0),
        ("押し出しに頼ったセルが占有セルの 25 % 未満",
         len(missing) < len(occupied_cells) * 0.25),
    ]
    print()
    for label, ok in checks:
        print(f"  {'✅' if ok else '❌'} {label}")
    if not all(ok for _, ok in checks):
        print("\n[NG] 種の検算に落ちた", file=sys.stderr)
        return 1
    print(f"\n[OK] 次: octomap_project_2d.py で 2D に落として score_nav_map.py で採点する")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
