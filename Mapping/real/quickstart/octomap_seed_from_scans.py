#!/usr/bin/env python3
"""姿勢つきスキャンから OctoMap（`.bt`）を作る。**種としては不採用**（測った結果）。

## ⚠️ 種づくりにはこれを使わない

2026-09-16 に作って採点したら落ちた。採用したのは
`octomap_seed_from_nav_map.py`（検証済みの 2D 地図から作る方）である。

| | 占有 | クリア中央 | <0.30m | 壁の帯 | 机の帯 | 通れる wp |
|---|---|---|---|---|---|---|
| `nav_map_run`（基準） | 20,404 | 0.728 m | 2.2 % | 97.3 % | 78.3 % | 14/14 |
| **これで作った種** | 12,110 | **0.224 m** | **56.7 %** | **55.7 %** | 62.7 % | **11/14** |

落ちた理由は 2 つ。どちらもレイキャストの性質からくる:

1. **追従者が消えない。**軌跡から 0.30 m 未満の 806 セルが残り、高さ中央 1.05 m
   （1.25〜1.50 m に 29.5 %）＝人である。機体と一緒に動くので**後から光線が通らず**、
   OctoMap の可視性除去が効かない。近距離除去（`filter_scans_near.py`）が要る
2. **遠方の構造が入らない。**`maxRange` 4 m のレイが届いた所しか木に入らないので
   範囲が x -7.1〜22.4 / y -14.8〜22.6 m にしかならず、`nav_map_run`（78.0 x 43.1 m）
   に在る壁が 11,863 セル足りない

## それでも残してある理由

**3D の自由空間（空き・未知の区別）を作れるのはこれだけ**である。段 4 で
`maxRange` を掃引して「天井と遠方構造の誤除去」を見るときにも要る
（`run_octomap.py` は木を捨ててしまう）。

## なぜ要るか

`octomap_server` の `/projected_map` は**走りながら育つ**ので、起動直後は部屋の
全体像が無い。まだ見ていない場所へは経路が引けない（`g1_nav2.yaml` の
`static_layer` が事前地図を選んでいるのは、まさにこの理由）。
`octomap_path` に `.bt` を渡せば、過去に作った地図から始めて**そこから育てられる**。

## なぜ地図の PCD からではなく、姿勢つきスキャンから作るのか

配備済みの `map_octomap_r4_s5_floor0.pcd` を点として `updateNode` で入れると、
**占有セルしか持たない木**になる。それだと

- 部屋の内側が全部「未知」になり、`/projected_map` が空きを出さない
- Nav2 の静的レイヤは未知を通してしまうので、地図が経路を拘束しなくなる

`run_octomap.py` の投入処理（`insertPointCloud`）はレイキャストするので、
**占有・空き・未知の 3 値が正しく入る**。同じ処理を呼んで、木そのものを書き出す。
`run_octomap.py` は木を掃除の判定にだけ使って捨てていた。

## 座標系（⚠️ ここを間違えると重畳が半分に出る）

配備済みの事前地図は **`*_floor0`**（床が z≈0）で、これは元の SLAM 出力を
**z に +1.247803 m** ずらしたものである（2026-09-16 に 728,052 点すべてで実測。
xy の差は 0.0、z の差は 1.2478028〜1.2478031 m）。
姿勢つきスキャンは**ずらす前**の系なので、`--z-offset` で揃える。

## 使い方

    G1_Hackason/.venv/bin/python quickstart/make_octomap_seed.py \\
        runs/20260906T135940_UiS_room_v3 --benchmark-dir benchmark_s5

既定は配備済みの事前地図 `map_octomap_r4_s5_floor0.pcd` と同じ条件
（`benchmark_s5` の 1135 枚・`maxRange` 4.0 m・解像度 0.1 m）にしてある。
**測位が使う地図と種を同じ条件で作る**ことで、食い違いの原因を 1 つ減らす。

⚠️ `maxRange` を変えるときは `octomap_project_2d.py` → `score_nav_map.py` まで
通して採点する。無制限にすると天井の 57 % が消える（2026-09-06 実測）。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import octomap

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_octomap import (DEFAULT_MAX_RANGE, DEFAULT_PROB_HIT, DEFAULT_PROB_MISS,  # noqa: E402
                         DEFAULT_RESOLUTION, DEFAULT_THRES_MAX, DEFAULT_THRES_MIN,
                         build_tree, insert_scans)

# 配備済みの `*_floor0.pcd` が、元の SLAM 出力から z にずれている量[m]。
# 2026-09-16 に map_octomap_r4_s5.pcd と map_octomap_r4_s5_floor0.pcd の
# 全 728,052 点で実測（xy の差 0.0 / z の差 1.2478028〜1.2478031）。
FLOOR0_Z_OFFSET = 1.247803
# 配備済みの事前地図を作ったスキャン束。r4_s5 = benchmark_s5 の 1135 枚 / maxRange 4.0
DEFAULT_BENCHMARK = "benchmark_s5"


def summarize(tree: octomap.OcTree, label: str) -> dict:
    """木の中身を数える。葉を全部なめるので数秒かかる。"""
    occupied_z: list[float] = []
    occupied = free = 0
    for node in tree.begin_leafs():
        # 葉は解像度より粗いことがある（prune 済み）。その分の重みを持たせる
        weight = int(round((node.getSize() / tree.getResolution()) ** 3))
        if tree.isNodeOccupied(node):
            occupied += weight
            occupied_z.append(float(node.getCoordinate()[2]))
        else:
            free += weight
    low, high = tree.getMetricMin(), tree.getMetricMax()
    z = np.asarray(occupied_z)
    counts, edges = np.histogram(z[z < np.percentile(z, 50.0)], bins=100) if len(z) else ([0], [0, 0])
    floor_z = float(edges[int(np.argmax(counts))] + (edges[1] - edges[0]) / 2) if len(z) else float("nan")
    print(f"[{label}] 葉 {tree.getNumLeafNodes():,} / ノード {tree.size():,} / "
          f"メモリ {tree.memoryUsage()/1e6:.0f} MB")
    print(f"[{label}] 占有 {occupied:,} セル / 空き {free:,} セル")
    print(f"[{label}] 範囲 x {low[0]:.2f}〜{high[0]:.2f} / y {low[1]:.2f}〜{high[1]:.2f} / "
          f"z {low[2]:.2f}〜{high[2]:.2f} m")
    print(f"[{label}] 占有セルの最頻 z（＝床とみなせる高さ）= {floor_z:+.3f} m")
    return dict(occupied=occupied, free=free, floor_z=floor_z,
                leafs=int(tree.getNumLeafNodes()), nodes=int(tree.size()))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("--benchmark-dir", default=DEFAULT_BENCHMARK,
                   help=f"姿勢つき PCD の置き場（<session> 配下の名前。既定 {DEFAULT_BENCHMARK}）")
    p.add_argument("--output", default="seed.bt",
                   help="<session>/map/ 配下の出力名（既定 seed.bt）")
    p.add_argument("--stride", type=int, default=1,
                   help="何枚に 1 枚投入するか。benchmark_s5 は既に間引き済みなので既定 1")
    p.add_argument("--limit", type=int, default=0, help="投入する枚数の上限（0 で全部）")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE,
                   help=f"レイを伸ばす上限[m]（既定 {DEFAULT_MAX_RANGE}＝配備済み地図と同じ）")
    p.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    p.add_argument("--z-offset", type=float, default=FLOOR0_Z_OFFSET,
                   help=f"スキャンを z にずらす量[m]。配備済みの floor0 系に揃える"
                        f"（既定 {FLOOR0_Z_OFFSET}）。0 を渡すと元の SLAM 系のまま")
    p.add_argument("--prob-hit", type=float, default=DEFAULT_PROB_HIT)
    p.add_argument("--prob-miss", type=float, default=DEFAULT_PROB_MISS)
    p.add_argument("--thres-min", type=float, default=DEFAULT_THRES_MIN)
    p.add_argument("--thres-max", type=float, default=DEFAULT_THRES_MAX)
    a = p.parse_args()

    pcd_dir = a.session_dir / a.benchmark_dir / "pcd"
    if not pcd_dir.is_dir():
        raise SystemExit(f"姿勢つき PCD がありません: {pcd_dir}")

    tree = build_tree(a.resolution, a.prob_hit, a.prob_miss, a.thres_min, a.thres_max)
    print(f"解像度={a.resolution}m probHit={a.prob_hit} probMiss={a.prob_miss} "
          f"閾値=[{a.thres_min},{a.thres_max}] maxRange={a.max_range} zOffset={a.z_offset:+.6f}")

    began = time.time()
    frames, points = insert_scans(tree, pcd_dir, a.stride, a.limit, a.max_range, a.z_offset)
    tree.updateInnerOccupancy()
    print(f"投入おわり {(time.time()-began)/60:.1f} 分 / {frames} 枚 / {points:,} 点\n")

    stats = summarize(tree, "木")

    out = a.session_dir / "map" / a.output
    out.parent.mkdir(parents=True, exist_ok=True)
    # ⚠️ writeBinary は toMaxLikelihood 相当を通すので、確率が上下の閾値に張り付く。
    # 占有セルは log-odds 3.476 から出発し、probMiss 0.4（-0.405/回）なら
    # **9 回通り抜けられれば空きに転ぶ**。幽霊が消える速さはこれで決まる。
    tree.writeBinary(str(out).encode())
    print(f"\n出力: {out}（{out.stat().st_size/1e6:.1f} MB）")

    # 読み戻して、書けたものが同じか確かめる（octomap_server が読むのはこちら）
    back = octomap.OcTree(a.resolution)
    if not back.readBinary(str(out).encode()):
        raise SystemExit(f"[NG] 書いた .bt を読み戻せない: {out}")
    reread = summarize(back, "読み戻し")

    checks = [
        ("占有セルが一致", reread["occupied"] == stats["occupied"]),
        ("空きセルが一致", reread["free"] == stats["free"]),
        ("床が z≈0（|z| < 0.10 m）", abs(reread["floor_z"]) < 0.10),
        ("空きセルが占有セルより多い（自由空間が入っている）", reread["free"] > reread["occupied"]),
    ]
    print()
    for label, ok in checks:
        print(f"  {'✅' if ok else '❌'} {label}")
    if not all(ok for _, ok in checks):
        print("\n[NG] 種の検算に落ちた", file=sys.stderr)
        return 1
    print(f"\n[OK] octomap_server に渡す: -p octomap_path:={out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
