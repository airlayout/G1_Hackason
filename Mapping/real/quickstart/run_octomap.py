#!/usr/bin/env python3
"""OctoMap の可視性（レイキャスト）で、地図から動的な点を消す。

## なぜこれで消えるのか

高さと距離だけの幾何では、人の腰（0.75〜1.00m）と机の天板（0.70〜0.75m）を
分離できない（2026-09-04 に実測済み。除去帯の下限を 0.8m にすると机が 78% 削れる）。
OctoMap は違う証拠を使う。**「後からそこを光線が通り抜けた＝そのとき空だった」**。
人は動くので、居た場所はすぐ後の走査で光線が通り抜ける。机は通り抜けない。

## 何を使っているか

`octomap-python`（OctoMap 1.10 の公式バインディング）の `insertPointCloud` は、
DynamicMap Benchmark の `octomap_mapping/src/octomapper.cpp` が呼ぶのと同じ
OctoMap の更新処理である。`vendor/octomap_benchmark` の C++ 版は PCL を要求し、
Homebrew では VTK と Qt まで引きずって 4〜6GB になるため、この Mac の空き容量では
入らない。手法（被引用 2,887 の OctoMap）は同じで、入口だけ Python にしてある。

benchmark の `assets/config.toml`（地面フィルタ無し・ノイズフィルタ無し）に
既定値を合わせてある。地面フィルタは車載前提で、Mid-360 は垂直FOV −7° で床を
grazing 角でしか見ないため、屋内でそのまま効くとは限らない。まず素で走らせる。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/run_octomap.py \\
        runs/20260904T183457_UiS_room_v2 --stride 5

入力は `<session>/benchmark/pcd/*.pcd`（export_benchmark_data.py が作る姿勢つきPCD）。
出力は `<session>/map/map_octomap.pcd`（掃除後）と `map_octomap_removed.pcd`（消した点）。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import octomap

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd  # noqa: E402
from g1_mapping.rebuild import write_pcd  # noqa: E402

# 既定値は vendor/octomap_benchmark/assets/config.toml に合わせてある
DEFAULT_RESOLUTION = 0.1
DEFAULT_PROB_HIT = 0.7
DEFAULT_PROB_MISS = 0.4
DEFAULT_THRES_MIN = 0.12
DEFAULT_THRES_MAX = 0.97
# maxRange だけ benchmark の config.toml（-1 ＝ 無制限）から変えてある。
# 無制限だと Mid-360 が床・天井を grazing 角でしか見ないせいで長い光線が
# 静止構造の voxel を舐めて「空」に投票し、天井の 56.7%・遠方構造の 31.6% が消えた。
# 2〜10m を実測して掃引した結果、3m が「遠方構造と天井の誤除去が 0.0% を保てる
# 最大の範囲」だった（そのとき軌跡上の動的点は 92.4% 消える）。詳細は eval_removal.py。
DEFAULT_MAX_RANGE = 3.0

LABEL_UNKNOWN, LABEL_FREE, LABEL_OCCUPIED = -1, 0, 1


def build_tree(resolution: float, prob_hit: float, prob_miss: float,
               thres_min: float, thres_max: float) -> octomap.OcTree:
    tree = octomap.OcTree(resolution)
    tree.setProbHit(prob_hit)
    tree.setProbMiss(prob_miss)
    tree.setClampingThresMin(thres_min)
    tree.setClampingThresMax(thres_max)
    return tree


def insert_scans(tree: octomap.OcTree, pcd_dir: Path, stride: int, limit: int,
                 max_range: float) -> "tuple[int, int]":
    """姿勢つきPCDを順に投入する。(投入した枚数, 投入した点数) を返す。"""
    files = sorted(pcd_dir.glob("*.pcd"))
    if not files:
        raise SystemExit(f"PCD がありません: {pcd_dir}。先に export_benchmark_data.py を回すこと")
    chosen = files[::stride]
    if limit:
        chosen = chosen[:limit]
    print(f"{len(files)} 枚のうち {len(chosen)} 枚を投入する（stride={stride}）")

    began = time.time()
    inserted_points = 0
    for index, path in enumerate(chosen, start=1):
        data = read_pcd(path)
        if len(data.points) == 0:
            continue
        tree.insertPointCloud(data.points, data.origin, max_range, False)
        inserted_points += len(data.points)
        if index % 50 == 0 or index == len(chosen):
            elapsed = time.time() - began
            remain = elapsed / index * (len(chosen) - index)
            print(f"  {index}/{len(chosen)} 枚 / {inserted_points:,} 点 / "
                  f"{index/elapsed:.2f} 枚每秒 / 残り {remain/60:.1f} 分 / "
                  f"ノード {tree.size():,}", flush=True)
    return len(chosen), inserted_points


def classify(tree: octomap.OcTree, points: np.ndarray, chunk: int = 200_000) -> np.ndarray:
    """地図の各点を占有(1)/空(0)/未知(-1)に分ける。"""
    labels = np.empty(len(points), dtype=np.int32)
    for start in range(0, len(points), chunk):
        stop = min(start + chunk, len(points))
        labels[start:stop] = tree.getLabels(points[start:stop])
    return labels


def report_by_height(points: np.ndarray, removed: np.ndarray, floor_z: float) -> "list[str]":
    """床上の高さ帯ごとに、どれだけ消えたかを出す。

    追従者の下半身は床上 0.25〜1.05m（特に 0.65〜0.85m）に残っていた。
    机の天板は 0.70〜0.75m。ここが分離できたかどうかが評価の要点である。
    """
    lines = [f"床の高さ z={floor_z:.3f} m を基準にした高さ帯ごとの除去率",
             f"  {'高さ[m]':>12s} {'元の点数':>10s} {'消した点数':>10s} {'除去率':>8s}"]
    heights = points[:, 2] - floor_z
    edges = [-0.2, 0.25, 0.45, 0.65, 0.85, 1.05, 1.25, 1.5, 1.8, 2.2, 3.5]
    for low, high in zip(edges[:-1], edges[1:]):
        band = (heights >= low) & (heights < high)
        total = int(band.sum())
        if total == 0:
            continue
        gone = int((band & removed).sum())
        lines.append(f"  {low:5.2f}〜{high:5.2f} {total:10,d} {gone:10,d} {100*gone/total:7.1f}%")
    return lines


def estimate_floor(points: np.ndarray) -> float:
    """z の下側の最頻ビンを床とみなす。

    範囲の下半分で切ると外れ値に引きずられる（map_raw.pcd は下端が −5.77m まで
    伸びており、その中点で切ると本当の床 −1.33m を取りこぼす）。
    分位で切ってから最頻ビンを探す。
    """
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    if len(lower) == 0:
        return float(z.min())
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="OctoMap で地図から動的な点を消す")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--map", type=Path, default=None,
                        help="掃除する地図（既定 <session>/map/map_raw.pcd）")
    parser.add_argument("--output", default="map_octomap.pcd", help="map/ 配下の出力名")
    parser.add_argument("--stride", type=int, default=1, help="何枚に1枚投入するか（既定 1）")
    parser.add_argument("--limit", type=int, default=0, help="投入する枚数の上限（0で全部）")
    parser.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE,
                        help=f"レイを伸ばす上限[m]（-1で制限なし。既定 {DEFAULT_MAX_RANGE}）")
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument("--prob-hit", type=float, default=DEFAULT_PROB_HIT)
    parser.add_argument("--prob-miss", type=float, default=DEFAULT_PROB_MISS)
    parser.add_argument("--thres-min", type=float, default=DEFAULT_THRES_MIN)
    parser.add_argument("--thres-max", type=float, default=DEFAULT_THRES_MAX)
    parser.add_argument("--drop-unknown", action="store_true",
                        help="未知（光線が一度も通らなかった）の点も消す。"
                             "benchmark の saveMap はこちらの挙動だが、"
                             "向こうは投入したスキャンをそのまま地図にするので未知が出ない。"
                             "こちらは既存の map_raw.pcd を掃除するため未知が出るので、既定では残す")
    args = parser.parse_args()

    session = args.session_dir
    map_path = args.map or session / "map" / "map_raw.pcd"
    if not map_path.exists():
        raise SystemExit(f"地図が見つかりません: {map_path}")

    tree = build_tree(args.resolution, args.prob_hit, args.prob_miss,
                      args.thres_min, args.thres_max)
    print(f"OctoMap 解像度={args.resolution}m probHit={args.prob_hit} probMiss={args.prob_miss} "
          f"閾値=[{args.thres_min},{args.thres_max}] maxRange={args.max_range}")

    began = time.time()
    frames, points_in = insert_scans(tree, session / "benchmark" / "pcd",
                                     args.stride, args.limit, args.max_range)
    tree.updateInnerOccupancy()
    insert_seconds = time.time() - began
    print(f"投入おわり {insert_seconds/60:.1f} 分 / ノード {tree.size():,} / "
          f"メモリ {tree.memoryUsage()/1e6:.0f} MB\n")

    target = read_pcd(map_path)
    labels = classify(tree, target.points)
    occupied = labels == LABEL_OCCUPIED
    unknown = labels == LABEL_UNKNOWN
    keep = occupied if args.drop_unknown else occupied | unknown
    removed = ~keep

    floor_z = estimate_floor(target.points)
    lines = [
        f"入力: {map_path}（{len(target.points):,} 点）",
        f"投入したスキャン: {frames} 枚 / {points_in:,} 点",
        f"占有={int(occupied.sum()):,}  空={int((labels==LABEL_FREE).sum()):,}  "
        f"未知={int(unknown.sum()):,}",
        f"残した点: {int(keep.sum()):,}（{100*keep.mean():.1f}%）  "
        f"消した点: {int(removed.sum()):,}（{100*removed.mean():.1f}%）",
        "",
        *report_by_height(target.points, removed, floor_z),
    ]
    print("\n".join(lines))

    out_dir = session / "map"
    write_pcd(out_dir / args.output, [tuple(p) for p in target.points[keep]])
    stem = Path(args.output).stem
    write_pcd(out_dir / f"{stem}_removed.pcd", [tuple(p) for p in target.points[removed]])
    (session / "benchmark" / "octomap_report.txt").write_text("\n".join(lines) + "\n")
    print(f"\n出力: {out_dir/args.output} と {out_dir/(stem+'_removed.pcd')}")


if __name__ == "__main__":
    main()
