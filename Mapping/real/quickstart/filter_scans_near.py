#!/usr/bin/env python3
"""姿勢つきスキャンから近距離観測を捨てて、地図の材料を作り直す。

## なぜ `filter_follower.py` ではないのか

判定の考え方は同じ（**追従者はいつもセンサの近くに居る。構造物は遠くからも見える**。
根拠は `filter_follower.py` 冒頭の実測表）。違うのは**どの座標系で作業するか**である。

`filter_follower.py` は bag の `/unitree/slam_mapping/{points,odom}`、つまり
**内蔵 SLAM の点群と姿勢**を読む。ところが `map_octomap_*.pcd` と、そこから作った
`nav_map.pgm` は `benchmark_*/pcd`（**ICP で作り直した姿勢**）から出来ている。
内蔵 odom の roll/pitch は歩行中に 8〜30° 狂うことが分かっているので、
**混ぜると点群が歪み、nav_map が MOLA の地図とずれる。**

こちらは `benchmark_*/pcd` の姿勢つき PCD をそのまま入力にするので、
出力は最初から nav_map と同じ座標系にある。

## 何をするか

各スキャンについて、**その瞬間の**センサからの水平距離と床上高さで点を捨てる:

    min_range <= 水平距離 <= radius  かつ  min_height <= 床上高さ <= max_height

方位は見ない（旋回中は追従者が真横まで振れる。方位で絞ると 73% 残る）。
出力は 2 つ:

- `<session>/<out>/pcd/*.pcd` … 掃除した姿勢つきスキャン（`run_octomap.py` の入力）
- `<session>/map/<name>.pcd`   … それを積んだ点群（`run_octomap.py` の掃除対象）

**両方を作るのが要点である。** `run_octomap.py` は「掃除対象の地図」を別に取るので、
スキャンだけ掃除しても、対象側に残った追従者は*未知*と判定されて生き残る。

## 使い方

    ../../G1_Hackason/.venv/bin/python quickstart/filter_scans_near.py \\
        runs/<id> --source benchmark_s5 --out benchmark_s5_clean --radius 5.0
    # 比較用に、掃除していない積み上げも作る（--radius 0 で何も捨てない）
    ... --radius 0 --name map_scan_raw.pcd --no-scans

⚠️ `octomap` は `G1_Hackason/.venv` にしか入っていない（Navigation の venv には無い）。
"""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd, write_pcd_array  # noqa: E402

# 既定値は filter_follower.py と揃えてある。根拠はあちらの冒頭の実測表
DEFAULT_MIN_RANGE = 0.6     # これより近い点はセンサ自身や機体
DEFAULT_RADIUS = 5.0        # ここまでの近距離観測を捨てる
DEFAULT_MIN_HEIGHT = 0.9    # 床上。0.8 まで下げると机が削れる
DEFAULT_MAX_HEIGHT = 2.0    # 追従者の頭は 1.8m で切れる
DEFAULT_VOXEL = 0.05
FLOOR_SAMPLE_SCANS = 50
HEIGHT_BANDS = (-0.2, 0.25, 0.45, 0.65, 0.85, 1.05, 1.25, 1.5, 1.8, 2.2, 3.5)


def estimate_floor(points: np.ndarray) -> float:
    """z の下側の最頻ビンを床とみなす（`run_octomap.estimate_floor` と同じ考え方）。

    範囲の下半分で切ると外れ値に引きずられるので、分位で切ってから最頻ビンを探す。
    """
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    if len(lower) == 0:
        return float(z.min())
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def near_mask(points: np.ndarray, origin: np.ndarray, floor_z: float,
              limits: "tuple[float, float, float, float]") -> np.ndarray:
    """捨てる点を True にする。距離は**水平**で測る（filter_follower と同じ）。"""
    min_range, radius, min_height, max_height = limits
    height = points[:, 2] - floor_z
    in_band = (height >= min_height) & (height <= max_height)
    distance = np.hypot(points[:, 0] - origin[0], points[:, 1] - origin[1])
    return in_band & (distance >= min_range) & (distance <= radius)


def voxel_unique(points: np.ndarray, voxel: float) -> np.ndarray:
    """ボクセルあたり 1 点に間引く。各ボクセルの最初の点を代表にする。"""
    if len(points) == 0 or voxel <= 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, index = np.unique(flat, return_index=True)
    return points[np.sort(index)]


def report_by_height(points: np.ndarray, dropped: np.ndarray, floor_z: float) -> None:
    """床上の高さ帯ごとに、どれだけ捨てたかを出す。机（0.70〜0.75m）を見る欄。"""
    print("\n床の高さ z={:+.3f} m を基準にした高さ帯ごとの除去率".format(floor_z))
    print("  {:>12s} {:>12s} {:>12s} {:>8s}".format("高さ[m]", "元の点数", "捨てた点数", "除去率"))
    height = points[:, 2] - floor_z
    for low, high in zip(HEIGHT_BANDS[:-1], HEIGHT_BANDS[1:]):
        band = (height >= low) & (height < high)
        total = int(band.sum())
        if total == 0:
            continue
        gone = int((band & dropped).sum())
        print("  {:5.2f}〜{:5.2f} {:12,d} {:12,d} {:7.1f}%".format(
            low, high, total, gone, 100.0 * gone / total))


def load_floor(files: "list[Path]", override: "float | None") -> float:
    if override is not None:
        return override
    step = max(1, len(files) // FLOOR_SAMPLE_SCANS)
    sample = np.concatenate([read_pcd(f).points for f in files[::step]])
    return estimate_floor(sample)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("--source", default="benchmark_s5", help="入力の姿勢つき PCD 置き場")
    p.add_argument("--out", default="benchmark_s5_clean", help="出力先の置き場")
    p.add_argument("--name", default="map_scan_clean.pcd", help="map/ 配下の積み上げ出力名")
    p.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                   help="この水平距離までの近距離観測を捨てる[m]。0 で何も捨てない")
    p.add_argument("--min-range", type=float, default=DEFAULT_MIN_RANGE)
    p.add_argument("--height", type=float, nargs=2, metavar=("下限", "上限"),
                   default=[DEFAULT_MIN_HEIGHT, DEFAULT_MAX_HEIGHT])
    p.add_argument("--floor-z", type=float, default=None, help="床の高さ[m]。省略で自動推定")
    p.add_argument("--voxel", type=float, default=DEFAULT_VOXEL, help="積み上げの間引き[m]")
    p.add_argument("--no-scans", action="store_true", help="スキャンを書かず積み上げだけ作る")
    p.add_argument("--dry-run", action="store_true", help="書き出さず、捨てる量だけ報告する")
    args = p.parse_args()

    src = args.session_dir / args.source / "pcd"
    files = sorted(src.glob("*.pcd"))
    if not files:
        raise SystemExit("PCD がありません: {}".format(src))
    limits = (args.min_range, args.radius, args.height[0], args.height[1])

    floor_z = load_floor(files, args.floor_z)
    print("{} 枚 / 床 z={:+.3f} m".format(len(files), floor_z))
    if args.radius > 0:
        print("除去条件: 水平距離 {}〜{} m かつ 床上 {}〜{} m（方位は見ない）".format(*limits))
    else:
        print("除去なし（--radius 0）。比較用の積み上げを作る")

    out_pcd = args.session_dir / args.out / "pcd"
    if not args.dry_run and not args.no_scans:
        out_pcd.mkdir(parents=True, exist_ok=True)

    began = time.time()
    all_points, all_dropped, kept_chunks = [], [], []
    for index, path in enumerate(files, start=1):
        data = read_pcd(path)
        points = data.points
        if len(points) == 0:
            continue
        drop = (near_mask(points, data.origin, floor_z, limits) if args.radius > 0
                else np.zeros(len(points), dtype=bool))
        all_points.append(points.astype(np.float32))
        all_dropped.append(drop)
        kept = points[~drop]
        kept_chunks.append(kept.astype(np.float32))
        if not args.dry_run and not args.no_scans:
            write_pcd_array(out_pcd / path.name, kept, data.viewpoint)
        if index % 200 == 0 or index == len(files):
            print("  {}/{} 枚 / {:.1f} 秒".format(index, len(files), time.time() - began),
                  flush=True)

    points = np.concatenate(all_points)
    dropped = np.concatenate(all_dropped)
    print("\n点 {:,} のうち {:,} 点（{:.1f}%）を捨てた".format(
        len(points), int(dropped.sum()), 100.0 * dropped.mean()))
    report_by_height(points, dropped, floor_z)

    accumulated = voxel_unique(np.concatenate(kept_chunks), args.voxel)
    print("\n積み上げ: {:,} 点（voxel {} m）".format(len(accumulated), args.voxel))
    if args.dry_run:
        print("[dry-run] 書き出していない")
        return 0

    map_path = args.session_dir / "map" / args.name
    write_pcd_array(map_path, accumulated)
    print("[OK] {}".format(map_path))
    if not args.no_scans:
        poses = args.session_dir / args.source / "poses.txt"
        if poses.exists():
            shutil.copy2(poses, args.session_dir / args.out / "poses.txt")
        print("[OK] {} に {} 枚".format(out_pcd, len(files)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
