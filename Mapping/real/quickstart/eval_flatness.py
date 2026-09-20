#!/usr/bin/env python3
"""地図の局所精度を、**同じ面**で比べられる形で測る。

## なぜ作ったか

`compare_maps.py` の「壁の厚み」は、PCA の第 1 主軸に射影したヒストグラムの
**最大ピーク**の半値全幅を返す。ところが地図によって最大ピークになる壁が違う:

    map_raw            ピーク位置 +0.35 m   FWHM  7.8 cm
    map_octomap_r3     ピーク位置 +13.99 m  FWHM 17.5 cm
    map_3dgs           ピーク位置 −4.22 m   FWHM  5.8 cm
    map_3dgs_C         ピーク位置 +14.35 m  FWHM 33.1 cm

**違う壁を比べている**ので、手法間の比較には使えない（2026-09-05 に判明）。

## どう測るか

1. 元の地図（`map_raw.pcd`）から、RANSAC で大きな平面をいくつか取る。
   基準を一度だけ決めるのが要点で、これで全手法が同じ面で評価される。
2. 各地図について、その平面の近く（±0.30 m）にある点を取り、
   平面の法線方向の**符号つき距離の標準偏差**を測る。
3. 平面ごとの値の中央値を返す。点が減ると分散が下がるので、
   **残った点の割合**も併記する（少なく残せば薄く見えるという抜け道を潰すため）。

    ../../Navigation/.venv/bin/python quickstart/eval_flatness.py \\
        runs/20260904T183457_UiS_room_v2 map_octomap_r3.pcd map_3dgs.pcd
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd  # noqa: E402

SLAB = 0.20          # 平面からこれ以内の点を「その面の点」とみなす[m]
TIGHT = 0.03         # 「面に乗っている」とみなす距離[m]
MIN_INLIERS = 800    # これ未満の平面は使わない
MAX_VERTICAL_NZ = 0.35   # 法線の z 成分がこれ以下なら「垂直な壁」

# 床と天井は除く。水平面は Mid-360 が斜入射でしか見ないため元から厚く、
# しかもスラブ幅で標準偏差が飽和して（0.20/√3 ≒ 11cm）手法差が出ない。
# 局所精度を見たいので、正面から何度も当たっている垂直面だけを使う。


def find_planes(points: np.ndarray, count: int, floor: float) -> "list[np.ndarray]":
    """元の地図から大きな**垂直**平面を count 枚取る。基準はここで一度だけ決める。"""
    band = points[(points[:, 2] - floor > 0.3) & (points[:, 2] - floor < 2.2)]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(band))
    cloud = cloud.voxel_down_sample(0.05)
    planes, remaining = [], cloud
    for _ in range(count * 6):          # 水平面を捨てるぶん多めに回す
        if len(planes) >= count or len(remaining.points) < MIN_INLIERS:
            break
        model, inliers = remaining.segment_plane(
            distance_threshold=0.03, ransac_n=3, num_iterations=2000)
        if len(inliers) < MIN_INLIERS:
            break
        if abs(model[2]) <= MAX_VERTICAL_NZ:
            planes.append(np.asarray(model))
        remaining = remaining.select_by_index(inliers, invert=True)
    return planes


def spread(points: np.ndarray, plane: np.ndarray) -> "tuple[float, int]":
    """平面の近くにある点のうち、±3 cm に乗っている割合[%]と点数。

    標準偏差はスラブ幅で頭打ちになる（0.20 m なら 11 cm で飽和）ので、
    「面に乗っているか」を割合で見る。高いほど面が薄い。
    """
    with np.errstate(all="ignore"):     # この環境の BLAS が偽の FP 警告を出す
        signed = points @ plane[:3] + plane[3]
    near = np.abs(signed) < SLAB
    if near.sum() < 200:
        return float("nan"), int(near.sum())
    return float((np.abs(signed[near]) < TIGHT).mean() * 100), int(near.sum())


def main() -> None:
    parser = argparse.ArgumentParser(description="同じ平面で局所精度を比べる")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("maps", nargs="+", help="map/ 配下の名前")
    parser.add_argument("--reference", default="map_raw.pcd")
    parser.add_argument("--planes", type=int, default=8)
    args = parser.parse_args()

    session = args.session_dir
    reference = read_pcd(session / "map" / args.reference).points
    z = reference[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    counts, edges = np.histogram(z[(z >= low) & (z <= high)], bins=100)
    floor = float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)
    planes = find_planes(reference, args.planes, floor)
    normals = np.array([p[:3] for p in planes])
    kind = ["水平" if abs(n[2]) > 0.85 else "垂直" if abs(n[2]) < 0.35 else "斜め"
            for n in normals]
    print(f"基準 {args.reference} から平面を {len(planes)} 枚取った"
          f"（{kind.count('垂直')} 垂直 / {kind.count('水平')} 水平 / {kind.count('斜め')} 斜め）\n")

    names = [args.reference] + [m for m in args.maps]
    clouds = {}
    for name in names:
        path = session / "map" / name
        if not path.exists():
            print(f"見つからないので飛ばす: {name}")
            continue
        clouds[name] = read_pcd(path).points

    header = "".join(f"{Path(n).stem[:18]:>20s}" for n in clouds)
    print(f"{'平面':<14s}{header}")
    print("-" * (14 + 20 * len(clouds)))
    table = {name: [] for name in clouds}
    base = clouds[args.reference]
    for index, plane in enumerate(planes):
        row = f"{index}: {kind[index]:<10s}"
        _, base_count = spread(base, plane)
        for name, points in clouds.items():
            value, count = spread(points, plane)
            table[name].append((value, count / max(base_count, 1)))
            row += f"{value:14.1f}%乗{100*count/max(base_count,1):4.0f}%残"
        print(row)
    print("-" * (14 + 20 * len(clouds)))
    row = f"{'中央値':<14s}"
    for name in clouds:
        values = np.array([v for v, _ in table[name]])
        kept = np.array([k for _, k in table[name]])
        row += f"{np.nanmedian(values):14.1f}%乗{100*np.nanmedian(kept):4.0f}%残"
    print(row)
    print("\n読み方: 「%乗」= その面の ±3 cm に乗っている点の割合（高いほど面が薄い）。"
          "「%残」= その面に残った点の割合。\n"
          "        点を減らせば見かけ上きれいになるので、両方セットで見ること。")


if __name__ == "__main__":
    main()
