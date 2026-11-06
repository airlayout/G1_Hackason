#!/usr/bin/env python3
"""2 つの地図を重ねた変換が「本物か」を、数字と絵の両方で確かめる。

## 何のためか

`align_to_map.py` は fitness と inlier RMSE を出すが、**その 2 つは偽の極大でも高く出る。**
2026-09-06 に fitness 0.949 のまま yaw が 37° ずれた解を掴み、2026-09-13 には
繰り返し構造の部屋で `reloc_probe` が真値でも外した。だから採用の前に別の証拠が要る。

ここで出す証拠は 3 つ。

1. **最近傍距離の分布** — 重なっているかどうかの直接の量。fitness と違い閾値に依存しない
2. **寸法が保たれているか**（合否 2） — 2 つの見方で測る。
   ⚠️ **部屋の外形は比べられない。** 被覆が違う（0911 は 15.8×29.4 m、旧は 24.1×32.1 m）ので、
   外形の差は幾何の差ではなく歩いた範囲の差になる。だから代わりに:
   - **縮尺を自由にして再フィットする。** 7 自由度（剛体＋縮尺）で解き直して
     **s が 1.000 を選ぶ**なら、2 つの地図は同じ寸法である。
     2026-09-16 の「自由にしても減らない」と同じ判定
   - **残差が場所によらないこと。** 縮尺誤差やドリフトがあれば、
     合わせた中心から遠いほど残差が育つ。区画ごとの中央値が平らなら育っていない
3. **ずらし検査** — 採用した変換の周りを ±0.1〜0.4 m / ±1〜4° 動かして採点する。
   最良が (0,0,0) なら局所最適に居る。2026-09-11 の
   `overlay-shift-fit-separates-misalignment-from-content` と同じ考え方

そして**絵**（上から見た重畳 PNG）を出す。合否 3 の目視はこれで行う。

⚠️ **床と天井は入れない。** 床は平らなのでどこに置いても一致する（align_to_map.py と同じ罠）。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/check_map_alignment.py \
        runs/<new>/map/map_octomap_r20.pcd \
        runs/<old>/map/map_octomap_clean.pcd \
        runs/<new>/align/T_old_from_new_band030-180.json \
        --out runs/<new>/align/check
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

# 位置合わせに使う床上の高さ帯[m]。align_to_map.py と同じ既定
STRUCTURE_BAND = (0.30, 1.80)
# 床の高さに使う分位
FLOOR_PERCENTILE = 5.0
# 最近傍が「合っている」とみなす距離[m]。地図の解像度 0.1 m の 1 セル
HIT_M = 0.10
# 残差を区画に分けるときの一辺[m]
CELL_M = 4.0
# 区画を採点に入れる最小点数。少ないと中央値が暴れる
CELL_MIN_POINTS = 200
# ずらし検査の量[m] と [deg]
SHIFTS_M = (0.1, 0.2, 0.4)
SHIFTS_DEG = (1.0, 2.0, 4.0)


def load_band(path: Path, band) -> np.ndarray:
    """床基準の高さ帯だけを返す。**床を入れると姿勢が拘束されない。**"""
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    points = points[np.isfinite(points).all(axis=1)]
    floor_z = float(np.percentile(points[:, 2], FLOOR_PERCENTILE))
    height = points[:, 2] - floor_z
    kept = points[(height >= band[0]) & (height <= band[1])]
    print("  {}: {:,} 点 -> 床 z={:.3f}m の {:.1f}〜{:.1f}m 帯で {:,} 点".format(
        path.name, len(points), floor_z, band[0], band[1], len(kept)))
    return kept


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    # ⚠️ errstate で黙らせているのは**偽の警告**。numpy 2 の matmul が Apple silicon の
    # BLAS 経由で divide-by-zero / overflow を報告するが、入力も出力も全て有限である
    # （T と点群を isfinite で確認済み）。本物の非有限は load_band で落としてある
    with np.errstate(all="ignore"):
        return points @ transform[:3, :3].T + transform[:3, 3]


def yaw_matrix(degrees: float) -> np.ndarray:
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def nn_distances(query: np.ndarray, target_tree: cKDTree) -> np.ndarray:
    """query の各点から target への最近傍距離。
    ⚠️ open3d の KDTree は 1 点ずつしか引けず、ずらし検査で 18 回まわすと分の単位になる。
    scipy の cKDTree なら一括で引ける（workers=-1）"""
    distances, _ = target_tree.query(query, k=1, workers=-1)
    return distances


def score(query: np.ndarray, target_tree) -> tuple[float, float]:
    """(HIT_M 以内の割合, 最近傍距離の中央値)。高い / 低いほど良い"""
    distances = nn_distances(query, target_tree)
    return float((distances <= HIT_M).mean()), float(np.median(distances))


def refit_with_scale(new_points: np.ndarray, old_points: np.ndarray,
                     transform: np.ndarray) -> tuple[float, float, float]:
    """縮尺を自由にして ICP をやり直し、(s, fitness, rmse) を返す。

    **s が 1.000 付近を選ぶことが「2 つの地図が同じ寸法である」直接の証拠。**
    剛体で合ったように見えても、片方に縮尺誤差があれば s はそちらへ逃げる。
    """
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(new_points))
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(old_points))
    result = o3d.pipelines.registration.registration_icp(
        source, target, 0.15, transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
    matrix = np.asarray(result.transformation)
    # 相似変換の縮尺は回転部分の列ノルム。3 列の平均を取る
    scale = float(np.mean(np.linalg.norm(matrix[:3, :3], axis=0)))
    return scale, float(result.fitness), float(result.inlier_rmse)


def residual_by_cell(points: np.ndarray, distances: np.ndarray):
    """区画ごとの残差の中央値と、中心からの距離との関係を返す。

    縮尺誤差やドリフトがあれば**遠いほど残差が育つ**。平らなら育っていない。
    """
    centroid = points[:, :2].mean(axis=0)
    keys = np.floor(points[:, :2] / CELL_M).astype(int)
    cells = []
    for key in np.unique(keys, axis=0):
        mask = (keys == key).all(axis=1)
        if mask.sum() < CELL_MIN_POINTS:
            continue
        center = (key + 0.5) * CELL_M
        cells.append({
            "center": [float(center[0]), float(center[1])],
            "points": int(mask.sum()),
            "median_m": float(np.median(distances[mask])),
            "hit_rate": float((distances[mask] <= HIT_M).mean()),
            "range_m": float(np.hypot(*(center - centroid))),
        })
    cells.sort(key=lambda c: c["range_m"])
    return cells


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("new_cloud", type=Path, help="新しい地図の点群（変換される側）")
    p.add_argument("old_cloud", type=Path, help="旧地図の点群（合わせ先）")
    p.add_argument("transform", type=Path, help="align_to_map.py --save の JSON")
    p.add_argument("--out", type=Path, required=True, help="出力の前置き（.png / .json が付く）")
    p.add_argument("--band", type=float, nargs=2, default=STRUCTURE_BAND)
    p.add_argument("--voxel", type=float, default=0.10,
                   help="採点用の間引き[m]。地図の解像度に合わせる")
    p.add_argument("--skip-shift", action="store_true", help="ずらし検査を省く（速い）")
    args = p.parse_args()

    transform = np.asarray(json.loads(args.transform.read_text())["transform"])

    print("高さ帯で絞る（床と天井を外す）")
    new_points = load_band(args.new_cloud, args.band)
    old_points = load_band(args.old_cloud, args.band)

    def thin(points: np.ndarray) -> np.ndarray:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        return np.asarray(cloud.voxel_down_sample(args.voxel).points)

    new_thin, old_thin = thin(new_points), thin(old_points)
    aligned = apply_transform(new_thin, transform)
    print("間引き後: 新 {:,} 点 / 旧 {:,} 点".format(len(new_thin), len(old_thin)))

    old_tree = cKDTree(old_thin)

    # --- 1. 最近傍距離 --------------------------------------------------------
    distances = nn_distances(aligned, old_tree)
    hit = float((distances <= HIT_M).mean())
    report = {
        "transform_json": str(args.transform),
        "hit_rate_0.10m": hit,
        "nn_median_m": float(np.median(distances)),
        "nn_p90_m": float(np.percentile(distances, 90)),
    }
    print()
    print("=== 1. 最近傍距離（新→旧、構造帯のみ） ===")
    print("  {:.1f}% が {:.2f} m 以内 / 中央値 {:.3f} m / p90 {:.3f} m".format(
        hit * 100, HIT_M, report["nn_median_m"], report["nn_p90_m"]))

    # --- 2. 寸法（縮尺を自由にした再フィット ＋ 残差の場所依存） ----------------
    print()
    print("=== 2. 寸法が保たれているか ===")
    scale, scale_fitness, scale_rmse = refit_with_scale(new_thin, old_thin, transform)
    report["free_scale"] = scale
    report["free_scale_fitness"] = scale_fitness
    report["free_scale_rmse_m"] = scale_rmse
    print("  縮尺を自由にして再フィット: s={:.5f}（fitness {:.3f} / rmse {:.3f} m）".format(
        scale, scale_fitness, scale_rmse))
    print("    → 30 m の部屋で {:+.0f} mm 相当".format((scale - 1.0) * 30000))

    cells = residual_by_cell(aligned, distances)
    report["cells"] = cells
    if cells:
        medians = np.array([c["median_m"] for c in cells])
        ranges = np.array([c["range_m"] for c in cells])
        slope = float(np.polyfit(ranges, medians, 1)[0]) if len(cells) >= 3 else 0.0
        report["residual_slope_m_per_m"] = slope
        print("  区画 {:.0f} m 角 × {} 区画の残差中央値: {:.3f} 〜 {:.3f} m（全体 {:.3f} m）".format(
            CELL_M, len(cells), medians.min(), medians.max(), float(np.median(distances))))
        print("    中心からの距離に対する傾き: {:+.4f} m/m（= 10 m 離れて {:+.0f} mm）".format(
            slope, slope * 10000))
        worst = max(cells, key=lambda c: c["median_m"])
        print("    最悪の区画: ({:+.0f}, {:+.0f}) で中央値 {:.3f} m / 一致率 {:.1f}%".format(
            worst["center"][0], worst["center"][1], worst["median_m"], worst["hit_rate"] * 100))

    # --- 3. ずらし検査 --------------------------------------------------------
    if not args.skip_shift:
        print()
        print("=== 3. ずらし検査（採用した変換が局所最適か） ===")
        print("  {:>16s}  {:>8s}  {:>8s}".format("ずらし", "一致率", "中央値"))
        print("  {:>16s}  {:7.1f}%  {:7.3f}m   <- 採用".format("なし", hit * 100, report["nn_median_m"]))
        shift_rows = []
        center = aligned.mean(axis=0)
        for delta in SHIFTS_M:
            for label, offset in (("+x", (delta, 0)), ("-x", (-delta, 0)),
                                  ("+y", (0, delta)), ("-y", (0, -delta))):
                moved = aligned + np.array([offset[0], offset[1], 0.0])
                rate, median = score(moved, old_tree)
                shift_rows.append(("{} {:+.1f}m".format(label, delta), rate, median))
        for delta in SHIFTS_DEG:
            for sign in (+1, -1):
                with np.errstate(all="ignore"):
                    moved = (aligned - center) @ yaw_matrix(sign * delta).T + center
                rate, median = score(moved, old_tree)
                shift_rows.append(("yaw {:+.1f}deg".format(sign * delta), rate, median))
        for label, rate, median in shift_rows:
            print("  {:>16s}  {:7.1f}%  {:7.3f}m  ({:+.1f}pt)".format(
                label, rate * 100, median, (rate - hit) * 100))
        best = max(shift_rows, key=lambda row: row[1])
        report["shift_best_label"] = best[0]
        report["shift_best_hit"] = best[1]
        if best[1] > hit:
            print("  [注意] ずらした方が良い: {} が {:.1f}%（採用より {:+.1f}pt）".format(
                best[0], best[1] * 100, (best[1] - hit) * 100))
        else:
            print("  [OK] どのずらしも採用を上回らない＝局所最適に居る")

    # --- 絵（合否 3 の目視） --------------------------------------------------
    args.out.parent.mkdir(parents=True, exist_ok=True)
    png = args.out.with_suffix(".png")
    both = np.vstack([aligned, old_thin])
    extent_x = (float(both[:, 0].min()), float(both[:, 0].max()))
    extent_y = (float(both[:, 1].min()), float(both[:, 1].max()))
    span_x, span_y = extent_x[1] - extent_x[0], extent_y[1] - extent_y[0]
    height = 11 * span_y / max(span_x, 1e-6) + 1.8
    fig, axes = plt.subplots(1, 3, figsize=(22, height))
    for ax, (title, layers) in zip(axes[:2], (
            ("1. before  (new map in its own frame)", [
                (old_thin, "#9aa0a6", "old  20260906 UiS_room_v3"),
                (new_thin, "#e8710a", "new  20260911 wireless_dualcam (raw)")]),
            ("2. after  (new map transformed into the old frame)", [
                (old_thin, "#9aa0a6", "old  20260906 UiS_room_v3"),
                (aligned, "#e8710a", "new  20260911 aligned")]))):
        for points, color, label in layers:
            ax.scatter(points[:, 0], points[:, 1], s=0.35, c=color, linewidths=0, label=label)
        ax.set_title(title, fontsize=13)
        ax.legend(loc="upper right", markerscale=14, fontsize=9)

    # 3 枚目: 残差。**赤が広がっていたら位置合わせ、点在なら家具が動いた側**
    ax = axes[2]
    order = np.argsort(distances)[::-1]
    scatter = ax.scatter(aligned[order, 0], aligned[order, 1], s=0.5,
                         c=np.minimum(distances[order], 0.4), cmap="turbo",
                         vmin=0.0, vmax=0.4, linewidths=0)
    ax.set_title("3. residual  (distance from each new point to the old map)", fontsize=13)
    # ⚠️ ax.clear() を呼ぶとカラーバーが消える（2026-09-08 に踏んだ）。ここでは呼ばない
    bar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.02)
    bar.set_label("nearest-neighbour distance [m]  (clipped at 0.40)", fontsize=9)

    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]  (old map frame)")
        ax.set_ylabel("y [m]")
        ax.grid(alpha=0.25, linewidth=0.4)
    fig.suptitle(
        "structure band {:.1f}-{:.1f} m above floor   |   hit<={:.2f} m: {:.1f}%   "
        "median {:.3f} m   |   free scale s={:.5f}   |   yaw {:+.2f} deg".format(
            args.band[0], args.band[1], HIT_M, hit * 100, report["nn_median_m"],
            report["free_scale"], math.degrees(math.atan2(transform[1, 0], transform[0, 0]))),
        fontsize=13)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print()
    print("絵: {}".format(png))

    args.out.with_suffix(".json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("数字: {}".format(args.out.with_suffix(".json")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
