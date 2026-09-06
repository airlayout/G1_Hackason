#!/usr/bin/env python3
"""稼働中の内蔵SLAM の座標系を、過去に作った地図の座標系へ合わせる。

## 何のためか

`1801` を投げた SLAM は**今いる場所を原点**に新しい地図を作る。一方こちらは
過去の記録から作った地図（OctoMap で掃除済み）を持っている。両者の原点は違う。

Nav2 を**過去の地図の上で**走らせたいので、その 2 つを重ねる変換を求める。
求めた変換は `map -> odom` の静的変換として流せばよく、**PC1 に地図を送り込む必要は無い**
（純正ナビ 1102 は PC1 の地図を使うが、Nav2 は Mac 側の地図を使うため）。

## どう合わせるか

初期姿勢が分からないので、いきなり ICP には入れない。2 段構えにする。

1. **大域位置合わせ**（FPFH 特徴 + RANSAC）— どこに居るのかを当てる。初期値が要らない
2. **ICP で精密化** — 粗→細で 3 段。`export_benchmark_data.py` と同じ考え方

⚠️ **床と天井は必ず外す。**（2026-09-06 に踏んだ）
静止して貯めた点群は大半が足元の床で、床は平らなので**どこに置いても一致する**。
その結果 fitness 0.949 という高い値が出たまま **yaw が 37° ずれた**解に落ちた。
姿勢を拘束するのは壁・什器などの**垂直構造**だけなので、床上の帯だけを使う。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/align_to_map.py \\
        /tmp/live_slam.pcd runs/<id>/map/map_octomap_r4_s5.pcd

合格の目安は **fitness 0.6 以上・inlier RMSE 0.15m 以下**。それを下回るなら、
別の場所で貯め直すか、貯める時間を延ばす。
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d

# 大域位置合わせ用の間引き。細かすぎると特徴が効かず、粗すぎると当たらない
GLOBAL_VOXEL = 0.30
# 床の高さに使う分位。低い側から取る
FLOOR_PERCENTILE = 5.0
# 位置合わせに使う高さ帯[m]（床基準）。床と天井を外し、垂直構造だけを残す
STRUCTURE_BAND = (0.30, 1.80)
# ICP の詰め方。粗→細
ICP_STEPS = ((1.00, 60), (0.40, 60), (0.15, 80))


def load(path: Path, voxel: float, band) -> o3d.geometry.PointCloud:
    """読み込んで、床基準の高さ帯だけ残す。**床を入れると姿勢が拘束されない。**"""
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise SystemExit("有限な点が 0 です: {}".format(path))
    floor_z = float(np.percentile(points[:, 2], FLOOR_PERCENTILE))
    height = points[:, 2] - floor_z
    kept = points[(height >= band[0]) & (height <= band[1])]
    print("  {}: {:,} 点 -> 床 z={:.3f}m の {:.1f}〜{:.1f}m 帯で {:,} 点".format(
        path.name, len(points), floor_z, band[0], band[1], len(kept)))
    if len(kept) < 500:
        raise SystemExit("帯に残った点が少なすぎる: {} 点".format(len(kept)))
    result = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(kept))
    return result.voxel_down_sample(voxel) if voxel > 0 else result


def with_features(cloud: o3d.geometry.PointCloud, voxel: float):
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    feature = o3d.pipelines.registration.compute_fpfh_feature(
        cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return cloud, feature


def matrix_to_xyz_rpy(matrix: np.ndarray):
    x, y, z = matrix[:3, 3]
    r = matrix[:3, :3]
    pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return (x, y, z), (roll, pitch, yaw)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("live", type=Path, help="稼働中SLAM から貯めた点群（capture_slam_cloud.py の出力）")
    p.add_argument("reference", type=Path, help="合わせ先の地図（OctoMap 掃除済みを推奨）")
    p.add_argument("--voxel", type=float, default=GLOBAL_VOXEL)
    p.add_argument("--min-fitness", type=float, default=0.6)
    p.add_argument("--band", type=float, nargs=2, default=STRUCTURE_BAND,
                   help="位置合わせに使う床上の高さ帯[m]。**床と天井は必ず外すこと**")
    args = p.parse_args()

    print("高さ帯で絞る（床と天井を外す）")
    live = load(args.live, args.voxel, args.band)
    reference = load(args.reference, args.voxel, args.band)
    print("間引き後: live {:,} 点 / reference {:,} 点".format(
        len(live.points), len(reference.points)))

    src, src_feature = with_features(live, args.voxel)
    dst, dst_feature = with_features(reference, args.voxel)

    print("大域位置合わせ（FPFH + RANSAC）...", flush=True)
    coarse = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, dst, src_feature, dst_feature, True, args.voxel * 1.5,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(args.voxel * 1.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999))
    print("  fitness={:.3f} rmse={:.3f}".format(coarse.fitness, coarse.inlier_rmse))

    transform = coarse.transformation
    # 精密化も帯の中だけで行う。床を入れると同じ罠にはまる
    live_full = load(args.live, 0.05, args.band)
    ref_full = load(args.reference, 0.05, args.band)
    ref_full.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))

    print("ICP で精密化 ...", flush=True)
    result = None
    for threshold, iterations in ICP_STEPS:
        result = o3d.pipelines.registration.registration_icp(
            live_full, ref_full, threshold, transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations))
        transform = result.transformation
        print("  閾値 {:.2f}m -> fitness={:.3f} rmse={:.3f}".format(
            threshold, result.fitness, result.inlier_rmse))

    (x, y, z), (roll, pitch, yaw) = matrix_to_xyz_rpy(np.asarray(transform))
    print()
    print("=== 求まった変換（reference <- live、つまり map <- odom） ===")
    print("  並進[m]   x={:+.3f} y={:+.3f} z={:+.3f}".format(x, y, z))
    print("  回転[deg] roll={:+.2f} pitch={:+.2f} yaw={:+.2f}".format(
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
    print()

    if result.fitness < args.min_fitness:
        print("[NG] fitness {:.3f} が閾値 {:.2f} に届かない。"
              "別の場所で貯め直すか、貯める時間を延ばすこと".format(result.fitness, args.min_fitness))
        return 1

    print("[OK] この変換を map -> odom として流す:")
    print("  ros2 run tf2_ros static_transform_publisher \\")
    print("      {:.4f} {:.4f} {:.4f} {:.5f} {:.5f} {:.5f} map odom".format(x, y, z, yaw, pitch, roll))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
