#!/usr/bin/env python
"""[エントリポイント] 収録した LiDAR スキャンから自己位置推定と 3D 地図構築を行う。

入力は `Mapping/sim/record_scans.py`（実機なら `Mapping/real/`）が吐いた `scans.npz`。
sim / real どちらのデータでも同じこのスクリプトで処理できる。

SLAM が使うのは各フレームの点群だけで、真値の姿勢は一切使わない。
npz に真値が入っている場合は、最後に推定軌跡との誤差(ATE)を表示する。

使い方:
    python Mapping/run_slam.py --scans Mapping/data/scans.npz
    python Mapping/run_slam.py --scans Mapping/data/scans.npz --map-voxel 0.03
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Mapping.common.pointcloud import height_colors, save_ply, transform_points  # noqa: E402
from Mapping.common.slam import LidarSlam, SlamConfig  # noqa: E402


def load_scans(path: Path) -> tuple[list[np.ndarray], np.ndarray | None]:
    """`scans.npz` を読み、フレームごとの点群リストと（あれば）真値姿勢を返す。"""
    data = np.load(path)
    points = data["points"]
    counts = data["counts"]
    bounds = np.concatenate([[0], np.cumsum(counts)])
    scans = [points[bounds[i] : bounds[i + 1]].astype(np.float64) for i in range(len(counts))]
    gt = data["gt_poses"] if "gt_poses" in data else None
    return scans, gt


def align_trajectories(estimated: np.ndarray, truth: np.ndarray) -> np.ndarray:
    """推定軌跡を真値の座標系に合わせる（yaw + 平行移動のみ）。

    SLAM の地図座標系は「1 フレーム目の LiDAR 姿勢」が原点なので、真値の
    ワールド座標系とは最初からずれている。この定数ぶんのオフセットを引かないと
    ATE が「SLAM の誤差」ではなく「座標系の違い」を測ってしまう。
    重力方向は LiDAR の姿勢から決まっていて傾かないので、水平回転だけを合わせる。
    """
    est_c = estimated - estimated.mean(axis=0)
    tru_c = truth - truth.mean(axis=0)
    h = est_c[:, :2].T @ tru_c[:, :2]
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot2 = vt.T @ np.diag([1.0, d]) @ u.T
    rot = np.eye(3)
    rot[:2, :2] = rot2
    return (estimated - estimated.mean(axis=0)) @ rot.T + truth.mean(axis=0)


def main() -> None:
    parser = argparse.ArgumentParser(description="LiDAR スキャン列から 3D 地図を作る")
    parser.add_argument("--scans", default="Mapping/data/scans.npz", help="入力の npz")
    parser.add_argument("--out", default="", help="出力 PLY。既定は入力と同じ場所の map.ply")
    parser.add_argument("--map-voxel", type=float, default=0.05, help="出力地図の解像度[m]")
    parser.add_argument("--scan-voxel", type=float, default=0.10, help="ICP に渡すスキャンの間引き[m]")
    args = parser.parse_args()

    scans_path = Path(args.scans)
    if not scans_path.exists():
        print(f"[error] スキャンが無い: {scans_path}", flush=True)
        print("[error] 先に python Mapping/sim/record_scans.py を実行すること", flush=True)
        raise SystemExit(1)

    scans, gt_poses = load_scans(scans_path)
    print(f"[load] {len(scans)} フレーム / {sum(len(s) for s in scans)} 点  ({scans_path})", flush=True)

    slam = LidarSlam(SlamConfig(scan_voxel_size=args.scan_voxel, map_voxel_size=args.map_voxel))
    t0 = time.perf_counter()
    for i, scan in enumerate(scans):
        slam.process(scan)
        if (i + 1) % 20 == 0 or i + 1 == len(scans):
            s = slam.stats[-1]
            print(
                f"[slam] {i + 1:4d}/{len(scans)}  pos=({s.pose[0, 3]:6.2f},{s.pose[1, 3]:6.2f},"
                f"{s.pose[2, 3]:5.2f})  fitness={s.fitness:.2f} rmse={s.inlier_rmse:.3f} "
                f"map={len(slam.map)}",
                flush=True,
            )
    elapsed = time.perf_counter() - t0

    map_points = slam.map.points()
    out_path = Path(args.out) if args.out else scans_path.parent / "map.ply"
    save_ply(out_path, map_points, height_colors(map_points))

    traj = slam.trajectory()
    traj_path = out_path.with_name(out_path.stem + "_trajectory.ply")
    save_ply(traj_path, traj)

    degraded = sum(1 for s in slam.stats if s.degraded)
    print("", flush=True)
    print(f"[result] 地図: {len(map_points)} 点 ({args.map_voxel}m ボクセル) -> {out_path}", flush=True)
    print(f"[result] 推定軌跡: {len(traj)} 点 -> {traj_path}", flush=True)
    print(f"[result] 処理時間: {elapsed:.1f}s ({elapsed / max(len(scans), 1) * 1000:.0f} ms/frame)", flush=True)
    print(f"[result] ICP が信用できず等速度モデルに落ちたフレーム: {degraded}/{len(scans)}", flush=True)
    print(
        f"[result] 地図の範囲: x [{map_points[:, 0].min():.2f}, {map_points[:, 0].max():.2f}] "
        f"y [{map_points[:, 1].min():.2f}, {map_points[:, 1].max():.2f}] "
        f"z [{map_points[:, 2].min():.2f}, {map_points[:, 2].max():.2f}]",
        flush=True,
    )

    if gt_poses is not None:
        gt_xyz = gt_poses[:, :3, 3]
        aligned = align_trajectories(traj, gt_xyz)
        err = np.linalg.norm(aligned - gt_xyz, axis=1)
        print("", flush=True)
        print("[eval] 推定軌跡 vs 真値（真値は SLAM には渡していない）", flush=True)
        print(f"[eval] ATE RMSE : {np.sqrt((err ** 2).mean()):.3f} m", flush=True)
        print(f"[eval] ATE 最大 : {err.max():.3f} m", flush=True)
        print(f"[eval] 移動距離 : {np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1).sum():.2f} m", flush=True)


if __name__ == "__main__":
    main()
