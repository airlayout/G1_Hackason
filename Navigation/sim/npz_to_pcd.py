#!/usr/bin/env python3
"""scans.npz から、1804へ渡せる地図PCDと軌跡を作る。

**このデータは実機で取得したもの。** 2026-08-26 に矢田が **UiS メインフロア**で
G1 を走らせて収集し、Google Drive の `20260826_UIS_Test/scans.npz`（78.5MB）として
共有された。ローカルの `artifacts/scans.npz` と同一ファイル。

実機データであることは中身からも裏付けられる（2026-09-02 実測）:

| 指標 | 実測 | 意味 |
|---|---|---|
| スキャン間隔 | 125.0 ± 7.92 ms | ばらつき 6.3%。シミュレータなら固定刻みでほぼ 0 |
| 1スキャンの点数 | 10,862 ± 98 | 変動あり。シミュレータはレイ本数が固定 |
| 距離のユニーク率 | 0.997 | 量子化されていない |

**つまりこの地図は実在の部屋のものなので、UiS メインフロアで 1804 の定位に使える
可能性がある。**（要実機検証。Phase 5 で 1801→1802 の建図を省ける見込み）

npzの構造:
  points   (N, 3) float32  全スキャンを連結した点。**センサー座標系**
  counts   (S,)   int64    1スキャンあたりの点数
  gt_poses (S,4,4) float64 各スキャン時刻の姿勢（world <- sensor のSE3）
  times    (S,)   float64  各スキャンの時刻[s]（先頭が0起点）

`points` はセンサー座標系なので、そのまま重ねても地図にならない。
`gt_poses`（収集時の推定姿勢）で world 座標へ変換してから累積する必要がある。

出力:
  map.pcd         world座標の点群（binary PCD, x/y/z float32）
  trajectory.tum  軌跡（TUM形式）。経路追従の誤差評価に使う

注: `Mapping/real/python/g1_mapping/rebuild.py` に同等のPCD書き出しがあるが、
`Navigation/README.md` の取り決めでMapping側から import してよいのは
config / doctor / session の3モジュールだけなので、ここでは自前で持つ。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# 同一ボクセルに落ちた点は最初の1点だけ残す（rebuild.pyと同じ方針。
# 平均を取るほうが滑らかだが、蓄積中に全点を保持する必要が出てメモリが跳ねる）
DEFAULT_VOXEL_M = 0.05


def load_scans(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    with np.load(path) as archive:
        missing = {"points", "counts", "gt_poses", "times"} - set(archive.files)
        if missing:
            raise ValueError(f"npzに必要な配列がありません: {sorted(missing)}")
        return (
            archive["points"].astype(np.float64),
            archive["counts"],
            archive["gt_poses"],
            archive["times"],
        )


def to_world(points: np.ndarray, counts: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """各スキャンを対応する真値姿勢でworld座標へ変換して連結する。"""

    if len(counts) != len(poses):
        raise ValueError(f"counts({len(counts)})とgt_poses({len(poses)})の数が一致しません")
    offsets = np.concatenate([[0], np.cumsum(counts)])
    if offsets[-1] != len(points):
        raise ValueError(f"countsの合計({offsets[-1]})がpoints数({len(points)})と一致しません")

    world = np.empty_like(points)
    for index, pose in enumerate(poses):
        start, end = offsets[index], offsets[index + 1]
        world[start:end] = points[start:end] @ pose[:3, :3].T + pose[:3, 3]
    return world


def voxel_filter(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """ボクセル格子で間引く。0以下なら間引かない。"""

    if voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, first_index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first_index)]


def drop_floor(points: np.ndarray, threshold_m: float) -> np.ndarray:
    """床付近の点を落とす。0以下なら何もしない。

    Isaac Simのデータは床が点群の2割を占める。定位に寄与しないうえ
    ファイルサイズを押し上げるので、既定で薄く切る。
    """

    if threshold_m <= 0.0:
        return points
    return points[points[:, 2] > threshold_m]


def write_pcd(path: Path, points: np.ndarray) -> None:
    """binary PCD として書き出す。**書式は open3d に任せる。**

    自前でヘッダを組んでいたが（23行）、読む側も `nav/occupancy.py` の
    `load_points` が open3d なので、書く側も揃えたほうが取り違えが起きない。
    """

    import open3d as o3d

    path.parent.mkdir(parents=True, exist_ok=True)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    if not o3d.io.write_point_cloud(str(path), cloud, write_ascii=False):
        raise OSError(f"PCD を書けなかった: {path}")


def write_trajectory(path: Path, poses: np.ndarray, times: np.ndarray) -> None:
    """真値軌跡をTUM形式で書き出す（Mapping/のtrajectory.tumと同じ形式）。

    回転行列 -> 四元数は `scipy.spatial.transform.Rotation` に任せる。
    自前の Shepperd 法（31行）を置き換えたもの。`nav/protocol.py` の
    四元数変換も scipy なので、リポジトリ内で 1 つに揃う。
    """

    path.parent.mkdir(parents=True, exist_ok=True)
    quaternions = Rotation.from_matrix(poses[:, :3, :3]).as_quat()  # (qx, qy, qz, qw)
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for pose, stamp, (qx, qy, qz, qw) in zip(poses, times, quaternions):
        tx, ty, tz = pose[:3, 3]
        lines.append(
            f"{stamp:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
            f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("npz", type=Path, help="UiS メインフロアで収集した scans.npz")
    parser.add_argument("--output-dir", type=Path, default=Path("maps"))
    parser.add_argument("--name", default="uis_main_floor", help="出力ファイルのベース名")
    parser.add_argument(
        "--voxel", type=float, default=DEFAULT_VOXEL_M, help="ボクセル一辺[m]。0で間引きなし"
    )
    parser.add_argument(
        "--floor-cut",
        type=float,
        default=0.1,
        help="この高さ[m]以下の点を落とす。0で落とさない",
    )
    arguments = parser.parse_args(argv)

    points, counts, poses, times = load_scans(arguments.npz)
    print(f"[LOAD] {len(counts)} scans / {len(points):,} points / {times[-1]-times[0]:.1f}s")

    world = to_world(points, counts, poses)
    print(f"[WORLD] 範囲 {np.round(world.max(0) - world.min(0), 2).tolist()} m")

    filtered = drop_floor(world, arguments.floor_cut)
    print(f"[FLOOR] {len(world):,} -> {len(filtered):,} points (床 {arguments.floor_cut}m 以下を除去)")

    reduced = voxel_filter(filtered, arguments.voxel)
    print(f"[VOXEL] {len(filtered):,} -> {len(reduced):,} points (voxel {arguments.voxel}m)")

    map_path = arguments.output_dir / f"{arguments.name}.pcd"
    trajectory_path = arguments.output_dir / f"{arguments.name}_gt.tum"
    write_pcd(map_path, reduced)
    write_trajectory(trajectory_path, poses, times)

    minimum, maximum = reduced.min(0), reduced.max(0)
    print(f"[OK] {len(reduced):,} points -> {map_path} ({map_path.stat().st_size/1e6:.1f} MB)")
    print(f"[OK] {len(poses)} poses -> {trajectory_path}")
    print(f"[EXTENT] x={minimum[0]:.2f}..{maximum[0]:.2f} "
          f"y={minimum[1]:.2f}..{maximum[1]:.2f} z={minimum[2]:.2f}..{maximum[2]:.2f}")
    start = poses[0][:3, 3]
    print(f"[INITIAL_POSE] 1804に渡す初期位姿の目安: "
          f"x={start[0]:.3f} y={start[1]:.3f} z={start[2]:.3f}")
    # 公式の適用範囲はX/Y軸45m未満（G1 SLAM導航服務接口）
    horizontal = max(maximum[0] - minimum[0], maximum[1] - minimum[1])
    print("[RANGE_OK] 公式適用範囲45m以内です" if horizontal < 45.0
          else f"[RANGE_NG] 公式適用範囲45mを超えています: {horizontal:.1f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
