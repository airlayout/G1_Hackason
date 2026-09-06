#!/usr/bin/env python3
"""生LiDARスキャンを、姿勢つきPCDの連番として書き出す（DynamicMap Benchmark 入力形式）。

OctoMap / ERASOR / DUFOMap などの可視性ベースの地図掃除は、入力として
「map座標系の1スキャン + そのときのセンサ姿勢」を要求する。姿勢は PCD の
VIEWPOINT 欄に入れる決まりで、`octomap_run` は sensor_origin_ をレイの始点に使う。

## 姿勢の作り方（2026-09-05 に実測して決めた）

内蔵SLAM の /unitree/slam_mapping/odom は frame_id=map / child_frame_id=base_link で、
**そのままでは生スキャンを map に置けない**。実測で分かったこと:

1. **並進は正しい。** odom の位置はそのまま LiDAR の位置でよい（レバーアーム無し）。
   RANSAC で求めた変換の並進は (-0.004, +0.016, -0.037) m で、odom 位置と一致した。
2. **回転には固定の外部パラメータが要る。** livox_frame -> base_link は
   rpy = (+178.35, -8.41, -0.72)°。**ロールがほぼ 180°、つまり Mid-360 は上下逆さま**に
   付いている。これを入れないと inlier(<0.10m) は 16.9% にしかならず、入れると 75% になる。
3. **歩行中は odom の roll/pitch が最大 30° 狂う。** 静止中（0〜348秒）は odom + 外部パラメータ
   だけで fitness 1.000 だが、歩行中（389〜636秒）は 0.30〜0.46 まで落ちる。
   yaw の補正は小さいので、腰から先の姿勢が odom に乗っていないためと考えられる。
   → **各スキャンを map_raw.pcd に ICP で合わせ直して姿勢を作る**（fitness 0.74〜1.00 に回復）。

つまり本スクリプトは「odom を初期値にした scan-to-map 位置合わせ」を行い、
その結果を VIEWPOINT に書く。合わせきれなかったスキャンは書き出さずに捨てる。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/export_benchmark_data.py \\
        runs/20260904T183457_UiS_room_v2 --limit 500        # まず試す
    ../../Navigation/.venv/bin/python quickstart/export_benchmark_data.py \\
        runs/20260904T183457_UiS_room_v2                    # 全件

出力は `<session>/benchmark/` の下に pcd/000000.pcd … と poses.txt / export_report.txt。
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import struct
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd  # noqa: E402
from g1_mapping.rebuild import _CdrReader, parse_pointcloud2, write_pcd  # noqa: E402

RAW_TOPIC = "/utlidar/cloud_livox_mid360"
ODOM_TOPIC = "/unitree/slam_mapping/odom"

# livox_frame -> base_link の回転。2026-09-05 に RANSAC + ICP で実測（冒頭の説明を参照）。
EXTRINSIC_RPY_DEG = (178.35, -8.41, -0.72)

MIN_RANGE = 0.3          # これより近い点はセンサ自身や機体
MAP_VOXEL = 0.08         # ICP の相手にする地図の間引き
ICP_SRC_VOXEL = 0.10     # ICP に渡すスキャンの間引き（合わせ込み専用。出力には使わない）
ICP_COARSE = 0.40        # 粗合わせの対応距離[m]
ICP_FINE = 0.15          # 仕上げの対応距離[m]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """度で与えたZ-Y-Xオイラー角を回転行列にする。"""
    return Rotation.from_euler("ZYX", [yaw, pitch, roll], degrees=True).as_matrix()


def read_f64(reader: _CdrReader) -> float:
    """_CdrReader に float64 が無いので補う（境界整列は本体先頭からの相対）。"""
    remainder = reader.position % 8
    if remainder:
        reader.position += 8 - remainder
    value = struct.unpack_from("<d", reader._buffer, reader.position)[0]
    reader.position += 8
    return value


def topic_id(connection: sqlite3.Connection, name: str) -> int:
    row = connection.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    if row is None:
        raise ValueError(f"{name} がbagにありません")
    return row[0]


class PoseTrack:
    """odom を連続時刻で引けるようにしたもの。

    時刻の軸は **db3 の記録時刻** を使う。/unitree/slam_mapping/points は
    header.stamp が全件 0 で、生LiDARも 7.07% が 0 のため、記録時刻が唯一
    全トピックで揃う時計であるため。
    """

    def __init__(self, connection: sqlite3.Connection) -> None:
        times, positions, quaternions = [], [], []
        for stamp, payload in connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id(connection, ODOM_TOPIC),),
        ):
            reader = _CdrReader(payload)
            reader.int32(), reader.uint32(), reader.string(), reader.string()
            positions.append([read_f64(reader) for _ in range(3)])
            quaternions.append([read_f64(reader) for _ in range(4)])
            times.append(stamp / 1e9)
        if len(times) < 2:
            raise ValueError("odom が2件未満です。姿勢を補間できません")
        self.times = np.array(times)
        self.positions = np.array(positions)
        self._slerp = Slerp(self.times, Rotation.from_quat(np.array(quaternions)))

    def at(self, when: float) -> "tuple[np.ndarray, np.ndarray]":
        """記録時刻 when における (回転行列, 位置)。範囲外は端で止める。"""
        clamped = float(np.clip(when, self.times[0], self.times[-1]))
        rotation = self._slerp([clamped]).as_matrix()[0]
        position = np.array([
            np.interp(clamped, self.times, self.positions[:, axis]) for axis in range(3)
        ])
        return rotation, position

    def yaw_at(self, when: float) -> float:
        rotation, _ = self.at(when)
        return math.atan2(rotation[1, 0], rotation[0, 0])


def iter_raw_scans(connection: sqlite3.Connection):
    """(記録時刻, 点群 Nx3) を時刻順に返す。header.stamp が 0 のものは捨てる。

    stamp が 0 なのは点数 1,000 前後の部分パケットで、全体の 7.07%。
    姿勢を引く時計としては記録時刻を使うので stamp 自体は要らないが、
    部分パケットは形が壊れているので除外する目印として使う。
    """
    for stamp, payload in connection.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id(connection, RAW_TOPIC),),
    ):
        reader = _CdrReader(payload)
        if reader.int32() == 0 and reader.uint32() == 0:
            continue
        layout = parse_pointcloud2(payload)
        count = min(layout.point_count, (len(payload) - layout.data_start) // layout.point_step)
        if count <= 0:
            continue
        block = np.frombuffer(
            payload, dtype=np.uint8, offset=layout.data_start, count=count * layout.point_step
        ).reshape(count, layout.point_step)
        points = block[:, layout.x_offset:layout.x_offset + 12].copy().view(np.float32)
        points = points.astype(np.float64)
        keep = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > MIN_RANGE)
        yield stamp / 1e9, points[keep]


def build_icp_target(map_points: np.ndarray) -> o3d.geometry.PointCloud:
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(map_points))
    target = target.voxel_down_sample(MAP_VOXEL)
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
    return target


def refine_pose(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
                initial: np.ndarray) -> "tuple[np.ndarray, float, float]":
    """scan-to-map の ICP。粗→細の2段。(変換, fitness, inlier_rmse) を返す。"""
    estimate = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    result = o3d.pipelines.registration.registration_icp(
        source, target, ICP_COARSE, initial, estimate,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
    result = o3d.pipelines.registration.registration_icp(
        source, target, ICP_FINE, result.transformation, estimate,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
    return np.asarray(result.transformation), result.fitness, result.inlier_rmse


def initial_guess(track: PoseTrack, when: float, extrinsic: np.ndarray,
                  previous: "np.ndarray | None", previous_time: float) -> np.ndarray:
    """ICP の初期値。位置は odom、向きは「前フレームの結果 + odom の yaw の変化」。

    歩行中は odom の roll/pitch が最大 30° 狂うので、そのまま初期値にすると
    ICP が引きずられる。yaw の変化は信頼できる（ICP の yaw 補正は 3° 未満）ので、
    向きだけ前フレームから引き継いで旋回分を odom で足す。
    """
    rotation, position = track.at(when)
    guess = np.eye(4)
    guess[:3, 3] = position
    if previous is None:
        guess[:3, :3] = rotation @ extrinsic
        return guess
    delta_yaw = track.yaw_at(when) - track.yaw_at(previous_time)
    turn = Rotation.from_euler("z", delta_yaw).as_matrix()
    guess[:3, :3] = turn @ previous[:3, :3]
    return guess


def export(session: Path, limit: int, start: int, stride: int, voxel: float,
           min_fitness: float, use_icp: bool, out_name: str) -> None:
    bags = sorted(session.glob("raw/rosbag2/*.db3"))
    if not bags:
        raise SystemExit(f"db3 が見つかりません: {session}/raw/rosbag2/")
    map_path = session / "map" / "map_raw.pcd"
    if not map_path.exists():
        raise SystemExit(f"地図が見つかりません: {map_path}")

    out_dir = session / out_name
    pcd_dir = out_dir / "pcd"
    pcd_dir.mkdir(parents=True, exist_ok=True)

    extrinsic = rpy_to_matrix(*EXTRINSIC_RPY_DEG)
    print(f"外部パラメータ rpy={EXTRINSIC_RPY_DEG}°（livox_frame -> base_link）")

    map_points = read_pcd(map_path).points
    target = build_icp_target(map_points) if use_icp else None
    print(f"地図 {len(map_points):,} 点 → ICP 用に {len(target.points):,} 点"
          if use_icp else "ICP による姿勢の作り直しは無効")

    connection = sqlite3.connect(bags[0])
    try:
        track = PoseTrack(connection)
        print(f"odom {len(track.times)} 件（{track.times[-1]-track.times[0]:.1f} 秒）\n")

        previous, previous_time = None, 0.0
        written = skipped_low = 0
        total_points = 0
        fitnesses: list[float] = []
        poses_lines: list[str] = []
        began = time.time()

        for index, (when, points) in enumerate(iter_raw_scans(connection)):
            if index < start:
                continue
            # 間引きは ICP に入る前に落とす。ICP が処理時間のほぼ全部なので、
            # ここで落とした枚数がそのまま時間の削減になる
            if stride > 1 and (index - start) % stride:
                continue
            if len(points) < 500:
                continue

            guess = initial_guess(track, when, extrinsic, previous, previous_time)
            if use_icp:
                source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
                transform, fitness, rmse = refine_pose(
                    source.voxel_down_sample(ICP_SRC_VOXEL), target, guess)
                if fitness < min_fitness:
                    skipped_low += 1
                    continue
                previous, previous_time = transform, when
            else:
                transform, fitness, rmse = guess, float("nan"), float("nan")

            world = points @ transform[:3, :3].T + transform[:3, 3]
            if voxel > 0:
                cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(world))
                world = np.asarray(cloud.voxel_down_sample(voxel).points)

            quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()  # x,y,z,w
            translation = transform[:3, 3]
            # PCL の VIEWPOINT は並進が先・四元数は qw が先
            viewpoint = (*translation, quaternion[3], *quaternion[:3])
            write_pcd(pcd_dir / f"{written:06d}.pcd", [tuple(p) for p in world], viewpoint)

            poses_lines.append(
                f"{when:.6f} {translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f} "
                f"{quaternion[0]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} {quaternion[3]:.6f} "
                f"{fitness:.4f} {rmse:.4f}")
            fitnesses.append(fitness)
            total_points += len(world)
            written += 1

            if written % 100 == 0:
                rate = written / max(time.time() - began, 1e-6)
                print(f"  {written} 枚 / 落とした {skipped_low} 枚 / "
                      f"{total_points:,} 点 / {rate:.1f} 枚每秒", flush=True)
            if limit and written >= limit:
                break
    finally:
        connection.close()

    (out_dir / "poses.txt").write_text(
        "# 記録時刻 tx ty tz qx qy qz qw icp_fitness icp_rmse\n" + "\n".join(poses_lines) + "\n")

    valid = [f for f in fitnesses if f == f]
    report = [
        f"セッション: {session}",
        f"外部パラメータ rpy[deg]: {EXTRINSIC_RPY_DEG}",
        f"書き出した枚数: {written}",
        f"fitness 不足で捨てた枚数: {skipped_low}（閾値 {min_fitness}）",
        f"点の総数: {total_points:,}（1枚あたり {total_points//max(written,1):,}）",
        f"フレーム内の間引き: {voxel} m",
        f"スキャンの間引き: {stride} 枚に 1 枚",
    ]
    if valid:
        report += [
            f"ICP fitness: 中央値={np.median(valid):.3f} p10={np.percentile(valid,10):.3f} "
            f"最小={min(valid):.3f}",
        ]
    (out_dir / "export_report.txt").write_text("\n".join(report) + "\n")
    print("\n" + "\n".join(report))
    print(f"\n出力: {out_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description="姿勢つきPCDの連番を書き出す")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--limit", type=int, default=0, help="書き出す枚数の上限（0で全件）")
    parser.add_argument("--start", type=int, default=0, help="先頭から読み飛ばすスキャン数")
    parser.add_argument("--stride", type=int, default=1,
                        help="何枚に1枚だけ書き出すか（既定 1＝全部）。"
                             "ICP が処理時間のほぼ全部なので、ここを N にすると時間も約 1/N になる。"
                             "ただし OctoMap の可視性除去は「後から光線が通り抜けた回数」を"
                             "証拠にするので、証拠も約 1/N に薄くなる")
    parser.add_argument("--out-name", default="benchmark",
                        help="<session>/ 直下の書き出し先の名前（既定 benchmark）。"
                             "間引きの効き方を比べるとき、既存の出力を潰さずに並べられる")
    parser.add_argument("--voxel", type=float, default=0.05,
                        help="フレーム内の間引き辺長[m]（0で間引かない。既定 0.05）")
    parser.add_argument("--min-fitness", type=float, default=0.6,
                        help="ICP がこれ未満のスキャンは捨てる（既定 0.6）")
    parser.add_argument("--no-icp", action="store_true",
                        help="ICP をせず odom + 外部パラメータのみで置く（検証用）")
    args = parser.parse_args()
    export(args.session_dir, args.limit, args.start, args.stride, args.voxel,
           args.min_fitness, not args.no_icp, args.out_name)


if __name__ == "__main__":
    main()
