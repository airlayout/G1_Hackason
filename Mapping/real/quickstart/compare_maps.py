#!/usr/bin/env python3
"""内蔵 SLAM の地図と FAST-LIO2 の地図を並べて比べる。

見るのは3つ。

1. **追従者が消えたか** — 軌跡の近く・人の背丈のところに点が残っていないか。
   ロボットが通った場所は本来なにも無いので、そこに点があれば移動物体。
2. **壁の厚み** — 局所精度。厚くなっていたら姿勢推定が悪化している。
3. **部屋の大きさ** — 幾何が保たれているか。

⚠️ **開始と終了のずれはここでは測れない。** FAST-LIO2 の軌跡は
`/g1_mapping/odom` に出るが PCD には入らないので、別に記録する必要がある。

    ./quickstart/compare_maps.py runs/<session_id>
"""
from __future__ import annotations

import argparse
import glob
import sqlite3
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

from g1_mapping.rebuild import _CdrReader  # noqa: E402

ODOM_TOPIC = "/unitree/slam_mapping/odom"


def read_trajectory(session: Path) -> np.ndarray:
    """内蔵 SLAM の odom を軌跡として読む。追従者の判定基準に使う。"""
    bag = glob.glob(str(session / "raw/rosbag2/*.db3"))[0]
    connection = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    row = connection.execute(
        "SELECT id FROM topics WHERE name=?", (ODOM_TOPIC,)).fetchone()
    if row is None:
        raise SystemExit(f"{ODOM_TOPIC} が bag にありません")
    points = []
    for (blob,) in connection.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
        payload = bytes(blob)
        reader = _CdrReader(payload)
        reader.int32(); reader.uint32(); reader.string(); reader.string()
        position = reader.position
        if position % 8:
            position += 8 - position % 8
        points.append(struct.unpack_from("<3d", payload[4:], position))
    return np.array(points)


def detect_floor(points: np.ndarray) -> float:
    """Z ヒストグラムの2大ピークを床と天井とみなし、低い方を返す。

    外れ値（窓の外を拾った点）が混じると最頻ビンがずれるので、
    先に各軸 p1-p99 で絞ってからヒストグラムを取る。
    """
    low, high = np.percentile(points, [1, 99], axis=0)
    core = ((points >= low) & (points <= high)).all(axis=1)
    counts, edges = np.histogram(points[core, 2], bins=200)
    centres = (edges[:-1] + edges[1:]) / 2
    peaks: list[int] = []
    for index in np.argsort(counts)[::-1]:
        if all(abs(centres[index] - centres[other]) > 1.0 for other in peaks):
            peaks.append(index)
        if len(peaks) == 2:
            break
    return float(min(centres[p] for p in peaks))


def wall_thickness(points: np.ndarray, floor: float) -> float:
    """主要な壁面の厚み（半値全幅）を返す。局所精度の指標。

    壁は「床から 0.3〜2.0m にある垂直面」。主軸を PCA で出し、その方向の
    ヒストグラムの最大ピークの半値全幅を測る。
    """
    band = points[(points[:, 2] - floor > 0.3) & (points[:, 2] - floor < 2.0)]
    if len(band) < 1000:
        return float("nan")
    centred = band[:, :2] - band[:, :2].mean(axis=0)
    # 主軸に直交する方向へ射影すると、壁が細いピークとして立つ
    _, _, vectors = np.linalg.svd(centred[
        np.random.default_rng(0).choice(len(centred), min(200000, len(centred)),
                                        replace=False)], full_matrices=False)
    projected = centred @ vectors[0]
    counts, edges = np.histogram(projected, bins=2000)
    peak = int(np.argmax(counts))
    half = counts[peak] / 2
    left = peak
    while left > 0 and counts[left] > half:
        left -= 1
    right = peak
    while right < len(counts) - 1 and counts[right] > half:
        right += 1
    return float((edges[right] - edges[left]))


def describe(label: str, path: Path, trajectory: np.ndarray) -> dict:
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    low, high = np.percentile(points, [1, 99], axis=0)
    core = ((points >= low) & (points <= high)).all(axis=1)
    floor = detect_floor(points)
    above = points[:, 2] - floor
    distance, _ = cKDTree(trajectory[:, :2]).query(points[:, :2])
    human = (above >= 0.2) & (above < 1.8)
    head = (above >= 1.8) & (above < 2.6)
    near_human = float((distance[human] < 1.0).mean() * 100) if human.any() else float("nan")
    near_head = float((distance[head] < 1.0).mean() * 100) if head.any() else float("nan")
    result = {
        "label": label,
        "points": len(points),
        "core": [float(np.ptp(points[core, i])) for i in range(3)],
        "floor": floor,
        "near_human": near_human,
        "near_head": near_head,
        "follower": float(((above >= 0.2) & (above < 1.8) & (distance < 1.6)).mean() * 100),
        "wall": wall_thickness(points, floor),
    }
    print(f"\n【{label}】 {path.name}")
    print(f"  点数            : {result['points']:>10,}")
    print(f"  主要部分        : {result['core'][0]:.1f} × {result['core'][1]:.1f} × {result['core'][2]:.1f} m")
    print(f"  床              : z = {floor:+.2f} m")
    print(f"  壁の厚み(半値全幅): {result['wall'] * 100:.1f} cm")
    print(f"  人の高さ帯で軌跡1m以内 : {near_human:5.1f} %")
    print(f"  頭上で軌跡1m以内       : {near_head:5.1f} %  ← 本来ほぼ 0")
    print(f"  追従者の痕跡（軌跡1.6m以内×人の高さ）: {result['follower']:.2f} % の点")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path)
    parser.add_argument("--maps", nargs="+", metavar="ラベル=ファイル名",
                        help="比較する地図を map/ 配下の名前で指定する。"
                             "例: --maps 内蔵SLAM=map_raw.pcd OctoMap=map_octomap.pcd")
    args = parser.parse_args()
    session = args.session if args.session.is_dir() else Path("runs") / args.session

    if args.maps:
        targets = tuple(tuple(entry.split("=", 1)) for entry in args.maps)
        if any(len(entry) != 2 for entry in targets):
            raise SystemExit("--maps は ラベル=ファイル名 の形で渡すこと")
    else:
        targets = (("内蔵 SLAM", "map_raw.pcd"),
                   ("FAST-LIO2 + CropBox", "map_fastlio.pcd"),
                   ("FAST-LIO2 マスク無し", "map_fastlio_nocrop.pcd"))

    trajectory = read_trajectory(session)
    print("=" * 70)
    print("地図の比較")
    print("=" * 70)
    print(f"追従者の判定に使う軌跡: 内蔵 SLAM の odom {len(trajectory)} poses")

    results = []
    for label, name in targets:
        path = session / "map" / name
        if path.exists():
            results.append(describe(label, path, trajectory))
        else:
            print(f"\n【{label}】 {name} … まだ無い")

    if len(results) >= 2:
        print("\n" + "=" * 70)
        print(f"{'指標':<28}" + "".join(f"{r['label']:>20}" for r in results))
        print("-" * 70)
        for key, name, fmt in (("points", "点数", "{:,}"),
                               ("wall", "壁の厚み [cm]", "{:.1f}"),
                               ("near_human", "人の高さ×軌跡1m [%]", "{:.1f}"),
                               ("near_head", "頭上×軌跡1m [%]", "{:.1f}"),
                               ("follower", "追従者の痕跡 [%]", "{:.2f}")):
            row = f"{name:<28}"
            for r in results:
                value = r[key] * 100 if key == "wall" else r[key]
                row += f"{fmt.format(value):>20}"
            print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
