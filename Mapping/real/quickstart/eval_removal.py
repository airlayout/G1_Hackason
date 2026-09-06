#!/usr/bin/env python3
"""動的物体の除去がうまくいったかを、正解ラベル無しで測る。

正解ラベルは無い。しかし**物理から言える確かなことが2つある**ので、それを指標にする。

1. **ロボットが実際に歩いた体積は、静止物ではありえない。**
   軌跡から 0.5m 以内・床上 0.25〜1.8m に残っている点は、ロボット自身か追従者。
   静止物なら歩けていない。→ **ここは消えているほどよい**。
2. **軌跡から遠い場所は、追従者が居られない。**
   追従者はケーブルを持って 1〜2m 後ろを歩いていたので、軌跡から 4m 以上離れた点は
   壁・机・什器である。→ **ここは残っているほどよい**。

床と天井も別に見る。Mid-360 は垂直FOV −7° で床を grazing 角でしか見ないため、
長い光線が床の voxel を舐めて「空」に投票しやすい。ここが削れていたら
maxRange か地面フィルタが要るという合図である。

    ../../Navigation/.venv/bin/python quickstart/eval_removal.py runs/<session> \\
        map_clean.pcd map_octomap.pcd map_octomap_r10.pcd
"""
from __future__ import annotations

import argparse
import glob
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd  # noqa: E402
from g1_mapping.rebuild import _CdrReader  # noqa: E402

ODOM_TOPIC = "/unitree/slam_mapping/odom"
MATCH_TOLERANCE = 1e-4      # 掃除後の地図は元の点をそのまま残すので厳密一致でよい

SWEPT_RADIUS = 0.5          # 軌跡からこれ以内は「ロボットが通った＝静止物ではない」
FAR_RADIUS = 4.0            # 軌跡からこれ以上離れたら追従者は居られない
HUMAN_LOW, HUMAN_HIGH = 0.25, 1.8
FLOOR_LOW, FLOOR_HIGH = -0.20, 0.25
CEILING_LOW = 2.2


def read_trajectory(session: Path) -> np.ndarray:
    bag = sorted(glob.glob(str(session / "raw/rosbag2/*.db3")))[0]
    connection = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    try:
        topic = connection.execute(
            "SELECT id FROM topics WHERE name=?", (ODOM_TOPIC,)).fetchone()[0]
        poses = []
        for (payload,) in connection.execute(
                "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic,)):
            reader = _CdrReader(payload)
            reader.int32(), reader.uint32(), reader.string(), reader.string()
            values = []
            for _ in range(3):
                remainder = reader.position % 8
                if remainder:
                    reader.position += 8 - remainder
                values.append(struct.unpack_from("<d", reader._buffer, reader.position)[0])
                reader.position += 8
            poses.append(values)
    finally:
        connection.close()
    return np.array(poses)


def estimate_floor(points: np.ndarray) -> float:
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def survived(reference: np.ndarray, cleaned: np.ndarray) -> np.ndarray:
    """referenceの各点が cleaned に残っているかの真偽値。"""
    if len(cleaned) == 0:
        return np.zeros(len(reference), dtype=bool)
    distance, _ = cKDTree(cleaned).query(reference, workers=-1)
    return distance <= MATCH_TOLERANCE


def regions(points: np.ndarray, trajectory: np.ndarray,
            floor_z: float) -> "dict[str, np.ndarray]":
    height = points[:, 2] - floor_z
    to_path, _ = cKDTree(trajectory[:, :2]).query(points[:, :2], workers=-1)
    human = (height >= HUMAN_LOW) & (height < HUMAN_HIGH)
    return {
        "歩いた体積（動的のはず）": human & (to_path < SWEPT_RADIUS),
        "軌跡 0.5〜1.5m（大半が追従者）": human & (to_path >= SWEPT_RADIUS) & (to_path < 1.5),
        "遠方の構造（静的のはず）": human & (to_path >= FAR_RADIUS),
        "床": (height >= FLOOR_LOW) & (height < FLOOR_HIGH),
        "天井・上部": height >= CEILING_LOW,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="動的物体除去の効き目を測る")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("maps", nargs="+", help="map/ 配下の掃除後の地図の名前")
    parser.add_argument("--reference", default="map_raw.pcd", help="元の地図（既定 map_raw.pcd）")
    args = parser.parse_args()

    session = args.session_dir
    reference = read_pcd(session / "map" / args.reference).points
    trajectory = read_trajectory(session)
    floor_z = estimate_floor(reference)
    masks = regions(reference, trajectory, floor_z)

    print(f"元の地図: {args.reference}（{len(reference):,} 点）  床 z={floor_z:.3f} m  "
          f"軌跡 {len(trajectory)} poses\n")
    print(f"{'区分':<30s} {'点数':>9s}" + "".join(f"{Path(m).stem:>22s}" for m in args.maps))
    print(f"{'':<30s} {'':>9s}" + "".join(f"{'除去率':>22s}" for _ in args.maps))
    print("-" * (40 + 22 * len(args.maps)))

    survivals = {}
    for name in args.maps:
        path = session / "map" / name
        if not path.exists():
            raise SystemExit(f"見つかりません: {path}")
        survivals[name] = survived(reference, read_pcd(path).points)

    for label, mask in masks.items():
        total = int(mask.sum())
        row = f"{label:<30s} {total:>9,d}"
        for name in args.maps:
            gone = int((mask & ~survivals[name]).sum())
            row += f"{100*gone/max(total,1):21.1f}%"
        print(row)

    print("-" * (40 + 22 * len(args.maps)))
    row = f"{'全体':<30s} {len(reference):>9,d}"
    for name in args.maps:
        row += f"{100*(1-survivals[name].mean()):21.1f}%"
    print(row)

    print("\n読み方: 「歩いた体積」と「軌跡0.5〜1.5m」は高いほどよい。"
          "「遠方の構造」「床」「天井」は低いほどよい。")


if __name__ == "__main__":
    main()
