#!/usr/bin/env python3
"""記録に写り込んだ「後ろについてきた人」を取り除いて PCD を作り直す。

## 判定の考え方（2026-09-04 に実測で決め直した）

**追従者はいつもセンサの近くに居る。構造物は遠くからも見える。**

これが効く理由は、地図が多数の姿勢からの観測を積んで作られていることにある。
壁や机は、ロボットが近くを通ったときにも遠くから見たときにも観測される。
近距離の観測だけを捨てても、遠距離の観測が残るので**地図からは消えない**。
一方、追従者はロボットと一緒に動くので**どの時点でも近くにしか居ない**。
近距離の観測をすべて捨てると、追従者だけが消える。

    ロボットが通った跡 ── 追従者は常にこの近傍 ── 近距離だけ捨てれば消える
    壁・机・柱        ── 近くからも遠くからも見える ── 遠距離の観測が残る

実測（`20260904T183457_UiS_room_v2`、内蔵SLAMの点群 362,500 ボクセル）:

    半径      追従者の帯の残存   遠方構造の残存   机の残存   天井の残存
    3.0m         10.0%            100%          100%      100%
    5.0m          2.4%             98.0%        100%      100%
    6.0m          1.4%             96.4%        100%      100%
    8.0m          0.02%            94.1%        100%      100%

## 却下した2つの案

- **後方セクタ限定**（旧実装。真後ろ±80°・0.6-2.2m・床上0.9-1.9m）
  → 追従者が **73% 残った**。人は真後ろだけでなく斜め後ろにも大きく振れる
  （実測で真後ろからの角度の p95 = 62°、しかも旋回中はさらに広がる）。
- **観測の持続性で切る**（実在物は長時間observed されるはず）
  → 内蔵SLAMは点を**初回のみ配信する増分方式**なので、観測回数が持続性を
     表していない。ノイズ帯も遠方構造も 42〜57% が観測幅 5 秒未満で、
     まったく分離できなかった。

## 高さ帯について

床上 **0.9〜2.0m** を対象にする。実測で:

- 下限 0.9m … 0.8m まで下げると**机が 78% まで削れた**。机の天板がここに入る
- 上限 2.0m … 追従者の頭は 1.8m で切れる（1.8-2.4m 帯は 589 点、2.4m 以上は 0 点）

⚠️ **較正時の値をそのまま使ってはいけない。** 2026-09-04 の静止較正は機体を
ウィンチで吊った状態で行ったため、LiDAR の床からの高さが歩行時と違う。
センサ基準の z をそのまま床上高さに読み替えると 0.2m ほどずれる。

⚠️ これは見た目を綺麗にするだけで、**姿勢のドリフトは直らない**。
内蔵 SLAM が出す点群は既に地図座標系へ変換済み＝姿勢が座標に焼き込まれており、
点を間引いてもずれは動かない。

⚠️ **狭い場所では使えない。** 半径 6m 以内の人の高さの観測をすべて捨てるので、
廊下や小部屋のように「遠くから見る機会が無い」環境では構造まで失う。
この記録は 23.5 × 32.0m の部屋なので成立している。

  python3 filter_follower.py runs/<session_id>
  python3 filter_follower.py runs/<session_id> --dry-run       # 落とす量だけ見る
  python3 filter_follower.py runs/<session_id> --radius 8.0    # もっと落とす
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.rebuild import _CdrReader, iter_points, write_pcd  # noqa: E402

POINTS_TOPIC = "/unitree/slam_mapping/points"
ODOM_TOPIC = "/unitree/slam_mapping/odom"

# 既定値。根拠は冒頭の表を参照。
DEFAULT_MIN_RANGE = 0.6     # これより近い点はセンサ自身や機体。元から少ない
DEFAULT_RADIUS = 6.0        # ここまでの近距離観測を捨てる
DEFAULT_MIN_HEIGHT = 0.9    # 床上。0.8 まで下げると机が削れる
DEFAULT_MAX_HEIGHT = 2.0    # 追従者の頭は 1.8m で切れる


def read_odom(connection: sqlite3.Connection) -> "list[tuple[float, float, float, float]]":
    """(timestamp, x, y, yaw) を時刻順に返す。"""
    row = connection.execute(
        "SELECT id FROM topics WHERE name=?", (ODOM_TOPIC,)).fetchone()
    if row is None:
        raise ValueError(f"{ODOM_TOPIC} がbagにありません。姿勢が無いと判定できません")
    poses: list[tuple[float, float, float, float]] = []
    for timestamp, payload in connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)
    ):
        reader = _CdrReader(payload)
        reader.int32(); reader.uint32(); reader.string(); reader.string()
        x, y = _f64(reader), _f64(reader)
        _f64(reader)                                   # z は使わない
        qx, qy, qz, qw = (_f64(reader) for _ in range(4))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        poses.append((timestamp / 1e9, x, y, yaw))
    return poses


def _f64(reader: _CdrReader) -> float:
    """_CdrReader に float64 が無いので補う（境界整列は本体先頭からの相対）。"""
    remainder = reader.position % 8
    if remainder:
        reader.position += 8 - remainder
    value = struct.unpack_from("<d", reader._buffer, reader.position)[0]
    reader.position += 8
    return value


def find_floor(points_sample: "list[tuple[float, float, float]]") -> float:
    """Z ヒストグラムの下半分の最頻ビンを床とみなす。"""
    zs = sorted(p[2] for p in points_sample)
    if not zs:
        raise ValueError("床の推定に使える点がありません")
    low, high = zs[0], zs[-1]
    middle = (low + high) / 2.0
    bins = 80
    width = max((high - low) / bins, 1e-6)
    counts: dict[int, int] = {}
    for z in zs:
        if z < middle:
            counts[int((z - low) / width)] = counts.get(int((z - low) / width), 0) + 1
    if not counts:
        return low
    best = max(counts, key=lambda k: counts[k])
    return low + (best + 0.5) * width


def is_follower(x: float, y: float, z: float, pose: "tuple[float, float, float, float]",
                floor_z: float, limits: "tuple[float, float, float, float]") -> bool:
    """観測した瞬間の距離と床上高さで判定する。

    方位は見ない。旋回中は追従者が真横まで振れるうえ、方位で絞ると
    追従者が 73% 残ることを実測で確認したため（冒頭の説明を参照）。
    """
    min_range, radius, min_height, max_height = limits
    height = z - floor_z
    if not (min_height <= height <= max_height):
        return False
    distance = math.hypot(x - pose[1], y - pose[2])
    return min_range <= distance <= radius


def main() -> None:
    parser = argparse.ArgumentParser(description="追従者を除去してPCDを作り直す")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--voxel", type=float, default=0.05, help="ボクセル辺長[m]（既定 0.05）")
    parser.add_argument("--output", default="map_clean.pcd", help="map/ 配下の出力名")
    parser.add_argument("--dry-run", action="store_true", help="書き出さず、落とす量だけ報告する")
    parser.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                        help=f"この距離までの近距離観測を捨てる[m]（既定 {DEFAULT_RADIUS}）。"
                             "大きいほど追従者は消えるが、遠くから見る機会の無い構造も失う")
    parser.add_argument("--min-range", type=float, default=DEFAULT_MIN_RANGE,
                        help=f"これより近い点は対象外[m]（既定 {DEFAULT_MIN_RANGE}）")
    parser.add_argument("--height", type=float, nargs=2, metavar=("下限", "上限"),
                        default=[DEFAULT_MIN_HEIGHT, DEFAULT_MAX_HEIGHT],
                        help=f"床上の高さ帯[m]（既定 {DEFAULT_MIN_HEIGHT} {DEFAULT_MAX_HEIGHT}）")
    args = parser.parse_args()
    limits = (args.min_range, args.radius, args.height[0], args.height[1])

    bags = sorted((args.session_dir / "raw" / "rosbag2").glob("*.db3"))
    if not bags:
        raise SystemExit(f"db3 がありません: {args.session_dir}")

    connection = sqlite3.connect(f"file:{bags[0]}?mode=ro", uri=True)
    try:
        poses = read_odom(connection)
        print(f"[filter] odom {len(poses)} 件を読みました")
        topic_row = connection.execute(
            "SELECT id FROM topics WHERE name=?", (POINTS_TOPIC,)).fetchone()
        if topic_row is None:
            raise SystemExit(f"{POINTS_TOPIC} がbagにありません")

        # 床の推定に最初の方のスキャンを使う
        sample: list[tuple[float, float, float]] = []
        for (payload,) in connection.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 300",
            (topic_row[0],),
        ):
            sample.extend(iter_points(payload))
        floor_z = find_floor(sample)
        print(f"[filter] 床 Z={floor_z:+.2f}m と推定。"
              f"除去条件: 観測距離 {limits[0]}-{limits[1]}m かつ "
              f"床上 {limits[2]}-{limits[3]}m（方位は見ない）")

        voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        total = dropped = messages = 0
        pose_index = 0
        for timestamp, payload in connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_row[0],),
        ):
            messages += 1
            scan_time = timestamp / 1e9
            # odom は時刻順なので、前方へ進めるだけで対応が取れる
            while pose_index + 1 < len(poses) and poses[pose_index + 1][0] <= scan_time:
                pose_index += 1
            pose = poses[pose_index]
            try:
                points = iter_points(payload)
            except (ValueError, struct.error, IndexError):
                continue
            for x, y, z in points:
                total += 1
                if is_follower(x, y, z, pose, floor_z, limits):
                    dropped += 1
                    continue
                key = (int(math.floor(x / args.voxel)),
                       int(math.floor(y / args.voxel)),
                       int(math.floor(z / args.voxel)))
                if key not in voxels:
                    voxels[key] = (x, y, z)
            if messages % 1000 == 0:
                print(f"  {messages} msg / {total} 点 / 除去 {dropped} / 残 {len(voxels)}")
    finally:
        connection.close()

    ratio = 100.0 * dropped / max(total, 1)
    print(f"\n[filter] 全 {total} 点のうち {dropped} 点 ({ratio:.2f}%) を追従者として除去")
    print(f"[filter] ボクセル {args.voxel}m 後の残り {len(voxels)} 点")
    if args.dry_run:
        print("[filter] --dry-run のため書き出しません")
        return
    output = args.session_dir / "map" / args.output
    write_pcd(output, list(voxels.values()))
    print(f"[OUTPUT] {output}")


if __name__ == "__main__":
    main()
