"""記録に写り込んだ「後ろについてきた人」を取り除いて PCD を作り直す。

2026-09-03 の UiS_room_v1 で、歩行中スキャンの 97.8% に人サイズの塊が
1.32m 後方（円平均 -168.8度、距離の標準偏差 0.27m）に写っていた。
ケーブルを持って追従した人物とみられる。

`mapctl rebuild` は全点を素直に積むので、この塊が経路に沿って帯状に焼き付く。
ここでは記録時のロボット姿勢を使い、**観測された瞬間の体軸座標**で判定して落とす。
最終 PCD から消すのでは駄目で、判定にはその時刻の姿勢が要る。

⚠️ これは見た目を綺麗にするだけで、**姿勢のドリフトは直らない**。
内蔵 SLAM が出す点群は既に地図座標系へ変換済み＝姿勢が座標に焼き込まれており、
11.2m のずれは点を間引いても動かない。ドリフトを直すには生 LiDAR + IMU から
姿勢を推定し直す必要がある（`--with-mapping` が記録するようになった）。

  python3 filter_follower.py runs/<session_id>
  python3 filter_follower.py runs/<session_id> --dry-run     # 落とす量だけ見る
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

# 追従者の判定範囲。2026-09-03 の実測（距離 p5=0.96 / p95=1.84m、
# 円平均 -168.8度・円標準偏差 37度、高さ帯 1.0-1.8m）に余裕を持たせた値。
FOLLOWER_MIN_RANGE = 0.6
FOLLOWER_MAX_RANGE = 2.2
FOLLOWER_MIN_HEIGHT = 0.9
FOLLOWER_MAX_HEIGHT = 1.9
# 真後ろ(180度)からこの角度以内だけを対象にする。前方・側方の構造は残す。
FOLLOWER_REAR_HALF_ANGLE = 80.0


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
                floor_z: float) -> bool:
    """観測時の体軸座標に直して、追従者の帯に入るかを判定する。"""
    height = z - floor_z
    if not (FOLLOWER_MIN_HEIGHT < height < FOLLOWER_MAX_HEIGHT):
        return False
    dx, dy = x - pose[1], y - pose[2]
    distance = math.hypot(dx, dy)
    if not (FOLLOWER_MIN_RANGE < distance < FOLLOWER_MAX_RANGE):
        return False
    bearing = math.degrees(math.atan2(dy, dx) - pose[3])
    bearing = (bearing + 180.0) % 360.0 - 180.0        # -180..180 に畳む
    return abs(abs(bearing) - 180.0) <= FOLLOWER_REAR_HALF_ANGLE


def main() -> None:
    parser = argparse.ArgumentParser(description="追従者を除去してPCDを作り直す")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--voxel", type=float, default=0.05, help="ボクセル辺長[m]（既定 0.05）")
    parser.add_argument("--output", default="map_clean.pcd", help="map/ 配下の出力名")
    parser.add_argument("--dry-run", action="store_true", help="書き出さず、落とす量だけ報告する")
    args = parser.parse_args()

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
              f"除去帯: 後方±{FOLLOWER_REAR_HALF_ANGLE:.0f}度 / "
              f"{FOLLOWER_MIN_RANGE}-{FOLLOWER_MAX_RANGE}m / "
              f"床上{FOLLOWER_MIN_HEIGHT}-{FOLLOWER_MAX_HEIGHT}m")

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
                if is_follower(x, y, z, pose, floor_z):
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
