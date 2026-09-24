#!/usr/bin/env python3
"""rosbag2(sqlite3) の Odometry トピックを TUM 形式の軌跡に落とす。**ROS 不要**。

地図を作るには点群だけでは足りず、**軌跡**が要る（何も無い場所は点群には写らないため。
[findings/map_rebuild_20260911.md](../findings/map_rebuild_20260911.md) §1）。
軌跡は `pointcloud_to_occupancy_grid.py --trajectory` に渡す。

⚠️ **ROS 環境が無い操作PC でも動くように、`.db3` を直接読んで CDR を自前で解く。**
rosbag2_py を使うには ROS を source した端末が要り、地図づくりのたびに PC2 へ
渡りに行くことになるため（2026-09-24）。

使い方:

    python3 bag_odom_to_tum.py <bagディレクトリ or .db3> --topic /dog_odom \\
        --out ../trajectory/room_b_sorasta_20260923.tum

⚠️ **`/unitree/slam_mapping/odom` が 0 件の記録がある**（内蔵SLAM を上げ忘れた記録）。
その場合は `/dog_odom`（脚オドメトリ）しか無い。歩いた距離が伸びるほどドリフトが
乗るので、**地図が歪んだら記録を取り直すこと**。`--topic` の件数は最後に出す。
"""

from __future__ import annotations

import argparse
import sqlite3
import struct
import sys
from pathlib import Path


class CdrReader:
    """CDR(rosbag2 の既定シリアライズ)を読む最小限の実装。

    ⚠️ **各要素は自分の大きさに整列している**（float64 なら 8 バイト境界）。
    整列を忘れると値が静かにずれる（例外は出ない）ので、必ず align してから読む。
    """

    def __init__(self, payload: bytes) -> None:
        # 先頭 4 バイトは encapsulation header。2 バイト目が 1 ならリトルエンディアン
        self._little = payload[1] in (1, 3)
        self._buf = payload[4:]
        self._pos = 0

    def _align(self, size: int) -> None:
        rem = self._pos % size
        if rem:
            self._pos += size - rem

    def _unpack(self, fmt: str, size: int):
        self._align(size)
        value = struct.unpack_from(("<" if self._little else ">") + fmt, self._buf, self._pos)[0]
        self._pos += size
        return value

    def int32(self) -> int:
        return self._unpack("i", 4)

    def uint32(self) -> int:
        return self._unpack("I", 4)

    def float64(self) -> float:
        return self._unpack("d", 8)

    def string(self) -> str:
        length = self.uint32()          # 終端の '\0' を含む長さ
        raw = self._buf[self._pos:self._pos + length - 1]
        self._pos += length
        return raw.decode("utf-8", "replace")

    def skip(self, count: int, size: int) -> None:
        self._align(size)
        self._pos += count * size


def parse_odometry(payload: bytes) -> tuple[float, tuple[float, ...], str, str]:
    """nav_msgs/msg/Odometry から (時刻, (x,y,z,qx,qy,qz,qw), frame_id, child_frame_id)。"""
    r = CdrReader(payload)
    sec = r.int32()
    nanosec = r.uint32()
    frame_id = r.string()
    child_frame_id = r.string()
    pose = tuple(r.float64() for _ in range(7))   # position(3) + orientation(4)
    return sec + nanosec * 1e-9, pose, frame_id, child_frame_id


def find_db3(target: Path) -> Path:
    if target.is_dir():
        files = sorted(target.glob("*.db3"))
        if not files:
            raise SystemExit(f"[tum] .db3 が見つからない: {target}")
        if len(files) > 1:
            raise SystemExit(f"[tum] .db3 が {len(files)} 個ある。1つを指定すること: {files}")
        return files[0]
    return target


def main() -> int:
    ap = argparse.ArgumentParser(description="rosbag2 の Odometry を TUM 軌跡にする(ROS 不要)")
    ap.add_argument("bag", type=Path, help="bag ディレクトリ、または .db3")
    ap.add_argument("--topic", default="/dog_odom", help="読むトピック(既定: /dog_odom)")
    ap.add_argument("--out", type=Path, help="出力する .tum(--list のときは不要)")
    ap.add_argument("--stride", type=int, default=1,
                    help="N 件に1件だけ書く(既定: 1)。脚オドメトリは 1kHz 級で出るので間引く")
    ap.add_argument("--list", action="store_true", help="トピックと件数を出して終わる")
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)

    if args.list:
        rows = con.execute(
            "SELECT t.name, t.type, COUNT(m.id) FROM topics t "
            "LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id ORDER BY t.name").fetchall()
        for name, type_, count in rows:
            print(f"{count:>9}  {name}  ({type_})")
        return 0

    if args.out is None:
        raise SystemExit("[tum] --out が要る(--list なら不要)")

    row = con.execute("SELECT id, type FROM topics WHERE name = ?", (args.topic,)).fetchone()
    if row is None:
        raise SystemExit(f"[tum] トピックが無い: {args.topic}（--list で一覧できる）")
    topic_id, topic_type = row
    if not topic_type.endswith("Odometry"):
        raise SystemExit(f"[tum] Odometry ではない: {topic_type}")

    cur = con.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    stamps: list[float] = []
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    length = 0.0
    frames: set[str] = set()
    written = 0
    with args.out.open("w", encoding="utf-8") as fp:
        for index, (_, blob) in enumerate(cur):
            stamp, pose, frame_id, child = parse_odometry(blob)
            frames.add(f"{frame_id}→{child}")
            if xs:
                length += ((pose[0] - xs[-1]) ** 2 + (pose[1] - ys[-1]) ** 2) ** 0.5
            stamps.append(stamp)
            xs.append(pose[0])
            ys.append(pose[1])
            zs.append(pose[2])
            if index % args.stride:
                continue
            fp.write("%.6f %.4f %.4f %.4f %.6f %.6f %.6f %.6f\n" % ((stamp,) + pose))
            written += 1

    if not stamps:
        raise SystemExit(f"[tum] {args.topic} は 0 件だった。記録時に出ていなかった可能性が高い")

    span = stamps[-1] - stamps[0]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    print(f"[tum] {args.topic}: {len(stamps)} 件 / {span:.1f} 秒 "
          f"= {len(stamps) / span:.1f} Hz（最大の欠測 {max(gaps):.3f} 秒）")
    print(f"[tum] frame: {', '.join(sorted(frames))}")
    print(f"[tum] 範囲: x {min(xs):.2f}..{max(xs):.2f} / y {min(ys):.2f}..{max(ys):.2f} "
          f"/ z {min(zs):.2f}..{max(zs):.2f}")
    print(f"[tum] 歩いた距離: {length:.1f} m")
    print(f"[tum] 書いた: {written} 行 → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
