#!/usr/bin/env python3
"""記録の LiDAR を間引いた bag を作る。**CPU 飽和による実効レート低下を再現するため。**

## なぜ要るか

2026-09-12 の実機の崩壊（`click_20260912T125919`）では、MOLA のレートが
**7.75 -> 6.77 -> 5.51 -> 3.34 Hz -> 停止**と階段状に落ちた。
一方 `mola-lidar-odometry-cli` の再生は**実時間で走らない**（自分のペースで
全スキャンを処理する）ので、この部分は**構造的に再現しない**。

CPU に飢えた MOLA がやっていることは「スキャンを取りこぼす」ことなので、
**入力側でスキャンを間引けば、同じ実効レートを決定的に再現できる。**
IMU（200 Hz）と脚 odom は落とさない。落ちるのは LiDAR だけだったため。

    Navigation/.venv/bin/python quickstart/decimate_bag.py \
        runs/click_20260912T125919/bag out_dir --keep 1/3

⚠️ `metadata.yaml` の件数も書き換える。rosbag2 の C++ 読み出しは
   **metadata の件数を信じる**ので、そのままだと足りない/多すぎるで黙って崩れる。
"""
from __future__ import annotations

import argparse
import glob
import re
import sqlite3
from pathlib import Path

LIDAR_DEFAULT = "/utlidar/cloud_livox_mid360"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", type=Path, help="元の bag ディレクトリ")
    ap.add_argument("dst", type=Path, help="出力の bag ディレクトリ")
    ap.add_argument("--keep", default="1/3", help='"1/3" = 3 枚に 1 枚だけ残す')
    ap.add_argument("--topic", default=LIDAR_DEFAULT, help="間引く対象")
    args = ap.parse_args()

    num, den = (int(v) for v in args.keep.split("/"))
    src_db = Path(sorted(glob.glob(str(args.src / "*.db3")))[0])
    args.dst.mkdir(parents=True, exist_ok=True)
    dst_db = args.dst / src_db.name

    if dst_db.exists():
        dst_db.unlink()
    con = sqlite3.connect(dst_db)
    src = sqlite3.connect("file:{}?mode=ro".format(src_db), uri=True)

    # スキーマをそのまま持ってくる（rosbag2 の版に依存しないため）
    for (sql,) in src.execute("SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"):
        con.execute(sql)
    # ⚠️ topics だけでなく schema / metadata の**行も**持ってくること。
    # rosbag2 の sqlite3 storage は `schema` 表で版を判定するので、空だと
    # 「No storage could be initialized」で落ちる（表の定義があっても中身が要る）。
    for table in ("schema", "metadata", "topics"):
        try:
            rows = list(src.execute("SELECT * FROM {}".format(table)))
        except sqlite3.OperationalError:
            continue
        for row in rows:
            con.execute("INSERT INTO {} VALUES ({})".format(
                table, ",".join("?" * len(row))), row)

    tid = {n: i for i, n in src.execute("SELECT id,name FROM topics")}
    target = tid.get(args.topic)
    counts: dict[int, int] = {}
    seen = 0
    for row in src.execute("SELECT * FROM messages ORDER BY timestamp"):
        mid, topic_id = row[0], row[1]
        if topic_id == target:
            keep = (seen % den) < num
            seen += 1
            if not keep:
                continue
        con.execute("INSERT INTO messages VALUES ({})".format(",".join("?" * len(row))), row)
        counts[topic_id] = counts.get(topic_id, 0) + 1
    con.commit()
    con.close()

    # metadata.yaml の件数を実際に書いた数へ直す
    meta_src = args.src / "metadata.yaml"
    text = meta_src.read_text()
    name = {i: n for n, i in tid.items()}
    total = sum(counts.values())
    text = re.sub(r"message_count: \d+", "message_count: {}".format(total), text, count=1)

    def fix(m: str) -> str:
        # 各トピックの message_count を差し替える
        out, cur = [], None
        for line in m.splitlines():
            hit = re.search(r"name:\s*(\S+)\s*$", line)
            if hit:
                cur = hit.group(1)
            # ⚠️ 末尾の `files:` 節は `path:` で始まり `name:` が無いので、
            # cur が最後のトピックのまま残る。そこの message_count は**全体の件数**
            # なので、トピックの件数で上書きすると reader が途中で止まる
            if re.search(r"^\s*- path:", line):
                cur = None
            if "message_count:" in line and cur is not None:
                for i, n in name.items():
                    if n == cur:
                        line = re.sub(r"message_count:\s*\d+",
                                      "message_count: {}".format(counts.get(i, 0)), line)
                        break
            out.append(line)
        return "\n".join(out) + "\n"

    (args.dst / "metadata.yaml").write_text(fix(text))
    for i, n in sorted(name.items()):
        print("  {:<44} {:>7}".format(n, counts.get(i, 0)))
    print("  -> {}  ({:.0f} MB)".format(args.dst, dst_db.stat().st_size / 1e6))


if __name__ == "__main__":
    main()
