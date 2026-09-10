#!/usr/bin/env python3
"""Humble 時代の db3 を Jazzy の rosbag2 で再生できるようにする。

## 何が起きるのか

`ros2 bag info` は通るのに `ros2 bag play` だけが落ちる（2026-09-10 実測）。

    [ERROR] [rosbag2_storage]: Could not open '..._0.db3' with 'sqlite3'.
    Error when preparing SQL statement
    'SELECT metadata_version, metadata FROM metadata ORDER BY id;'.
    SQLite error: (1): SQL logic error

`bag info` は隣の metadata.yaml を読むので通る。`bag play` は **db3 の中の
metadata テーブル**を読むので落ちる。Humble の db3 にはそのテーブルが無い。

Jazzy が録った db3 と突き合わせると、足りないのは 3 つ。

    metadata テーブル                    … 無い（これが致命傷）
    message_definitions テーブル          … 無い
    topics.type_description_hash 列       … 無い

`schema` テーブルの schema_version はどちらも 4 なので、**版番号では判別できない**。
テーブルの有無で見ること。

## この修復のやり方

**追記しかしない。** 既存の messages / topics の中身は 1 行も書き換えない。
だから何度実行してもよいし、失敗しても元の記録は壊れない。

metadata テーブルに入れる YAML は、隣の metadata.yaml ではなく
**db3 の中身から数え直して**作る（両者が食い違っていても db3 が正）。

## 使い方

    python3 repair_bag_for_jazzy.py <bag ディレクトリ>            # 調べるだけ
    python3 repair_bag_for_jazzy.py <bag ディレクトリ> --apply    # 直す
"""

import argparse
import pathlib
import sqlite3
import sys

METADATA_VERSION = 9


def find_db3(bag_dir: pathlib.Path) -> pathlib.Path:
    """bag ディレクトリの中の db3 を 1 つ返す。"""
    candidates = sorted(bag_dir.glob("*.db3"))
    if not candidates:
        raise SystemExit(f"[中断] db3 がありません: {bag_dir}")
    if len(candidates) > 1:
        raise SystemExit(
            f"[中断] db3 が {len(candidates)} 個あります。分割された記録には未対応です:\n"
            + "\n".join(f"  {c.name}" for c in candidates)
        )
    return candidates[0]


def table_names(con: sqlite3.Connection) -> set[str]:
    return {r[0] for r in con.execute("select name from sqlite_master where type='table'")}


def column_names(con: sqlite3.Connection, table: str) -> list[str]:
    return [c[1] for c in con.execute(f"pragma table_info({table})")]


def diagnose(con: sqlite3.Connection) -> list[str]:
    """足りないものを並べる。空なら直すところは無い。"""
    missing = []
    tables = table_names(con)
    if "metadata" not in tables:
        missing.append("metadata テーブル")
    if "message_definitions" not in tables:
        missing.append("message_definitions テーブル")
    if "type_description_hash" not in column_names(con, "topics"):
        missing.append("topics.type_description_hash 列")
    return missing


def build_metadata_yaml(con: sqlite3.Connection, db3_name: str) -> str:
    """db3 の中身を数えて、Jazzy が読める版 9 の metadata を組み立てる。"""
    topics = list(con.execute("select id, name, type, serialization_format from topics order by id"))

    counts = dict(con.execute("select topic_id, count(*) from messages group by topic_id"))
    total = sum(counts.values())

    row = con.execute("select min(timestamp), max(timestamp) from messages").fetchone()
    start_ns, end_ns = (row[0] or 0), (row[1] or 0)
    duration_ns = max(0, end_ns - start_ns)

    lines = [
        f"version: {METADATA_VERSION}",
        "storage_identifier: sqlite3",
        "duration:",
        f"  nanoseconds: {duration_ns}",
        "starting_time:",
        f"  nanoseconds_since_epoch: {start_ns}",
        f"message_count: {total}",
        "topics_with_message_count:",
    ]
    for tid, name, typ, fmt in topics:
        lines += [
            "  - topic_metadata:",
            f"      name: {name}",
            f"      type: {typ}",
            f"      serialization_format: {fmt}",
            # 元の記録は offered_qos_profiles が "" だった。空リストにして
            # 購読側の既定に任せる（Jazzy は文字列を受け付けずリストを要求する）。
            "      offered_qos_profiles: []",
            '      type_description_hash: ""',
            f"    message_count: {counts.get(tid, 0)}",
        ]
    lines += [
        'compression_format: ""',
        'compression_mode: ""',
        "relative_file_paths:",
        f"  - {db3_name}",
        "files:",
        f"  - path: {db3_name}",
        "    starting_time:",
        f"      nanoseconds_since_epoch: {start_ns}",
        "    duration:",
        f"      nanoseconds: {duration_ns}",
        f"    message_count: {total}",
        "custom_data: ~",
        "ros_distro: jazzy",
        "",
    ]
    return "\n".join(lines)


def repair(con: sqlite3.Connection, db3_name: str) -> None:
    """追記だけで直す。既存の行には触らない。"""
    tables = table_names(con)

    if "type_description_hash" not in column_names(con, "topics"):
        print("  topics に type_description_hash 列を足します")
        con.execute("alter table topics add column type_description_hash TEXT NOT NULL default ''")

    if "message_definitions" not in tables:
        print("  message_definitions テーブルを作ります（中身は空でよい）")
        con.execute(
            "create table message_definitions("
            "id INTEGER PRIMARY KEY,"
            "topic_type TEXT NOT NULL,"
            "encoding TEXT NOT NULL,"
            "encoded_message_definition TEXT NOT NULL,"
            "type_description_hash TEXT NOT NULL)"
        )

    if "metadata" not in tables:
        print("  metadata テーブルを作ります")
        con.execute(
            "create table metadata("
            "id INTEGER PRIMARY KEY,"
            "metadata_version INTEGER NOT NULL,"
            "metadata TEXT NOT NULL)"
        )

    if con.execute("select count(*) from metadata").fetchone()[0] == 0:
        print("  db3 の中身を数えて metadata を書きます")
        con.execute(
            "insert into metadata (metadata_version, metadata) values (?, ?)",
            (METADATA_VERSION, build_metadata_yaml(con, db3_name)),
        )

    con.commit()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=pathlib.Path, help="bag のディレクトリ（db3 が入っているところ）")
    ap.add_argument("--apply", action="store_true", help="実際に直す（付けないと調べるだけ）")
    args = ap.parse_args()

    if not args.bag_dir.is_dir():
        raise SystemExit(f"[中断] ディレクトリがありません: {args.bag_dir}")

    db3 = find_db3(args.bag_dir)
    print(f"[INFO] 対象: {db3}")

    with sqlite3.connect(f"file:{db3}?mode=ro", uri=True) as con:
        missing = diagnose(con)
        distro = list(con.execute("select * from schema"))
        print(f"[INFO] schema テーブル: {distro}（版番号では判別できないので中身で見る）")

    if not missing:
        print("[OK]   直すところはありません。Jazzy でそのまま再生できます。")
        return 0

    print("[NG]   Jazzy の rosbag2 に足りないもの:")
    for m in missing:
        print(f"         - {m}")

    if not args.apply:
        print("\n[INFO] 直すには --apply を付けてください。追記しかしないので元の記録は壊れません。")
        return 1

    print("\n[INFO] 直します（追記のみ）")
    with sqlite3.connect(db3) as con:
        repair(con, db3.name)

    with sqlite3.connect(f"file:{db3}?mode=ro", uri=True) as con:
        left = diagnose(con)
    if left:
        print("[NG]   まだ足りません:", left)
        return 1
    print("[OK]   直りました。ros2 bag play が通るはずです。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
