"""11-10 手順 L1: 頭のライダーの記録（record_dds_to_bag.py の db3・rosbag2 の版 4）を、時刻の窓で切り出した別のセッションにする。

OMEN で大きな記録（11-06 夕は 2 本で 5.6 GB）から要る区間だけを切り、Mac の colima の FAST-LIO の再生（replay_realtime.sh）へ運ぶため。
jazzy の `ros2 bag convert` は新しい版（metadata の版 9）で書き、colima の humble で読めない恐れがあるので、
同じ SQLite の表の作り・同じ bytes のまま、行を受け取りの時刻（db3 の timestamp・PC2 の壁の時計）で絞って写す。

出力は `<出力>/raw/rosbag2/<出力の名前>_0.db3` と `metadata.yaml`（件数・始まり・長さを書き直す）、`manifest.json`（元の場所と窓を足す）・`state.json`。
replay_realtime.sh は `G1_BAG_SUB=raw/rosbag2` で読む。

使い方（OMEN・doorman6 の Python）:
  python cut_bag_window_1110.py <元のセッション> <出力のセッション> --mono 1495:1560 --offset 1793960152.156
  python cut_bag_window_1110.py <元のセッション> <出力のセッション> --wall-ns <始め>:<終わり>
  --mono は画像・走らせ役の monotonic 秒、--offset は 壁 − monotonic（R0 の正解の meta の clock_offset_s）
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path
from typing import Dict, Tuple

import yaml

BAG_SUB = Path("raw") / "rosbag2"
LIDAR_TOPIC = "/utlidar/cloud_livox_mid360"
COPY_CHUNK = 2000  # 一度に写す行の数（点群 1 回 約 440 KB）


def window_ns(mono: str, offset_s: float) -> Tuple[int, int]:
    """'始め:終わり'（monotonic 秒）→ 壁の時計の ns"""
    a, b = (float(x) for x in mono.split(":"))
    if not b > a:
        raise SystemExit(f"[cut] 窓の終わりが始めより前: {mono}")
    return round((a + offset_s) * 1e9), round((b + offset_s) * 1e9)


def find_db(session: Path) -> Tuple[Path, Dict]:
    bag = session / BAG_SUB
    dbs = sorted(bag.glob("*.db3"))
    if len(dbs) != 1:
        raise SystemExit(f"[cut] {bag} の db3 が {len(dbs)} 個（1 個だけを扱う）")
    meta = yaml.safe_load((bag / "metadata.yaml").read_text())
    if meta["rosbag2_bagfile_information"]["storage_identifier"] != "sqlite3":
        raise SystemExit(f"[cut] {bag} は sqlite3 でない")
    return dbs[0], meta


def copy_rows(src: sqlite3.Connection, dst: sqlite3.Connection, t0: int, t1: int) -> Dict[int, int]:
    """窓の行を時刻の順に写し、話題ごとの件数を返す"""
    counts: Dict[int, int] = {}
    cur = src.execute("SELECT topic_id, timestamp, data FROM messages WHERE timestamp BETWEEN ? AND ? ORDER BY timestamp",
                      (t0, t1))
    while True:
        rows = cur.fetchmany(COPY_CHUNK)
        if not rows:
            break
        dst.executemany("INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)", rows)
        for tid, _, _ in rows:
            counts[tid] = counts.get(tid, 0) + 1
    return counts


def new_metadata(meta: Dict, db_name: str, counts_by_name: Dict[str, int], first: int, last: int) -> Dict:
    info = meta["rosbag2_bagfile_information"]
    topics = [{**t, "message_count": counts_by_name.get(t["topic_metadata"]["name"], 0)}
              for t in info["topics_with_message_count"]]
    return {"rosbag2_bagfile_information": {
        **info, "relative_file_paths": [db_name], "duration": {"nanoseconds": last - first},
        "starting_time": {"nanoseconds_since_epoch": first}, "message_count": sum(counts_by_name.values()),
        "topics_with_message_count": topics}}


def cut_session(session: Path, out: Path, t0: int, t1: int) -> Dict:
    """session の [t0, t1]（壁の時計 ns・端を含む）を out へ切り出す"""
    if out.exists():
        raise SystemExit(f"[cut] 出力が既にある（上書きしない）: {out}")
    db, meta = find_db(session)
    src = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        first, last, n_scan = src.execute(
            "SELECT min(timestamp), max(timestamp), sum(topic_id = (SELECT id FROM topics WHERE name = ?)) "
            "FROM messages WHERE timestamp BETWEEN ? AND ?", (LIDAR_TOPIC, t0, t1)).fetchone()
        if not n_scan:
            rng = src.execute("SELECT min(timestamp), max(timestamp) FROM messages").fetchone()
            raise SystemExit(f"[cut] 窓 {t0}〜{t1} に点群が無い（記録は {rng[0]}〜{rng[1]}）")
        bag = out / BAG_SUB
        bag.mkdir(parents=True)
        db_out = bag / f"{out.name}_0.db3"
        dst = sqlite3.connect(db_out)
        try:
            # 表の作りは元のまま（索引は行を入れた後に作る）
            ddl = src.execute("SELECT type, sql FROM sqlite_master WHERE sql IS NOT NULL").fetchall()
            for typ, sql in ddl:
                if typ == "table":
                    dst.execute(sql)
            for (name,) in src.execute("SELECT name FROM sqlite_master WHERE type = 'table' AND name != 'messages'"):
                rows = src.execute(f"SELECT * FROM {name}").fetchall()
                if rows:
                    dst.executemany(f"INSERT INTO {name} VALUES ({','.join('?' * len(rows[0]))})", rows)
            counts = copy_rows(src, dst, t0, t1)
            for typ, sql in ddl:
                if typ == "index":
                    dst.execute(sql)
            dst.commit()
            names = dict(src.execute("SELECT id, name FROM topics").fetchall())
        finally:
            dst.close()
    finally:
        src.close()
    by_name = {names[k]: v for k, v in counts.items()}
    (bag / "metadata.yaml").write_text(yaml.safe_dump(new_metadata(meta, db_out.name, by_name, first, last), sort_keys=False,
                                                      allow_unicode=True))
    for f in ("manifest.json", "state.json"):
        p = session / f
        if p.exists():
            d = json.loads(p.read_text())
            if f == "manifest.json":
                d = {**d, "session_id": out.name, "cut_from": {"session": str(session), "wall_ns": [t0, t1]}}
            (out / f).write_text(json.dumps(d, ensure_ascii=False, indent=2))
    rep = {"点群": by_name.get(LIDAR_TOPIC, 0), "全部": sum(by_name.values()), "長さ [s]": round((last - first) / 1e9, 2),
           "大きさ [MB]": round(db_out.stat().st_size / 1e6, 1)}
    print(f"[cut] {out.name}: {rep}", flush=True)
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("out", type=Path)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--mono", help="始め:終わり（monotonic 秒）。--offset と使う")
    g.add_argument("--wall-ns", help="始め:終わり（壁の時計 ns）")
    ap.add_argument("--offset", type=float, help="壁 − monotonic [s]")
    a = ap.parse_args(argv)
    if a.mono is not None:
        if a.offset is None:
            raise SystemExit("[cut] --mono には --offset が要る")
        t0, t1 = window_ns(a.mono, a.offset)
    else:
        t0, t1 = (int(x) for x in a.wall_ns.split(":"))
    cut_session(a.session, a.out, t0, t1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
