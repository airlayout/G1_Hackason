"""cut_bag_window_1110.py の試験: record_dds_to_bag.py と同じ形の小さな db3 を作り、時刻の窓で切り出す。"""
from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cut_bag_window_1110 as cut  # noqa: E402

T0 = 1_793_962_769_000_000_000  # 壁の時計 [ns]
SCHEMA = [
    "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL)",
    "CREATE TABLE topics(\n    id INTEGER PRIMARY KEY,\n    name TEXT NOT NULL,\n    type TEXT NOT NULL,\n"
    "    serialization_format TEXT NOT NULL,\n    offered_qos_profiles TEXT NOT NULL)",
    "CREATE TABLE messages(\n    id INTEGER PRIMARY KEY,\n    topic_id INTEGER NOT NULL,\n    timestamp INTEGER NOT NULL,\n"
    "    data BLOB NOT NULL)",
    "CREATE INDEX timestamp_idx ON messages(timestamp ASC)",
]


def make_session(root: Path, name: str = "20261106T185929_x_li") -> Path:
    """点群 10 Hz・IMU 100 Hz を 10 秒ぶん持つセッションを作る"""
    ses = root / name
    bag = ses / "raw" / "rosbag2"
    bag.mkdir(parents=True)
    (ses / "manifest.json").write_text(json.dumps({"session_id": name}))
    (ses / "state.json").write_text(json.dumps({"status": "completed"}))
    db = bag / f"{name}_0.db3"
    con = sqlite3.connect(db)
    for s in SCHEMA:
        con.execute(s)
    con.execute("INSERT INTO schema VALUES (3, 'humble')")
    con.execute("INSERT INTO topics VALUES (1, '/utlidar/cloud_livox_mid360', 'sensor_msgs/msg/PointCloud2', 'cdr', '')")
    con.execute("INSERT INTO topics VALUES (2, '/utlidar/imu_livox_mid360', 'sensor_msgs/msg/Imu', 'cdr', '')")
    rows = [(1, T0 + i * 100_000_000, bytes([i % 256]) * 8) for i in range(100)]
    rows += [(2, T0 + i * 10_000_000 + 1, bytes([i % 256]) * 4) for i in range(1000)]
    con.executemany("INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)", sorted(rows, key=lambda r: r[1]))
    con.commit()
    con.close()
    meta = {"rosbag2_bagfile_information": {
        "version": 4, "storage_identifier": "sqlite3", "relative_file_paths": [db.name],
        "duration": {"nanoseconds": 9_990_000_001}, "starting_time": {"nanoseconds_since_epoch": T0},
        "message_count": 1100,
        "topics_with_message_count": [
            {"topic_metadata": {"name": "/utlidar/cloud_livox_mid360", "type": "sensor_msgs/msg/PointCloud2",
                                "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": 100},
            {"topic_metadata": {"name": "/utlidar/imu_livox_mid360", "type": "sensor_msgs/msg/Imu",
                                "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": 1000}],
        "compression_format": "", "compression_mode": ""}}
    (bag / "metadata.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    return ses


def read_out(ses: Path):
    bag = ses / "raw" / "rosbag2"
    meta = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    dbs = sorted(bag.glob("*.db3"))
    con = sqlite3.connect(dbs[0])
    rows = con.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp").fetchall()
    topics = con.execute("SELECT * FROM topics ORDER BY id").fetchall()
    schema = con.execute("SELECT * FROM schema").fetchall()
    sql = sorted(r[0] for r in con.execute("SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"))
    con.close()
    return meta, dbs, rows, topics, schema, sql


def test_cuts_window_and_keeps_bytes(tmp_path: Path) -> None:
    # Arrange
    src = make_session(tmp_path / "in")
    out = tmp_path / "out" / "cut_a"
    # Act
    rep = cut.cut_session(src, out, T0 + 2_000_000_000, T0 + 5_000_000_000)
    # Assert
    meta, dbs, rows, topics, schema, sql = read_out(out)
    assert len(dbs) == 1 and dbs[0].name == "cut_a_0.db3"
    assert all(T0 + 2_000_000_000 <= r[1] <= T0 + 5_000_000_000 for r in rows)
    n_pc = sum(r[0] == 1 for r in rows)
    n_imu = sum(r[0] == 2 for r in rows)
    assert n_pc == 31 and n_imu == 300  # 端は含む（点群は 2.0 s と 5.0 s ちょうどの回がある）
    i = 25
    assert (1, T0 + i * 100_000_000, bytes([i]) * 8) in rows  # 中身の bytes は元のまま
    assert topics[0][1] == "/utlidar/cloud_livox_mid360" and schema == [(3, "humble")]
    con = sqlite3.connect(next((src / "raw" / "rosbag2").glob("*.db3")))
    src_sql = sorted(r[0] for r in con.execute("SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"))
    con.close()
    assert sql == src_sql
    assert meta["version"] == 4 and meta["relative_file_paths"] == ["cut_a_0.db3"]
    assert meta["message_count"] == n_pc + n_imu
    assert meta["starting_time"]["nanoseconds_since_epoch"] == rows[0][1]
    assert meta["duration"]["nanoseconds"] == rows[-1][1] - rows[0][1]
    counts = {t["topic_metadata"]["name"]: t["message_count"] for t in meta["topics_with_message_count"]}
    assert counts == {"/utlidar/cloud_livox_mid360": n_pc, "/utlidar/imu_livox_mid360": n_imu}
    assert json.loads((out / "manifest.json").read_text())["cut_from"]["session"] == str(src)
    assert rep["点群"] == n_pc


def test_refuses_existing_output(tmp_path: Path) -> None:
    src = make_session(tmp_path / "in")
    out = tmp_path / "out" / "cut_b"
    cut.cut_session(src, out, T0, T0 + 1_000_000_000)
    with pytest.raises(SystemExit):
        cut.cut_session(src, out, T0, T0 + 1_000_000_000)


def test_refuses_window_without_scans(tmp_path: Path) -> None:
    src = make_session(tmp_path / "in")
    with pytest.raises(SystemExit):
        cut.cut_session(src, tmp_path / "out" / "cut_c", T0 + 20_000_000_000, T0 + 30_000_000_000)


def test_mono_window_to_wall() -> None:
    assert cut.window_ns("1500:1560", 1793960152.156) == (1793961652156000000, 1793961712156000000)
    with pytest.raises(SystemExit):
        cut.window_ns("1560:1500", 0.0)
