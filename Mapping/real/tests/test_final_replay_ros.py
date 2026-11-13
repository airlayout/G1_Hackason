"""Exercise the complete raw-recording -> FAST-LIO2 -> OctoMap finalization path."""
from __future__ import annotations

import json
from dataclasses import replace
import os
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

try:
    from ament_index_python.packages import get_package_prefix
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    get_package_prefix("fast_lio")
    get_package_prefix("g1_mapping_octomap")
except (ImportError, LookupError) as error:
    raise unittest.SkipTest(f"FAST-LIO ROS workspace unavailable: {error}")

TOOLS = Path(__file__).resolve().parents[1] / "ros2_ws/src/g1_mapping_tools"
sys.path.insert(0, str(TOOLS))
from g1_mapping_tools.remote_record import SCHEMA, write_metadata
from g1_mapping.pcd import inspect_pcd, validate_session
from g1_mapping.native import NativeRuntime
from g1_mapping.remote import RemoteRecording
from g1_mapping.session import SessionManager
from test_mapping import make_settings


class FinalReplayTest(unittest.TestCase):
    def test_complete_g1_raw_bag_produces_rebuildable_final_map(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            settings = replace(make_settings(root), runtime="native", transport="ssh",
                               camera_enabled=False, camera_required=False, min_map_points=100,
                               ros_domain_id=140 + os.getpid() % 25)
            session = SessionManager(settings).create(name="full", backend="raw", diagnostic={})
            original = session.directory / "raw" / "g1"
            (original / "rosbag2").mkdir(parents=True)
            topics = [("/utlidar/cloud_livox_mid360", "sensor_msgs/msg/PointCloud2"),
                      ("/utlidar/imu_livox_mid360", "sensor_msgs/msg/Imu")]
            data = bytearray()
            points = []
            for i in range(100):
                for j in range(20):
                    a, z = -2.5 + i * 5. / 99, -1. + j * 2.5 / 19
                    points.extend([(a, -2.5, z), (a, 2.5, z), (-2.5, a, z), (2.5, a, z)])
            for index, (x, y, z) in enumerate(points):
                data.extend(struct.pack("<ffffI", x, y, z, 50., int(index / (len(points) - 1) * 99_000_000)))
            base = 1_700_000_000 * 10**9
            records = []
            for index in range(40):
                stamp = base + index * 100_000_000
                msg = PointCloud2()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                msg.header.frame_id = "livox_frame"
                msg.height, msg.width = 1, len(points)
                msg.point_step, msg.row_step = 20, 20 * len(points)
                msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(("x", "y", "z", "intensity"))]
                msg.fields.append(PointField(name="t", offset=16, datatype=PointField.UINT32, count=1))
                msg.data = bytes(data)
                records.append((1, stamp + 100_000_000, serialize_message(msg)))
            for index in range(850):
                stamp = base - 100_000_000 + index * 5_000_000
                msg = Imu()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                msg.header.frame_id = "livox_frame"
                msg.linear_acceleration.z = 9.81
                msg.orientation.w = 1.
                records.append((2, stamp, serialize_message(msg)))
            with sqlite3.connect(original / "rosbag2/capture.db3") as db:
                db.executescript(SCHEMA)
                for i, (topic, kind) in enumerate(topics, 1):
                    db.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (i, topic, kind, "cdr", ""))
                db.executemany("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                               sorted(records, key=lambda item: item[1]))
                write_metadata(original, db, topics)
            (original / "inventory.json").write_text(json.dumps({"state": {"status": "completed", "dropped": 0}}))
            session.map_path.write_bytes(b"preview map")
            runtime = NativeRuntime(settings)
            # Already inside the sourced ROS test environment; only SSH control is replaced.
            with patch.object(runtime, "command", side_effect=lambda *args: list(args)), \
                 patch.object(RemoteRecording, "stop"), \
                 patch.object(RemoteRecording, "collect", return_value=original / "rosbag2"):
                try:
                    runtime.finalize_remote(session)
                except RuntimeError:
                    logs = list(session.directory.glob("derived/final_*/replay.log"))
                    self.fail("\n".join(path.read_text()[-10000:] for path in logs))
            manifest = json.loads(session.manifest_path.read_text())
            self.assertTrue(manifest["final_replay"]["complete"])
            output = session.directory / manifest["final_replay"]["directory"]
            self.assertEqual(session.map_path.read_bytes(), (output / "map_raw.pcd").read_bytes())
            self.assertEqual(session.map_path.with_name("map_preview.pcd").read_bytes(), b"preview map")
            valid, report = validate_session(session.directory, min_map_points=100, allow_partial=False)
            self.assertTrue(valid, report)
            info = inspect_pcd(output / "map_raw.pcd")
            self.assertGreater(info["points"], 100)
            self.assertEqual(info["finite_ratio"], 1.)
            report = json.loads((output / "replay.json").read_text())
            self.assertGreaterEqual(report["registered_scans"], 30)
            rebuilt = root / "rebuilt.pcd"
            subprocess.run(["ros2", "run", "g1_mapping_octomap", "rebuild_occupancy", "--bag-dir", str(output / "rosbag2"),
                            "--output", str(rebuilt)], check=True, capture_output=True, timeout=30)
            self.assertEqual(rebuilt.read_bytes(), (output / "map_raw.pcd").read_bytes())
            info = subprocess.run(["ros2", "bag", "info", str(original / "rosbag2")],
                                  text=True, capture_output=True, timeout=15)
            self.assertEqual(info.returncode, 0, info.stderr)
