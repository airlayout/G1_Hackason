"""ROS transport regression: force SSH EOF and verify automatic ordered resume."""
from __future__ import annotations

import os
import base64
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2
except ImportError as error:
    raise unittest.SkipTest(f"ROS unavailable: {error}")

TOOLS = Path(__file__).resolve().parents[1] / "ros2_ws/src/g1_mapping_tools"
sys.path.insert(0, str(TOOLS))
from g1_mapping_tools.remote_record import SCHEMA


class AutomaticResumeTest(unittest.TestCase):
    def test_bridge_reconnects_without_exiting_ros_or_republishing_old_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            database = root / "capture.db3"
            with sqlite3.connect(database) as db:
                db.executescript(SCHEMA)
                for i in range(30):
                    msg = Imu() if i % 3 else PointCloud2()
                    msg.header.stamp.sec = 1700000000
                    msg.header.stamp.nanosec = i * 5_000_000
                    db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                               (2 if i % 3 else 1, 1700000000 * 10**9 + i * 5_000_000, serialize_message(msg)))
            executable = root / "ssh"
            # Emulate an SSH connection that drops after five records, then serves the backlog.
            executable.write_text("#!/usr/bin/env python3\n" + '''
import os,shlex,sqlite3,sys
from pathlib import Path
from g1_mapping_tools.remote_record import JOURNAL_HEADER,JOURNAL_MAGIC
args=shlex.split(sys.argv[-1]); after=int(args[args.index('--after')+1])
root=Path(os.environ['G1_TEST_DIR'])
with (root/'cursors').open('a') as log: log.write(str(after)+'\\n')
with sqlite3.connect(root/'capture.db3') as db:
    rows=db.execute('SELECT id,topic_id,timestamp,data FROM messages WHERE id>? ORDER BY id LIMIT ?', (after,5 if after==0 else 100)).fetchall()
for sequence,kind,stamp,payload in rows:
    sys.stdout.buffer.write(JOURNAL_HEADER.pack(JOURNAL_MAGIC,kind,sequence,stamp,len(payload))+payload)
sys.stdout.buffer.flush()
''')
            executable.chmod(0o755)
            previous_domain = os.environ.get("ROS_DOMAIN_ID")
            os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 30)
            rclpy.init()
            node = rclpy.create_node("g1_sensor_adapter")
            received = []
            subscriptions = [node.create_subscription(kind, topic,
                              lambda msg: received.append(msg.header.stamp.nanosec), 256)
                             for kind, topic in ((PointCloud2, "/test/points"), (Imu, "/test/imu"))]
            env = dict(os.environ, PATH=str(root) + ":" + os.environ["PATH"], G1_TEST_DIR=str(root),
                       PYTHONPATH=str(TOOLS) + ":" + os.environ.get("PYTHONPATH", ""))
            log = (root / "bridge.log").open("w")
            process = subprocess.Popen([sys.executable, "-m", "g1_mapping_tools.sensor_stream", "--host", "test",
                                        "--session-dir", str(root), "--journal-script", "unused",
                                        "--points-topic", "/test/points", "--imu-topic", "/test/imu",
                                        "--status-path", str(root / "connection.json")], env=env, stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 20
                while len(received) < 30 and time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), (root / "bridge.log").read_text())
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertEqual(sorted(received), [i * 5_000_000 for i in range(30)], (root / "bridge.log").read_text())
                self.assertIn("5", (root / "cursors").read_text().splitlines())
                # Sensor topics can dispatch independently, but each individual topic must stay ordered.
                self.assertEqual([x for x in received if x // 5_000_000 % 3 == 0],
                                 [i * 5_000_000 for i in range(0, 30, 3)])
            finally:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
                log.close()
                node.destroy_node()
                rclpy.shutdown()
                if previous_domain is None:
                    os.environ.pop("ROS_DOMAIN_ID", None)
                else:
                    os.environ["ROS_DOMAIN_ID"] = previous_domain

    def test_rgb_live_bridge_recovers_after_camera_connection_is_lost(self):
        try:
            import zmq
            from g1_mapping_tools.zmq_camera_bridge import ZmqCameraBridge
            from sensor_msgs.msg import CompressedImage
            from rclpy.qos import qos_profile_sensor_data
        except ImportError:
            self.skipTest("camera dependencies unavailable")
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        port = socket.bind_to_random_port("tcp://127.0.0.1")
        old_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 30)
        rclpy.init(args=["--ros-args", "-p", "host:=127.0.0.1", "-p", f"port:={port}"])
        bridge = ZmqCameraBridge()
        viewer = rclpy.create_node("test_rgb_viewer")
        received = []
        subscription = viewer.create_subscription(CompressedImage, "/g1_camera/color/image/compressed",
                                                  received.append, qos_profile_sensor_data)
        jpeg = b"\xff\xd8\xff\xc0\x00\x11\x08\x01\x68\x02\x80" + b"\x00" * 10

        def spin(duration, publish=False):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if publish:
                    socket.send_json({"images": {"head_camera": base64.b64encode(jpeg).decode()},
                                      "timestamps": {"head_camera": time.time()}})
                rclpy.spin_once(bridge, timeout_sec=0.02)
                rclpy.spin_once(viewer, timeout_sec=0.02)

        try:
            spin(1.5, publish=True)
            self.assertTrue(received)
            socket.close(linger=0)
            spin(4.)
            self.assertTrue(bridge._stale_reported)
            socket = context.socket(zmq.PUB)
            socket.bind(f"tcp://127.0.0.1:{port}")
            count = len(received)
            spin(2.5, publish=True)
            self.assertGreater(len(received), count)
            self.assertFalse(bridge._stale_reported)
            self.assertEqual(bytes(received[-1].data), jpeg)
        finally:
            socket.close(linger=0)
            context.term()
            bridge.destroy_node()
            viewer.destroy_node()
            rclpy.shutdown()
            if old_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = old_domain
