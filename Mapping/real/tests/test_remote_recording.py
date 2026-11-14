from __future__ import annotations

import ast
import base64
from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parents[1] / "ros2_ws/src/g1_mapping_tools"
sys.path.insert(0, str(TOOLS))
from g1_mapping_tools import remote_record as rr
from g1_mapping_tools.camera_wire import camera_records
from g1_mapping_tools.sensor_stream import read_journal_frame
from g1_mapping.cli import _parser, _recover, _stop
from g1_mapping.native import NativeRuntime
from g1_mapping.remote import RemoteRecording
from g1_mapping.session import SessionManager
from g1_mapping.mock import write_mock_artifacts
from g1_mapping.pcd import validate_session
from test_mapping import make_settings


def wait_for(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.025)
    raise AssertionError("timed out")


class DetachedRecordingTest(unittest.TestCase):
    def test_missing_camera_dependency_is_installed_only_into_private_directory(self):
        with patch.dict(sys.modules, {"zmq": None}), patch.object(rr.subprocess, "run") as install:
            rr.prepare(True)
        command = install.call_args.args[0]
        self.assertIn("pyzmq==25.1.2", command)
        self.assertIn("--only-binary=:all:", command)
        self.assertEqual(command[command.index("--target") + 1], str(Path(rr.__file__).parent / "deps"))

    def test_network_disconnect_keeps_capture_alive_and_resume_has_no_holes(self):
        self._run_capture(False)

    def test_rgb_keeps_recording_alongside_lidar_during_disconnect(self):
        try:
            import zmq
        except ImportError:
            self.skipTest("pyzmq unavailable")
        self._run_capture(True)

    def _run_capture(self, camera):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sdk = root / "sdk"
            sdk.mkdir()
            for name in ("unitree_sdk2py", "unitree_sdk2py/core", "unitree_sdk2py/idl",
                         "unitree_sdk2py/idl/sensor_msgs", "unitree_sdk2py/idl/sensor_msgs/msg"):
                (sdk / name).mkdir(exist_ok=True)
                (sdk / name / "__init__.py").touch()
            (sdk / "idl_imu.py").touch()
            (sdk / "unitree_sdk2py/idl/sensor_msgs/msg/dds_.py").write_text("Imu_ = PointCloud2_ = object\n")
            (sdk / "unitree_sdk2py/core/channel.py").write_text('''
import threading
class Message:
    def __init__(self, sequence): self.sequence = sequence
    def serialize(self, use_version_2=False): return b'\\x00\\x01\\x00\\x00' + str(self.sequence).encode()
def ChannelFactoryInitialize(*args): pass
class ChannelSubscriber:
    def __init__(self, *args): self.stop = threading.Event()
    def Init(self, callback, depth):
        def loop():
            sequence = 0
            while not self.stop.wait(0.005):
                callback(Message(sequence)); sequence += 1
        self.thread = threading.Thread(target=loop)
        self.thread.start()
    def Close(self): self.stop.set(); self.thread.join()
''')
            directory = root / "runs" / "session"
            config = dict(points_topic="/points", imu_topic="/imu", sdk_path=str(sdk),
                          iface="eth0", min_free_gib=0, camera_enabled=camera,
                          camera_required=camera, camera_name="head_camera")
            camera_stop = threading.Event()
            if camera:
                import zmq
                camera_context = zmq.Context()
                camera_socket = camera_context.socket(zmq.PUB)
                config["camera_port"] = camera_socket.bind_to_random_port("tcp://127.0.0.1")
                def publish():
                    jpeg = b"\xff\xd8\xff\xc0\x00\x11\x08\x01\x68\x02\x80" + b"\x00" * 10
                    while not camera_stop.wait(0.02):
                        camera_socket.send_json({"images": {"head_camera": base64.b64encode(jpeg).decode()},
                                                 "timestamps": {"head_camera": time.time()}})
                camera_thread = threading.Thread(target=publish)
                camera_thread.start()
            script = Path(rr.__file__)
            command = [sys.executable, str(script)]
            # The launcher's process exits; only detached capture remains.
            streams = []
            try:
                launched = subprocess.run(command + ["start", "--directory", str(directory), "--config", json.dumps(config)],
                                          text=True, capture_output=True, timeout=25)
                self.assertEqual(launched.returncode, 0, launched.stderr)
                def connect(after):
                    process = subprocess.Popen(command + ["stream", "--directory", str(directory), "--after", str(after)],
                                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                    streams.append(process)
                    return process
                first = connect(0)
                frames = [read_journal_frame(first.stdout) for _ in range(8)]
                cursor = frames[-1][1]
                first.terminate()
                first.wait(timeout=5)
                before = sum(rr.read_state(directory)["counts"].values())
                wait_for(lambda: sum(rr.read_state(directory)["counts"].values()) > before + 20)
                self.assertTrue(rr.alive(rr.read_state(directory)))
                second = connect(cursor)
                frames += [read_journal_frame(second.stdout) for _ in range(20)]
                second.terminate()
                second.wait(timeout=5)
                stopped = subprocess.run(command + ["stop", "--directory", str(directory)], text=True,
                                         capture_output=True, timeout=30)
                self.assertEqual(stopped.returncode, 0, stopped.stderr)
                self.assertEqual(json.loads(stopped.stdout)["status"], "completed")
                with sqlite3.connect(directory / "rosbag2/capture.db3") as db:
                    rows = db.execute("SELECT topic_id,id,timestamp,data FROM messages WHERE topic_id IN (1,2) ORDER BY id LIMIT ?", (len(frames),)).fetchall()
                    self.assertEqual(frames, rows)
                    self.assertGreater(db.execute("SELECT count(*) FROM messages").fetchone()[0], len(frames))
                    self.assertEqual(db.execute("PRAGMA integrity_check").fetchone()[0], "ok")
                metadata = json.loads((directory / "rosbag2/metadata.yaml").read_text())
                self.assertEqual(metadata["rosbag2_bagfile_information"]["message_count"],
                                 sum(json.loads(stopped.stdout)["counts"].values()))
                self.assertTrue(rr.inventory(directory)["files"])
                if camera:
                    self.assertGreater(json.loads(stopped.stdout)["counts"]["3"], 2)
            finally:
                if rr.alive(rr.read_state(directory)):
                    (directory / "stop.request").touch()
                    wait_for(lambda: not rr.alive(rr.read_state(directory)))
                for stream in streams:
                    if stream.poll() is None:
                        stream.kill()
                        stream.wait()
                    stream.stdout.close()
                if camera:
                    camera_stop.set()
                    camera_thread.join()
                    camera_socket.close(linger=0)
                    camera_context.term()

    def test_all_remote_files_parse_on_g1_python38(self):
        for name in ("remote_record.py", "camera_wire.py", "sensor_stream.py"):
            ast.parse((TOOLS / "g1_mapping_tools" / name).read_text(), feature_version=(3, 8))


class CollectionRecoveryTest(unittest.TestCase):
    def test_interrupted_copy_resumes_and_verifies_original_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            settings = replace(make_settings(root), runtime="native", transport="ssh")
            session = SessionManager(settings).create(name="recover", backend="raw", diagnostic={})
            original = bytes(range(256)) * 4096
            import hashlib
            inventory = {"state": {"status": "completed", "dropped": 0, "counts": {"1": 10, "2": 200, "3": 5}},
                         "files": [{"name": "rosbag2/capture.db3", "size": len(original),
                                    "sha256": hashlib.sha256(original).hexdigest()}]}
            source = root / "original"
            source.write_bytes(original)
            recording = RemoteRecording(settings, session)
            offsets = []

            def command(action, *args):
                offset = int(args[args.index("--offset") + 1])
                offsets.append(offset)
                if len(offsets) == 1:
                    return [sys.executable, "-c", "import sys;sys.stdout.buffer.write(open(sys.argv[1],'rb').read(12345));sys.exit(1)", str(source)]
                return [sys.executable, "-c", "import sys;f=open(sys.argv[1],'rb');f.seek(int(sys.argv[2]));sys.stdout.buffer.write(f.read())", str(source), str(offset)]

            with patch.object(recording, "request", return_value=inventory), patch.object(recording, "command", side_effect=command), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "続きから"):
                    recording.collect()
                destination = recording.collect()
                inventory["files"][0]["sha256"] = "0" * 64
                with self.assertRaisesRegex(RuntimeError, "照合"):
                    recording.collect()
            self.assertEqual(offsets, [0, 12345, 0])
            self.assertEqual((destination / "capture.db3").read_bytes(), original)
            self.assertTrue(source.exists())

    def test_valid_preview_cannot_be_reported_as_a_complete_remote_session(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = replace(make_settings(Path(temporary)), runtime="native", transport="ssh")
            session = SessionManager(settings).create(name="preview", backend="raw", diagnostic={})
            write_mock_artifacts(session)
            valid, report = validate_session(session.directory, min_map_points=1, allow_partial=True)
            self.assertFalse(valid)
            statuses = {check["name"]: check["status"] for check in report["checks"]}
            self.assertEqual(statuses["g1.originals"], "FAIL")
            self.assertEqual(statuses["g1.final_replay"], "FAIL")

    def test_stop_failure_stays_pending_and_recover_finishes_same_session(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings = replace(make_settings(Path(temporary)), runtime="native", transport="ssh")
            manager = SessionManager(settings)
            session = manager.create(name="pending", backend="raw", diagnostic={})
            with patch.object(NativeRuntime, "save_and_stop"), patch.object(NativeRuntime, "collect_logs"), \
                 patch.object(NativeRuntime, "finalize_remote", side_effect=RuntimeError("wifi disconnected")), \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(_stop(settings, _parser().parse_args(["stop"])), 1)
            self.assertEqual(manager.active(), session)
            self.assertEqual(manager.state(session)["status"], "stopping")
            with patch.object(NativeRuntime, "_stop_processes"), patch.object(NativeRuntime, "finalize_remote") as finalize, \
                 patch("g1_mapping.cli.validate_session", return_value=(True, {"checks": [], "success": True})), \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(_recover(settings, _parser().parse_args(["recover"])), 0)
            finalize.assert_called_once_with(session)
            self.assertIsNone(manager.active())
            self.assertEqual(manager.state(session)["status"], "completed")


class CameraCdrTest(unittest.TestCase):
    def test_camera_cdr_roundtrip_with_ros(self):
        try:
            from rclpy.serialization import deserialize_message
            from sensor_msgs.msg import CompressedImage, CameraInfo
            from std_msgs.msg import String
        except ImportError:
            self.skipTest("ROS serialization unavailable")
        jpeg = b"\xff\xd8\xff\xc0\x00\x11\x08\x01\x68\x02\x80" + b"\x00" * 10
        config = dict(camera_frame_id="camera", camera_fx=450., camera_fy=451., camera_cx=320., camera_cy=180.)
        stamp = 1_700_000_000_123456789
        image, info, metadata = [deserialize_message(data, kind) for data, kind in
                                 zip(camera_records(jpeg, stamp, config, 7), (CompressedImage, CameraInfo, String))]
        self.assertEqual(bytes(image.data), jpeg)
        self.assertEqual(image.header.stamp.sec, stamp // 10**9)
        self.assertEqual(image.header.stamp.nanosec, stamp % 10**9)
        self.assertEqual((info.width, info.height), (640, 360))
        self.assertEqual(list(info.k), [450., 0., 320., 0., 451., 180., 0., 0., 1.])
        self.assertEqual(json.loads(metadata.data)["sequence"], 7)
