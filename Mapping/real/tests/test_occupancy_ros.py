"""ROS integration: normalized sensor poses -> live clearing -> save -> db3 rebuild.

Runs in the mapping image after sourcing its workspace; skipped on ROS-free hosts.
"""
from __future__ import annotations

import os
from pathlib import Path
import sqlite3
import struct
import subprocess
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.serialization import serialize_message
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from ament_index_python.packages import get_package_prefix
    get_package_prefix("g1_mapping_octomap")
except (ImportError, LookupError) as error:
    raise unittest.SkipTest(f"ROS mapping workspace unavailable: {error}")


def cloud(stamp: int, points, frame="map"):
    msg = PointCloud2()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = frame
    msg.height, msg.width = 1, len(points)
    msg.point_step, msg.row_step = 16, 16 * len(points)
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
    msg.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return msg


def pose(stamp: int, frame="map"):
    msg = PoseStamped()
    msg.header.stamp.sec, msg.header.frame_id = stamp, frame
    # Deliberately far from the world origin: using IMU origin/identity TF would fail clearing.
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 10.05, 0.05, 0.55
    msg.pose.orientation.w = 1.0
    return msg


class OccupancyROSTest(unittest.TestCase):
    def test_live_save_and_metadata_free_rebuild_match(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output, rebuilt = root / "live.pcd", root / "rebuilt.pcd"
            env = dict(os.environ, ROS_DOMAIN_ID=str(170 + os.getpid() % 40))
            previous_domain = os.environ.get("ROS_DOMAIN_ID")
            os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
            rclpy.init()
            node = rclpy.create_node("occupancy_integration_test")
            processes = []
            log = (root / "nodes.log").open("w")
            try:
                processes.append(subprocess.Popen([
                    "ros2", "run", "g1_mapping_tools", "mapping_adapter", "--ros-args",
                    "-p", "source_cloud:=/test/cloud", "-p", "source_sensor_pose:=/test/pose",
                ], env=env, stdout=log, stderr=log))
                processes.append(subprocess.Popen([
                    "ros2", "run", "g1_mapping_octomap", "occupancy_mapper", "--ros-args",
                    "-p", f"output_path:={output}", "-p", "publish_hz:=10.0",
                ], env=env, stdout=log, stderr=log))
                cloud_pub = node.create_publisher(PointCloud2, "/test/cloud", 20)
                pose_pub = node.create_publisher(PoseStamped, "/test/pose", 20)
                received = []
                subscription = node.create_subscription(PointCloud2, "/g1_mapping/map", received.append, 10)
                client = node.create_client(Trigger, "/g1_mapping/save_map")

                def wait_for(predicate, timeout=15):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        if predicate():
                            return
                        for process in processes:
                            if process.poll() is not None:
                                self.fail((root / "nodes.log").read_text())
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.fail("ROS timeout\n" + (root / "nodes.log").read_text())

                wait_for(lambda: cloud_pub.get_subscription_count() and pose_pub.get_subscription_count()
                         and node.count_subscribers("/g1_mapping/sensor_pose") and client.service_is_ready())
                time.sleep(0.25)  # Allow DDS endpoint discovery in the mapper as well.
                premature = subprocess.run([
                    "ros2", "run", "g1_mapping_tools", "save_occupancy_map", "--timeout", "2",
                ], env=env, text=True, capture_output=True, timeout=10)
                self.assertNotEqual(premature.returncode, 0)
                self.assertIn("no synchronized scans", premature.stderr)
                self.assertFalse(output.exists())
                # A cloud without its exact pose must not use any subsequently received pose.
                cloud_pub.publish(cloud(90, [(30.0, 0.05, 0.55, 1.0)], "camera_init"))
                pose_pub.publish(pose(91, "camera_init"))
                ghost, wall = (11.05, 0.05, 0.55, 10.0), (12.55, 0.05, 0.55, 20.0)
                frames = [(cloud(1, [ghost, wall]), pose(1))]
                frames.extend((cloud(i, [wall]), pose(i)) for i in range(2, 13))
                for c, p in frames:
                    c.header.frame_id = p.header.frame_id = "camera_init"
                    # Alternate arrival order to exercise exact stamp pairing.
                    if c.header.stamp.sec % 2:
                        cloud_pub.publish(c); pose_pub.publish(p)
                    else:
                        pose_pub.publish(p); cloud_pub.publish(c)
                    wait_for(lambda: received and received[-1].header.stamp.sec == c.header.stamp.sec)
                    if c.header.stamp.sec == 1:
                        self.assertEqual(received[-1].width, 2)
                    c.header.frame_id = p.header.frame_id = "map"
                self.assertEqual(received[-1].width, 1, "the ghost must be removed in the live map")
                self.assertAlmostEqual(struct.unpack_from("<f", received[-1].data)[0], wall[0], places=4)
                future = client.call_async(Trigger.Request())
                wait_for(future.done)
                self.assertTrue(future.result().success, future.result().message)
                self.assertTrue(output.with_suffix(".bt").is_file())

                # No metadata.yaml: interrupted-record recovery uses SQLite/CDR directly.
                bag = root / "bag"; bag.mkdir()
                with sqlite3.connect(bag / "scan_0.db3") as db:
                    db.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);"
                                     "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);")
                    db.executemany("INSERT INTO topics VALUES(?,?,?)", [
                        (1, "/g1_mapping/cloud_registered", "sensor_msgs/msg/PointCloud2"),
                        (2, "/g1_mapping/sensor_pose", "geometry_msgs/msg/PoseStamped")])
                    for c, p in frames:
                        stamp = c.header.stamp.sec * 1_000_000_000
                        db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(1,?,?)", (stamp, serialize_message(c)))
                        db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(2,?,?)", (stamp + 1, serialize_message(p)))
                result = subprocess.run([
                    "ros2", "run", "g1_mapping_octomap", "rebuild_occupancy", "--bag-dir", str(bag),
                    "--output", str(rebuilt),
                ], env=env, text=True, capture_output=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(output.read_bytes(), rebuilt.read_bytes())
                self.assertEqual(output.with_suffix(".octomap.json").read_bytes(),
                                 rebuilt.with_suffix(".octomap.json").read_bytes())
                # A missing pose must fail, never silently produce an append-only map.
                with sqlite3.connect(bag / "scan_0.db3") as db:
                    db.execute("DELETE FROM messages WHERE topic_id=2")
                result = subprocess.run([
                    "ros2", "run", "g1_mapping_octomap", "rebuild_occupancy", "--bag-dir", str(bag),
                    "--output", str(root / "should_not_exist.pcd"),
                ], env=env, text=True, capture_output=True, timeout=30)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("sensor_pose", result.stderr)
                self.assertFalse((root / "should_not_exist.pcd").exists())
                node.destroy_subscription(subscription)
            finally:
                for process in processes:
                    process.terminate()
                for process in processes:
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                node.destroy_node(); rclpy.shutdown(); log.close()
                if previous_domain is None: os.environ.pop("ROS_DOMAIN_ID", None)
                else: os.environ["ROS_DOMAIN_ID"] = previous_domain


if __name__ == "__main__":
    unittest.main()
