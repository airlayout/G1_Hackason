"""Replay G1 originals through FAST-LIO2, then rebuild OctoMap from exact pairs."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

from .remote_record import SCHEMA, write_metadata


def replay(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message, serialize_message
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2

    output = args.output_dir
    bag = output / "rosbag2"
    bag.mkdir()
    db = sqlite3.connect(str(bag / "capture.db3"))
    db.executescript(SCHEMA)
    topics = [("/g1_mapping/cloud_registered", "sensor_msgs/msg/PointCloud2"),
              ("/g1_mapping/sensor_pose", "geometry_msgs/msg/PoseStamped"),
              ("/g1_mapping/odom", "nav_msgs/msg/Odometry")]
    for i, (topic, kind) in enumerate(topics, 1):
        db.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (i, topic, kind, "cdr", ""))
    rclpy.init()
    node = rclpy.create_node("g1_final_replay")
    qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)
    sensor_qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = {args.points_topic: node.create_publisher(PointCloud2, args.points_topic, qos),
                  args.imu_topic: node.create_publisher(Imu, args.imu_topic, qos)}
    message_types = {args.points_topic: PointCloud2, args.imu_topic: Imu}
    clock = node.create_publisher(Clock, "/clock", 10)
    received, stamps = {1: 0, 2: 0, 3: 0}, {1: set(), 2: set()}
    last_output = [time.monotonic()]
    trajectory = (output / "trajectory.tum").open("w")
    trajectory.write("# timestamp tx ty tz qx qy qz qw\n")

    def callback(kind, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                   (kind, stamp, serialize_message(message)))
        received[kind] += 1
        last_output[0] = time.monotonic()
        if kind in stamps:
            stamps[kind].add(stamp)
        else:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            trajectory.write(f"{stamp / 1e9:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                             f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")

    subscriptions = [node.create_subscription(kind, topic, lambda msg, i=i: callback(i, msg), sensor_qos)
                     for i, ((topic, _), kind) in enumerate(zip(topics, (PointCloud2, PoseStamped, Odometry)), 1)]
    launch = None
    total_lidar = 0
    try:
        launch = subprocess.Popen(["ros2", "launch", "g1_mapping_bringup", "offline_mapping.launch.py"],
                                  start_new_session=True)

        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError("FAST-LIO replay pipeline exited")
                rclpy.spin_once(node, timeout_sec=min(0.02, max(0., deadline - time.monotonic())))

        deadline = time.monotonic() + 60
        while not (all(node.count_subscribers(topic) for topic in publishers) and
                   any(name == "/g1_mapping/save_map" for name, _ in node.get_service_names_and_types())):
            if time.monotonic() > deadline:
                raise RuntimeError("replay nodes did not become ready")
            spin(0.1)
        spin(1.)
        first_stamp, start = None, time.monotonic()
        # G1 recorder currently produces one DB. Refuse ambiguous split bags rather than reorder them.
        sources = sorted(args.bag_dir.glob("*.db3"))
        if len(sources) != 1:
            raise RuntimeError("expected one complete G1 capture database")
        with sqlite3.connect(f"file:{sources[0]}?mode=ro", uri=True) as source:
            rows = source.execute("SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                                  "WHERE t.name IN (?,?) ORDER BY m.timestamp,m.id",
                                  (args.points_topic, args.imu_topic))
            for topic, stamp, payload in rows:
                if first_stamp is None:
                    first_stamp = stamp
                target = start + (stamp - first_stamp) / 1e9 / args.rate
                spin(max(0., target - time.monotonic()))
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(stamp, 10**9)
                clock.publish(message)
                publishers[topic].publish(deserialize_message(payload, message_types[topic]))
                if topic == args.points_topic:
                    total_lidar += 1
                rclpy.spin_once(node, timeout_sec=0.)
                if launch.poll() is not None:
                    raise RuntimeError("FAST-LIO replay pipeline exited")
        spin(3.)
        deadline = time.monotonic() + 30
        while time.monotonic() - last_output[0] < 2 and time.monotonic() < deadline:
            spin(0.1)
        if stamps[1] != stamps[2] or received[1] != len(stamps[1]) or received[2] != len(stamps[2]):
            raise RuntimeError("replayed scans and sensor poses do not pair exactly")
        if received[1] < max(1, total_lidar - 10):
            raise RuntimeError(f"too many replay scans missing: {received[1]}/{total_lidar}")
        (output / "replay.json").write_text(json.dumps({"raw_lidar_scans": total_lidar,
                                                       "registered_scans": received[1],
                                                       "sensor_poses": received[2]}, indent=2) + "\n")
    finally:
        if launch is not None and launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        trajectory.close()
        db.commit()
        write_metadata(output, db, topics)
        db.close()
        node.destroy_node()
        rclpy.shutdown()
    options = {
        "voxel-size": os.environ.get("MAP_VOXEL_SIZE", "0.05"),
        "target-scans": os.environ.get("DENSITY_TARGET_SCANS", "10"),
        "max-points": os.environ.get("MAP_MAX_POINTS", "2000000"),
        **{name.replace("_", "-"): os.environ.get("OCTOMAP_" + name.upper(), default)
           for name, default in {"resolution": "0.1", "min_range": "0.3", "max_range": "3",
                                 "prob_hit": "0.7", "prob_miss": "0.4", "clamp_min": "0.12",
                                 "clamp_max": "0.97", "keep_unknown": "true"}.items()},
    }
    subprocess.run(["ros2", "run", "g1_mapping_octomap", "rebuild_occupancy",
                    "--bag-dir", str(bag), "--output", str(output / "map_raw.pcd"),
                    *[value for key, val in options.items() for value in ("--" + key, val)]], check=True)
    metadata = json.loads((output / "map_raw.octomap.json").read_text())
    if metadata["scans"] != received[1] or metadata.get("candidate_limit_drops", 0):
        raise RuntimeError("final OctoMap lost scans or exceeded its point capacity; originals retained")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--points-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--rate", type=float, default=0.8)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("positive replay rate required")
    replay(args)


if __name__ == "__main__":
    main()
