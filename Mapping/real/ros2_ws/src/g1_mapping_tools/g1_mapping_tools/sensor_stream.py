"""Carry original LiDAR/IMU CDR over SSH; the G1 side only creates DDS readers.

The same file runs on G1 Python 3.8 (--remote) and on the ROS 2 host. Sensor
timestamps, point fields and CDR bytes cross the wire without modification.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import shlex
import signal
import struct
import subprocess
import sys
import threading
import time

try:
    from .remote_record import JOURNAL_HEADER, JOURNAL_MAGIC
except ImportError:
    from remote_record import JOURNAL_HEADER, JOURNAL_MAGIC

HEADER = struct.Struct("!4sBI")
MAGIC = b"G1S1"
MAX_PAYLOAD = 16 * 1024 * 1024
POINTS, IMU = 1, 2


def _read_exact(stream, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            if remaining == size:
                return None
            raise EOFError("truncated sensor stream")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(stream):
    header = _read_exact(stream, HEADER.size)
    if header is None:
        return None
    magic, kind, size = HEADER.unpack(header)
    if magic != MAGIC or kind not in (POINTS, IMU) or not 0 < size <= MAX_PAYLOAD:
        raise ValueError("invalid sensor frame")
    payload = _read_exact(stream, size)
    if payload is None:
        raise EOFError("missing sensor payload")
    return kind, payload


def write_frame(stream, kind, payload):
    if kind not in (POINTS, IMU) or not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError("invalid sensor frame")
    for part in (HEADER.pack(MAGIC, kind, len(payload)), payload):
        view = memoryview(part)
        while view:
            written = stream.write(view)
            if not written:
                raise BrokenPipeError("sensor stream closed")
            view = view[written:]
    stream.flush()


def read_journal_frame(stream):
    header = _read_exact(stream, JOURNAL_HEADER.size)
    if header is None:
        return None
    magic, kind, sequence, stamp, size = JOURNAL_HEADER.unpack(header)
    if magic != JOURNAL_MAGIC or kind not in (POINTS, IMU) or sequence <= 0 or stamp <= 0 or not 0 < size <= MAX_PAYLOAD:
        raise ValueError("invalid journal frame")
    payload = _read_exact(stream, size)
    if payload is None:
        raise EOFError("missing journal payload")
    return kind, sequence, stamp, payload


def remote_main(args):
    # Keep all SDK diagnostics off the binary SSH stdout channel.
    original_stdout = sys.stdout
    wire = original_stdout.buffer
    sys.stdout = sys.stderr
    sys.path.insert(0, args.sdk_path)
    import idl_imu  # noqa: F401 -- shipped beside this script; no SDK files changed
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_, PointCloud2_

    stopped = threading.Event()
    overflow = threading.Event()
    frames = queue.Queue(maxsize=512)
    counts = {POINTS: 0, IMU: 0}
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())

    def connection_closed():
        sys.stdin.buffer.read(1)
        stopped.set()

    threading.Thread(target=connection_closed, daemon=True).start()

    def receive(kind):
        def callback(message):
            try:
                frames.put_nowait((kind, message.serialize(use_version_2=False)))
            except queue.Full:
                overflow.set()

        return callback

    ChannelFactoryInitialize(0, args.iface)
    subscribers = []
    try:
        for kind, topic, message_type in (
            (POINTS, args.points_topic, PointCloud2_),
            (IMU, args.imu_topic, Imu_),
        ):
            subscriber = ChannelSubscriber("rt" + topic, message_type)
            subscriber.Init(receive(kind), 64)
            subscribers.append(subscriber)
        print(
            f"[SENSORS] receive-only DDS on {args.iface} -> SSH CDR", flush=True
        )
        while not stopped.is_set():
            if overflow.is_set():
                raise RuntimeError(
                    "wireless sensor queue overflow; recording must be checked"
                )
            try:
                kind, payload = frames.get(timeout=0.2)
            except queue.Empty:
                continue
            write_frame(wire, kind, payload)
            counts[kind] += 1
    except BrokenPipeError:
        pass
    finally:
        for subscriber in subscribers:
            subscriber.Close()
        print(
            f"[SENSORS] closed: lidar={counts[POINTS]} imu={counts[IMU]}",
            flush=True,
        )
    return 0


def host_main(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String

    remote_arguments = (
        [
            "python3",
            "-u",
            args.remote_script,
            "--remote",
            "--iface",
            args.iface,
            "--sdk-path",
            args.sdk_path,
            "--points-topic",
            args.points_topic,
            "--imu-topic",
            args.imu_topic,
        ]
    )
    rclpy.init()
    node = rclpy.create_node("g1_ssh_sensor_bridge")
    # SSH can deliver several sensor samples together; retain the short burst.
    qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
    publishers = {
        POINTS: node.create_publisher(PointCloud2, args.points_topic, qos),
        IMU: node.create_publisher(Imu, args.imu_topic, qos),
    }
    message_types = {POINTS: PointCloud2, IMU: Imu}
    status_publisher = node.create_publisher(String, "/g1_mapping/connection", 1)
    stopped = threading.Event()
    last_received = [time.monotonic()]
    counts = {POINTS: 0, IMU: 0}
    active = [None]
    cursor = [0]
    status = {"state": "connecting", "sequence": 0, "reconnections": 0}
    status_lock = threading.Lock()
    ssh_arguments = [
            "ssh",
            "-T",
            "-o",
            "IPQoS=none",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "ServerAliveInterval=3",
            "-o",
            "ServerAliveCountMax=2",
            args.host,
        ]

    def publish_status(state, **values):
        with status_lock:
            status.update(state=state, sequence=cursor[0], updated_ns=time.time_ns(), **values)

    def read_sensors():
        # Do not consume the start of the journal before FAST-LIO's input adapter joins.
        while args.session_dir and not stopped.is_set():
            if all(any(info.node_name == "g1_sensor_adapter" for info in node.get_subscriptions_info_by_topic(topic))
                   for topic in (args.points_topic, args.imu_topic)):
                break
            stopped.wait(0.1)
        while not stopped.is_set():
            process = None
            try:
                arguments = remote_arguments
                if args.session_dir:
                    arguments = ["python3", "-u", args.journal_script, "stream",
                                 "--directory", args.session_dir, "--after", str(cursor[0])]
                process = subprocess.Popen(ssh_arguments + [shlex.join(arguments)],
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                active[0] = process
                last_received[0] = time.monotonic()
                previous_stamp, previous_wall = None, None
                while not stopped.is_set():
                    frame = read_journal_frame(process.stdout) if args.session_dir else read_frame(process.stdout)
                    if frame is None:
                        raise EOFError("SSH sensor stream ended")
                    if args.session_dir:
                        kind, sequence, stamp, payload = frame
                        if sequence <= cursor[0]:
                            raise ValueError("journal sequence moved backwards")
                        # Bound catch-up speed so a reconnection does not flood FAST-LIO queues.
                        now = time.monotonic()
                        if previous_stamp is not None:
                            delay = max(0., (stamp - previous_stamp) / 1e9 / args.catchup_rate - (now - previous_wall))
                            if stopped.wait(min(delay, 1.0)):
                                break
                        previous_stamp, previous_wall = stamp, time.monotonic()
                    else:
                        kind, payload = frame
                    message = deserialize_message(payload, message_types[kind])
                    if stopped.is_set() or not rclpy.ok():
                        break
                    publishers[kind].publish(message)
                    last_received[0] = time.monotonic()
                    counts[kind] += 1
                    if args.session_dir:
                        cursor[0] = sequence
                        lag = max(0., (time.time_ns() - stamp) / 1e9)
                        publish_status("catching_up" if lag > 2 else "connected", lag_seconds=round(lag, 2))
                    else:
                        publish_status("connected")
            except Exception as error:
                if not stopped.is_set():
                    publish_status("disconnected", error=str(error), reconnections=status["reconnections"] + 1)
                    node.get_logger().warning(f"{error}; reconnecting, journal cursor={cursor[0]}")
            finally:
                active[0] = None
                if process is not None:
                    process.stdin.close()
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    process.stdout.close()
            stopped.wait(1.0)

    reader = threading.Thread(target=read_sensors, daemon=True)
    reader.start()

    def watchdog():
        message = String()
        with status_lock:
            message.data = json.dumps(status)
        if args.status_path:
            path = Path(args.status_path)
            temporary = path.with_suffix(".tmp")
            temporary.write_text(message.data + "\n")
            temporary.replace(path)
        status_publisher.publish(message)
        process = active[0]
        if process is not None and time.monotonic() - last_received[0] > 10:
            process.terminate()  # unblock pipe read; the reader reconnects without exiting ROS

    node.create_timer(0.5, watchdog)
    node.get_logger().info(f"G1 {args.host} -> SSH -> local LiDAR/IMU")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        process = active[0]
        if process is not None:
            process.terminate()
        reader.join(timeout=10)
        print(
            f"[SENSORS] local closed: lidar={counts[POINTS]} imu={counts[IMU]}",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remote", action="store_true")
    parser.add_argument("--host")
    parser.add_argument(
        "--remote-script",
        default="/home/unitree/mapping_tools/wireless/sensor_stream.py",
    )
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--sdk-path", default="/home/unitree/unitree_sdk2_python")
    parser.add_argument("--points-topic", default="/utlidar/cloud_livox_mid360")
    parser.add_argument("--imu-topic", default="/utlidar/imu_livox_mid360")
    parser.add_argument("--session-dir", help="G1 detached recording directory; resume from committed rows")
    parser.add_argument("--journal-script")
    parser.add_argument("--status-path")
    parser.add_argument("--catchup-rate", type=float, default=1.5)
    args = parser.parse_args()
    if not args.remote and not args.host:
        parser.error("--host is required on the mapping PC")
    if not math.isfinite(args.catchup_rate) or args.catchup_rate <= 0 or (args.session_dir and not args.journal_script):
        parser.error("positive --catchup-rate and --journal-script are required")
    return remote_main(args) if args.remote else host_main(args)


if __name__ == "__main__":
    sys.exit(main())
