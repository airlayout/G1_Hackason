"""Carry original LiDAR/IMU CDR over SSH; the G1 side only creates DDS readers.

The same file runs on G1 Python 3.8 (--remote) and on the ROS 2 host. Sensor
timestamps, point fields and CDR bytes cross the wire without modification.
"""

from __future__ import annotations

import argparse
import queue
import shlex
import signal
import struct
import subprocess
import sys
import threading
import time

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

    remote_command = shlex.join(
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
    stopped = threading.Event()
    errors = []
    last_received = [time.monotonic()]
    counts = {POINTS: 0, IMU: 0}
    process = subprocess.Popen(
        [
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
            remote_command,
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )

    def read_sensors():
        try:
            while not stopped.is_set():
                frame = read_frame(process.stdout)
                if frame is None:
                    raise EOFError("SSH sensor stream ended")
                kind, payload = frame
                message = deserialize_message(payload, message_types[kind])
                if stopped.is_set() or not rclpy.ok():
                    break
                publishers[kind].publish(message)
                last_received[0] = time.monotonic()
                counts[kind] += 1
        except Exception as error:  # noqa: BLE001 -- surface all reader-thread failures to the watchdog
            if not stopped.is_set():
                errors.append(str(error))

    reader = threading.Thread(target=read_sensors, daemon=True)
    reader.start()

    def watchdog():
        if not errors and time.monotonic() - last_received[0] > 10:
            errors.append("no sensor frames for 10 seconds")
        if errors:
            node.get_logger().error(errors[-1])
            stopped.set()
            rclpy.try_shutdown()

    node.create_timer(0.5, watchdog)
    node.get_logger().info(f"G1 {args.host} -> SSH -> local LiDAR/IMU")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        # SSH stdin EOF stops only this remote reader, without a daemon or pkill.
        try:
            process.stdin.close()
        except BrokenPipeError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        reader.join(timeout=2)
        process.stdout.close()
        print(
            f"[SENSORS] local closed: lidar={counts[POINTS]} imu={counts[IMU]}",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if errors else 0


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
    args = parser.parse_args()
    if not args.remote and not args.host:
        parser.error("--host is required on the mapping PC")
    return remote_main(args) if args.remote else host_main(args)


if __name__ == "__main__":
    sys.exit(main())
