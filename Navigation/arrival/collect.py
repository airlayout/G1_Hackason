"""Subscribers only. No publishers, action clients, LocoClient or motor commands."""
from __future__ import annotations

import json
from pathlib import Path
import struct
import time
import uuid
import zlib

import numpy as np

from .session import Writer, read_json


def decode_rgbd(payload):
    """Decode the existing G1RGBD1 single-message wire format independently."""
    import cv2

    if len(payload) < 12 or payload[:8] != b"G1RGBD1\0":
        raise ValueError("expected G1RGBD1 aligned RGB-D, not the legacy RGB stream")
    size = struct.unpack("<I", payload[8:12])[0]
    header = json.loads(payload[12:12+size])
    color, depth, intrinsics = header["color"], header["depth"], header["intrinsics"]
    if (header["version"] != 1 or color["encoding"] != "jpeg" or color["order"] != "bgr"
            or depth["dtype"] != "uint16le" or depth["aligned_to"] != "color"
            or depth["encoding"] not in ("none", "zlib")):
        raise ValueError("unsupported/misaligned RGB-D encoding")
    offset = 12+size
    if len(payload) != offset+color["nbytes"]+depth["nbytes"]:
        raise ValueError("truncated/oversized RGB-D payload")
    image = cv2.imdecode(np.frombuffer(payload[offset:offset+color["nbytes"]], np.uint8), cv2.IMREAD_COLOR)
    raw = payload[offset+color["nbytes"]:]
    raw = zlib.decompress(raw) if depth["encoding"] == "zlib" else raw
    array = np.frombuffer(raw, "<u2").reshape(depth["height"], depth["width"])
    scale = float(depth["scale_m"])
    if image is None or image.shape[:2] != array.shape or not np.isfinite(scale) or scale <= 0:
        raise ValueError("invalid color/depth dimensions or scale")
    if any((metadata.get("height"), metadata.get("width")) != array.shape for metadata in (color, intrinsics)):
        raise ValueError("intrinsics/color dimensions do not match aligned depth")
    K = np.array([[intrinsics["fx"], 0, intrinsics["cx"]], [0, intrinsics["fy"], intrinsics["cy"]], [0, 0, 1.]])
    if not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("invalid camera intrinsics")
    if any(abs(float(c)) > 1e-9 for c in intrinsics.get("coeffs", [])):
        raise ValueError("rectified RGB-D with zero distortion coefficients is required")
    stamp = float(header["timestamp"])
    if not np.isfinite(stamp):
        raise ValueError("invalid camera timestamp")
    return header, image[..., ::-1], array.astype(np.float32)*scale, K


def record_camera(args):
    import zmq

    root = Path(args.session)
    manifest = read_json(root/"session.json")
    if manifest["clock"]["id"] != "unix":
        raise ValueError("live camera recorder uses Unix timestamps")
    writer = Writer(root, "frames")
    root.joinpath("frames").mkdir(exist_ok=True)
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.connect(f"tcp://{args.host}:{args.port}")
    started, last_saved, last_source, count = time.monotonic(), -float("inf"), None, 0
    try:
        while time.monotonic()-started < args.duration:
            try:
                payload = sock.recv()
            except zmq.Again:
                continue
            received = time.time()
            header, rgb, depth, K = decode_rgbd(payload)
            source = (header["camera"], header["frame_id"], header["timestamp"])
            if source == last_source or time.monotonic()-last_saved < 1/args.fps:
                continue
            frame_id = uuid.uuid4().hex
            relative = f"frames/{frame_id}.npz"
            np.savez_compressed(root/relative, rgb=rgb, depth_m=depth, K=K)
            writer.append({"id": frame_id, "source_frame_id": header["frame_id"], "camera": header["camera"],
                           "optical_frame": args.optical_frame, "t": header["timestamp"], "received_at": received,
                           "clock_id": "unix", "stamp_kind": "exposure" if args.exposure_verified else "source_timestamp_unverified",
                           "depth_aligned": True, "path": relative})
            last_source, last_saved = source, time.monotonic()
            count += 1
    finally:
        writer.close()
        sock.close(linger=0)
        ctx.term()
    print(f"camera frames: {count}")


def record_body(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    manifest = read_json(Path(args.session)/"session.json")
    if manifest["clock"]["id"] != "unix":
        raise ValueError("DDS recorder uses Unix receive timestamps")
    ChannelFactoryInitialize(args.domain, args.interface)
    writer = Writer(args.session, "body")
    previous = -float("inf")

    def receive(msg):
        nonlocal previous
        now = time.monotonic()
        if now-previous < 1/args.hz:
            return
        previous = now
        writer.append({"kind": "body", "t": time.time(), "clock_id": "unix", "stamp_kind": "receive",
                       "time_uncertainty_s": args.time_uncertainty,
                       "device_tick": int(getattr(msg, "tick", 0)),
                       "q": [float(m.q) for m in msg.motor_state[:29]],
                       "dq": [float(m.dq) for m in msg.motor_state[:29]],
                       "imu_wxyz": list(map(float, msg.imu_state.quaternion)),
                       "imu_gyro": list(map(float, msg.imu_state.gyroscope)),
                       "imu_accel": list(map(float, msg.imu_state.accelerometer))})

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(receive, 10)
    try:
        started = time.monotonic()
        while time.monotonic()-started < args.duration:
            time.sleep(.05)
    finally:
        # Stop the callback before closing its file.
        sub.Close()
        writer.close()


def record_ros(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    manifest = read_json(Path(args.session)/"session.json")
    if manifest["clock"]["id"] != "unix":
        raise ValueError("live ROS recorder expects a Unix ROS clock; simulated time needs an offline dataset")
    rclpy.init()
    node = Node("arrival_measurement_recorder")
    writer = Writer(args.session, "ros")
    buf = Buffer()
    listener = TransformListener(buf, node)
    subscriptions = []

    def stamp(msg):
        return msg.sec+msg.nanosec*1e-9

    def pose(kind, t, frame_id, child, p, q):
        writer.append({"kind": kind, "t": t, "received_at": time.time(), "frame_id": frame_id,
                       "child_frame_id": child, "xyz": [p.x, p.y, p.z], "xyzw": [q.x, q.y, q.z, q.w]})

    def odom(msg):
        pose("odom", stamp(msg.header.stamp), msg.header.frame_id, msg.child_frame_id,
             msg.pose.pose.position, msg.pose.pose.orientation)

    def nav_pose():
        try:
            value = buf.lookup_transform("map", args.base_frame, Time())
            pose("nav_pose", stamp(value.header.stamp), "map", args.base_frame,
                 value.transform.translation, value.transform.rotation)
        except Exception as exc:
            node.get_logger().debug(str(exc))

    def status(msg):
        names = {1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded", 5: "canceled", 6: "aborted"}
        for item in msg.status_list:
            writer.append({"kind": "nav_status", "t": node.get_clock().now().nanoseconds*1e-9,
                           "received_at": time.time(), "goal_id": bytes(item.goal_info.goal_id.uuid).hex(),
                           "goal_time": stamp(item.goal_info.stamp), "status": names.get(item.status, "unknown")})

    def goal(msg):
        pose("goal_pose", stamp(msg.header.stamp), msg.header.frame_id, args.base_frame,
             msg.pose.position, msg.pose.orientation)

    def velocity(msg, topic):
        writer.append({"kind": "velocity_command", "t": node.get_clock().now().nanoseconds*1e-9,
                       "source": topic, "linear": [msg.linear.x, msg.linear.y, msg.linear.z],
                       "angular": [msg.angular.x, msg.angular.y, msg.angular.z]})

    subscriptions.append(node.create_subscription(Odometry, args.odom_topic, odom, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(PoseStamped, "/goal_pose", goal, 10))
    for action in ("navigate_to_pose", "navigate_through_poses"):
        subscriptions.append(node.create_subscription(GoalStatusArray, f"/{action}/_action/status", status, 10))
    for topic in ("/cmd_vel", "/cmd_vel_smoothed"):
        subscriptions.append(node.create_subscription(Twist, topic, lambda m, t=topic: velocity(m, t), 10))
    for topic in ("/g1/bridge_status", "/g1/patrol/status"):
        subscriptions.append(node.create_subscription(String, topic,
            lambda m, t=topic: writer.append({"kind": "health", "t": time.time(), "source": t, "data": m.data}), 10))
    timer = node.create_timer(.05, nav_pose)
    try:
        started = time.monotonic()
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        writer.close()
    assert listener is not None and timer is not None and subscriptions
