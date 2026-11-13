"""Detached G1 recorder and resumable disk-backed sensor stream (Python 3.8).

Only the capture process subscribes to DDS. SSH clients read committed SQLite
rows; their lifetime and network speed never control acquisition.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import sqlite3
import struct
import subprocess
import sys
import threading
import time

# Optional camera dependency is installed here, never into G1's system/LeRobot Python.
sys.path.insert(0, str(Path(__file__).resolve().parent / "deps"))

try:
    from .camera_wire import camera_records, decode_lerobot_message
except ImportError:
    from camera_wire import camera_records, decode_lerobot_message

JOURNAL_HEADER = struct.Struct("!4sBQQI")
JOURNAL_MAGIC = b"G1J1"
MAX_PAYLOAD = 16 * 1024 * 1024
SCHEMA = """
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
INSERT INTO schema VALUES(4, 'humble');
CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
 serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
 timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages(timestamp);
"""


def prepare(camera_enabled):
    if not camera_enabled:
        return
    try:
        import zmq  # noqa: F401
    except ImportError:
        # This release provides Python 3.8 aarch64 wheels for the G1 SDK interpreter.
        destination = Path(__file__).resolve().parent / "deps"
        subprocess.run([sys.executable, "-m", "pip", "install", "--disable-pip-version-check",
                        "--only-binary=:all:", "--no-deps", "--upgrade", "--target", str(destination),
                        "pyzmq==25.1.2"], check=True)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def read_state(directory):
    path = Path(directory) / "state.json"
    return json.loads(path.read_text()) if path.exists() else {}


def identity(pid):
    try:
        fields = Path("/proc/{}/stat".format(pid)).read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def alive(state):
    return bool(state.get("identity")) and identity(state.get("pid", 0)) == state["identity"]


def topics_for(config):
    topics = [(config["points_topic"], "sensor_msgs/msg/PointCloud2"),
              (config["imu_topic"], "sensor_msgs/msg/Imu")]
    if config.get("camera_enabled"):
        topics += [("/g1_camera/color/image/compressed", "sensor_msgs/msg/CompressedImage"),
                   ("/g1_camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
                   ("/g1_camera/frame_metadata", "std_msgs/msg/String")]
    return topics


def write_metadata(directory, db, topics):
    counts = dict(db.execute("SELECT topic_id, count(*) FROM messages GROUP BY topic_id"))
    first, last = db.execute("SELECT min(timestamp), max(timestamp) FROM messages").fetchone()
    payload = {"rosbag2_bagfile_information": {
        "version": 4, "storage_identifier": "sqlite3", "relative_file_paths": ["capture.db3"],
        "duration": {"nanoseconds": (last or 0) - (first or 0)},
        "starting_time": {"nanoseconds_since_epoch": first or 0},
        "message_count": sum(counts.values()), "compression_format": "", "compression_mode": "",
        "topics_with_message_count": [
            {"topic_metadata": {"name": name, "type": kind, "serialization_format": "cdr",
                                "offered_qos_profiles": ""}, "message_count": counts.get(i, 0)}
            for i, (name, kind) in enumerate(topics, 1)]}}
    # JSON is a YAML subset; rosbag2's yaml-cpp accepts this without PyYAML on G1.
    atomic_json(Path(directory) / "rosbag2" / "metadata.yaml", payload)
    return counts


def camera_loop(config, enqueue, stopped, errors):
    try:
        import zmq
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.setsockopt(zmq.RCVHWM, 32)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect("tcp://{}:{}".format(config.get("camera_record_host", "127.0.0.1"),
                                            config["camera_port"]))
        sequence = 0
        try:
            while not stopped.is_set():
                if not socket.poll(200):
                    continue
                decoded = decode_lerobot_message(socket.recv(), config["camera_name"])
                if decoded is None:
                    continue
                jpeg, stamp = decoded
                if stamp <= 0 or not jpeg.startswith(b"\xff\xd8"):
                    raise ValueError("invalid camera JPEG or capture timestamp")
                for kind, data in enumerate(camera_records(jpeg, stamp, config, sequence), 3):
                    enqueue(kind, time.time_ns(), data)
                sequence += 1
        finally:
            socket.close()
            context.term()
    except Exception as error:
        errors.append("camera: " + str(error))
        stopped.set()


def capture(directory, config):
    directory = Path(directory)
    # Prevent two sessions from silently competing for disk / leaving orphan recorders.
    with (directory.parent / ".capture.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _capture(directory, config)


def _capture(directory, config):
    state = {"pid": os.getpid(), "identity": identity(os.getpid()), "status": "starting",
             "counts": {}, "dropped": 0, "error": ""}
    atomic_json(directory / "state.json", state)
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    records = queue.Queue(maxsize=4096)
    errors, subscribers, threads = [], [], []
    db = None
    topics = topics_for(config)

    def enqueue(kind, stamp, data):
        if len(data) > MAX_PAYLOAD:
            errors.append("sensor payload exceeds limit")
            stopped.set()
            return
        try:
            records.put_nowait((kind, stamp, data))
        except queue.Full:
            state["dropped"] += 1
            errors.append("capture queue overflow")
            stopped.set()

    try:
        if shutil.disk_usage(directory).free < config["min_free_gib"] * 1024**3:
            raise RuntimeError("insufficient free space on G1")
        bag = directory / "rosbag2"
        bag.mkdir()
        db = sqlite3.connect(str(bag / "capture.db3"))
        db.execute("PRAGMA journal_mode=WAL")
        db.execute("PRAGMA synchronous=FULL")
        db.executescript(SCHEMA)
        for i, (name, kind) in enumerate(topics, 1):
            db.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (i, name, kind, "cdr", ""))
        db.commit()
        sys.path.insert(0, config["sdk_path"])
        import idl_imu  # noqa: F401 -- installed beside this script
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_, PointCloud2_
        ChannelFactoryInitialize(0, config["iface"])
        for kind, topic, message_type in ((1, config["points_topic"], PointCloud2_),
                                           (2, config["imu_topic"], Imu_)):
            subscriber = ChannelSubscriber("rt" + topic, message_type)
            def callback(message, kind=kind):
                try:
                    enqueue(kind, time.time_ns(), message.serialize(use_version_2=False))
                except Exception as error:
                    errors.append("sensor serialization: " + str(error))
                    stopped.set()
            subscriber.Init(callback, 512)
            subscribers.append(subscriber)
        if config.get("camera_enabled"):
            thread = threading.Thread(target=camera_loop, args=(config, enqueue, stopped, errors))
            thread.start()
            threads.append(thread)
        last_commit = 0.0
        last_sensor = {1: time.monotonic(), 2: time.monotonic(), 3: time.monotonic()}
        counts = {}
        while not stopped.is_set():
            if (directory / "stop.request").exists():
                break
            try:
                kind, stamp, data = records.get(timeout=0.1)
                db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (kind, stamp, data))
                counts[kind] = counts.get(kind, 0) + 1
                last_sensor[kind] = time.monotonic()
            except queue.Empty:
                pass
            now = time.monotonic()
            if now - last_commit >= 0.1:
                db.commit()
                state.update(counts={str(k): v for k, v in counts.items()}, updated_ns=time.time_ns())
                ready = counts.get(1, 0) and counts.get(2, 0)
                if config.get("camera_required"):
                    ready = ready and counts.get(3, 0)
                state["status"] = "recording" if ready else "starting"
                atomic_json(directory / "state.json", state)
                last_commit = now
                if shutil.disk_usage(directory).free < config["min_free_gib"] * 1024**3:
                    raise RuntimeError("G1 free space limit reached")
                for kind in (1, 2, *([3] if config.get("camera_required") else [])):
                    if now - last_sensor[kind] > 10:
                        raise RuntimeError("G1 source {} missing for 10 seconds".format(kind))
        if errors:
            raise RuntimeError("; ".join(errors))
    except Exception as error:
        state["error"] = str(error)
    finally:
        stopped.set()
        for subscriber in subscribers:
            subscriber.Close()
        for thread in threads:
            thread.join(timeout=5)
        if errors:
            state["error"] = state["error"] or "; ".join(errors)
        if db is not None:
            try:
                while not records.empty():
                    kind, stamp, data = records.get_nowait()
                    db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (kind, stamp, data))
                db.commit()
                counts = write_metadata(directory, db, topics)
                state["counts"] = {str(k): v for k, v in counts.items()}
                db.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            except Exception as error:
                state["error"] += "; closing: " + str(error)
            finally:
                db.close()
        state.update(status="failed" if state["error"] or state["dropped"] else "completed",
                     updated_ns=time.time_ns())
        atomic_json(directory / "state.json", state)


def start(directory, config):
    directory = Path(directory)
    if directory.exists():
        state = read_state(directory)
        if alive(state) and state.get("status") == "recording":
            return state
        raise RuntimeError("session already exists; recover it instead of overwriting: " + str(directory))
    directory.mkdir(parents=True)
    atomic_json(directory / "config.json", config)
    with (directory / "capture.log").open("ab") as log:
        process = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve()),
                                    "capture", "--directory", str(directory)],
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                   start_new_session=True, close_fds=True)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        state = read_state(directory)
        if state.get("status") == "recording":
            return state
        if process.poll() is not None or state.get("status") == "failed":
            raise RuntimeError("G1 recorder failed: " + str(state) + " " + (directory / "capture.log").read_text()[-2000:])
        time.sleep(0.1)
    (directory / "stop.request").touch()
    raise RuntimeError("G1 recorder readiness timed out")


def stop(directory):
    directory = Path(directory)
    (directory / "stop.request").touch()
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        state = read_state(directory)
        if not alive(state):
            if state.get("status") not in {"completed", "failed"}:
                raise RuntimeError("G1 recorder exited without closing the bag: " + str(state))
            return state
        time.sleep(0.1)
    raise RuntimeError("G1 recorder stop timed out; originals retained")


def stream(directory, after):
    directory = Path(directory)
    with sqlite3.connect("file:{}?mode=ro".format(directory / "rosbag2/capture.db3"), uri=True) as db:
        while True:
            rows = db.execute("SELECT id,topic_id,timestamp,data FROM messages WHERE id>? AND topic_id IN (1,2) ORDER BY id LIMIT 128", (after,)).fetchall()
            for sequence, kind, stamp, payload in rows:
                sys.stdout.buffer.write(JOURNAL_HEADER.pack(JOURNAL_MAGIC, kind, sequence, stamp, len(payload)))
                sys.stdout.buffer.write(payload)
                after = sequence
            sys.stdout.buffer.flush()
            if not rows:
                state = read_state(directory)
                if state.get("status") in {"completed", "failed"}:
                    return
                if not alive(state):
                    raise RuntimeError("G1 recorder is no longer running")
                time.sleep(0.05)


def inventory(directory):
    directory = Path(directory)
    state = read_state(directory)
    if alive(state) or state.get("status") not in {"completed", "failed"}:
        raise RuntimeError("stop and close the G1 recorder before collecting")
    files = []
    for name in ("rosbag2/capture.db3", "rosbag2/metadata.yaml", "state.json", "config.json", "capture.log"):
        path = directory / name
        digest = hashlib.sha256()
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
        files.append({"name": name, "size": path.stat().st_size, "sha256": digest.hexdigest()})
    return {"state": state, "files": files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "start", "capture", "stop", "status", "stream", "inventory", "fetch"))
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--config")
    parser.add_argument("--after", type=int, default=0)
    parser.add_argument("--name")
    parser.add_argument("--offset", type=int, default=0)
    args = parser.parse_args()
    if not args.directory.is_absolute():
        parser.error("--directory must be absolute")
    if args.action == "prepare":
        prepare(json.loads(args.config).get("camera_enabled", False))
    elif args.action == "capture":
        capture(args.directory, json.loads((args.directory / "config.json").read_text()))
    elif args.action == "start":
        print(json.dumps(start(args.directory, json.loads(args.config))))
    elif args.action == "stop":
        print(json.dumps(stop(args.directory)))
    elif args.action == "status":
        print(json.dumps(read_state(args.directory)))
    elif args.action == "stream":
        stream(args.directory, args.after)
    elif args.action == "inventory":
        print(json.dumps(inventory(args.directory)))
    else:
        if args.name not in {"rosbag2/capture.db3", "rosbag2/metadata.yaml", "state.json", "config.json", "capture.log"} or args.offset < 0:
            parser.error("invalid fetch file or offset")
        if alive(read_state(args.directory)):
            raise RuntimeError("cannot fetch a mutable recording")
        with (args.directory / args.name).open("rb") as source:
            source.seek(args.offset)
            shutil.copyfileobj(source, sys.stdout.buffer, 1024 * 1024)


if __name__ == "__main__":
    main()
