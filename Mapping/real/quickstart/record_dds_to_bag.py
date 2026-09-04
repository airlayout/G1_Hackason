#!/usr/bin/env python3
"""PC2上でUnitreeのDDS点群をrosbag2互換のdb3へ記録する。

`Mapping/real`は`ros2 bag record`で記録する設計だが、**PC2のROS 2 foxyからは
Unitreeのトピックが見えない**（Navigation/README.md「PC2のROS 2 foxyからは
`ros2 topic list`でUnitreeのトピックが見えない」）。一方`unitree_sdk2py`の
ChannelSubscriberは型を明示すれば購読できる（実測で確認済み）。

そこで受信したメッセージを**CDRのまま**rosbag2のdb3へ落とす。
`cyclonedds`のIDL型は`serialize()`がCDR（先頭`00 01 00 00`）を返し、これは
`g1_mapping/rebuild.py`の`_CdrReader`がそのまま解釈できる形式である。

結果としてMac側は無改造で使える:

    ./mapctl rebuild <session_id>
    ./mapctl validate <session_id>

Python 3.8（PC2の既定）で動くように書いてある。
"""
import argparse
import json
import os
import queue
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
# `mapping_ctl.py` は同じディレクトリに置いてある前提（PC2では ~/mapping_tools/）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mapping_ctl  # noqa: E402  （SDKの読み込みはこの時点では起きない）

MAPPING_DEFAULT_MAP_PATH = mapping_ctl.DEFAULT_MAP_PATH

# ROSのトピック名（先頭"/"）とDDSのトピック名（先頭"rt/"）の対応。
# db3にはrosbag2と同じくROS名で入れる。そうしないとmapctlの既定トピック名と噛み合わない。
KNOWN_TOPICS = {
    "/utlidar/cloud_livox_mid360": "sensor_msgs/msg/PointCloud2",
    # IMUは記録しておかないと後からFAST-LIO2に掛け直せない。取り返しがつかないので既定に入れる
    "/utlidar/imu_livox_mid360": "sensor_msgs/msg/Imu",
    "/unitree/slam_mapping/points": "sensor_msgs/msg/PointCloud2",
    "/unitree/slam_mapping/odom": "nav_msgs/msg/Odometry",
    "/unitree/slam_relocation/points": "sensor_msgs/msg/PointCloud2",
}


def resolve_idl_type(ros_type):
    """`sensor_msgs/msg/PointCloud2` から unitree_sdk2py の IDL クラスを引く。

    型ごとにモジュールが違う（sensor_msgs / nav_msgs）ので名前から解決する。
    SDKに無い型はここで分かる。
    """
    import importlib
    package, _, name = ros_type.split("/")
    module = importlib.import_module("unitree_sdk2py.idl.{}.msg.dds_".format(package))
    return getattr(module, name + "_")


ROSBAG2_SCHEMA = """
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE topics(
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    type TEXT NOT NULL,
    serialization_format TEXT NOT NULL,
    offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(
    id INTEGER PRIMARY KEY,
    topic_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages(timestamp ASC);
"""


def git_revision(project_dir):
    try:
        out = subprocess.run(
            ["git", "-C", project_dir, "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() if out.returncode == 0 else "unknown"
    except Exception:
        return "unknown"


def create_session(runs_dir, name, backend, topics, g1_host, iface, domain_id):
    """`g1_mapping/session.py`が読める形のセッションを作る。"""
    session_id = "{}_{}".format(datetime.now().strftime("%Y%m%dT%H%M%S"), name)
    directory = os.path.join(runs_dir, session_id)
    for child in ("raw", "map", "trajectory", "logs", "report"):
        os.makedirs(os.path.join(directory, child), exist_ok=True)
    now = datetime.now(timezone.utc).isoformat()
    manifest = {
        "schema_version": 1,
        "session_id": session_id,
        "name": name,
        "backend": backend,
        "created_at": now,
        "git_revision": git_revision(os.path.dirname(os.path.abspath(__file__))),
        "robot": {"host": g1_host, "network_interface": iface,
                  "ros_domain_id": domain_id},
        "topics": {"raw_points": "/utlidar/cloud_livox_mid360",
                   "raw_imu": "/utlidar/imu_livox_mid360",
                   "onboard_points": "/unitree/slam_mapping/points",
                   "onboard_odom": "/unitree/slam_mapping/odom"},
        "remote_map_path": "",
        "diagnostic": {"note": "record_dds_to_bag.py（PC2直接DDS購読）で記録"},
        "recorded_topics": topics,
    }
    state = {"session_id": session_id, "backend": backend, "status": "running",
             "updated_at": now, "message": "PC2で直接DDS購読して記録中"}
    with open(os.path.join(directory, "manifest.json"), "w") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    with open(os.path.join(directory, "state.json"), "w") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)
    return session_id, directory


def write_metadata_yaml(bag_dir, bag_name, topics, known_types, counts,
                        start_ns, duration_ns):
    """rosbag2のmetadata.yamlを書く。

    `mapctl validate`が存在を見るだけでなく、`ros2 bag info`でも読める形式にする。
    PyYAMLに依存させたくないので素直に組み立てる。
    """
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 4",
        "  storage_identifier: sqlite3",
        "  relative_file_paths:",
        "    - {}".format(bag_name),
        "  duration:",
        "    nanoseconds: {}".format(int(duration_ns)),
        "  starting_time:",
        "    nanoseconds_since_epoch: {}".format(int(start_ns)),
        "  message_count: {}".format(sum(counts.values())),
        "  topics_with_message_count:",
    ]
    for topic in topics:
        lines += [
            "    - topic_metadata:",
            "        name: {}".format(topic),
            "        type: {}".format(known_types[topic]),
            "        serialization_format: cdr",
            '        offered_qos_profiles: ""',
            "      message_count: {}".format(counts[topic]),
        ]
    lines += ['  compression_format: ""', '  compression_mode: ""', ""]
    with open(os.path.join(bag_dir, "metadata.yaml"), "w") as f:
        f.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", action="append", dest="topics",
                        help="ROS名で指定。複数可。省略時は生LiDAR+IMU"
                             "（--with-mapping なら地図点群+odom）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="秒。0なら Ctrl-C まで記録し続ける")
    parser.add_argument("--name", default="lidar")
    parser.add_argument("--backend", default="raw", choices=["raw", "onboard"],
                        help="mapctl rebuild の既定トピック選択に効く")
    parser.add_argument("--runs-dir", default=os.path.expanduser("~/mapping_runs"))
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--g1-host", default="192.168.123.164")
    parser.add_argument("--with-mapping", action="store_true",
                        help="記録の前後で 1801(建図開始)/1802(建図終了・保存) を呼ぶ。"
                             "既定トピックが /unitree/slam_mapping/points になる")
    parser.add_argument("--map-path", default=MAPPING_DEFAULT_MAP_PATH,
                        help="--with-mapping のとき 1802 に渡す保存先。"
                             "**PC1上のパス**（既定 {}）".format(MAPPING_DEFAULT_MAP_PATH))
    args = parser.parse_args()

    # 建図を通す場合、地図座標系の点群（内蔵SLAMの出力）に加えて、
    # **生LiDARとIMUも必ず並行記録する**。
    #
    # 2026-09-03のUiS_room_v1で、ここが if/else の排他になっており
    # `--with-mapping` では生データが1バイトも残らなかった。その結果
    # 開始と終了で11.2mずれた地図をFAST-LIO2で検証し直すことができず、
    # 原因の切り分け自体が不可能になった。記録しなかったデータは取り返せない。
    #
    # 内蔵SLAMの点群は既に地図座標系へ変換済み＝姿勢が座標に焼き込まれており、
    # 後から姿勢を推定し直せない。生LiDAR+IMUだけが再処理の余地を残す。
    #
    # 帯域は生LiDARが約4.4MB/s（地図点群は0.4MB/s）。12分で約3.5GB増えるが、
    # PC2の空きは1.8TBあるので問題にならない。容量を切り詰めたい場合のみ
    # `--topic`で明示指定して外すこと。
    default_topics = (["/unitree/slam_mapping/points", "/unitree/slam_mapping/odom",
                       "/utlidar/cloud_livox_mid360", "/utlidar/imu_livox_mid360"]
                      if args.with_mapping
                      else ["/utlidar/cloud_livox_mid360", "/utlidar/imu_livox_mid360"])
    topics = list(args.topics or default_topics)
    for topic in topics:
        if topic not in KNOWN_TOPICS:
            parser.error("未知のトピックです: {}（既知: {}）".format(
                topic, ", ".join(sorted(KNOWN_TOPICS))))

    backend = args.backend
    if args.with_mapping and backend == "raw" and "--backend" not in sys.argv:
        # mapctl rebuild が既定で ONBOARD_POINTS_TOPIC を選ぶようにする
        backend = "onboard"

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

    # 型はトピックごとに解決する。SDKに無ければここで落ちる（黙って無視しない）。
    # 明示指定していないトピックだけは、解決できなければ外して続行する。
    idl_types = {}
    for topic in list(topics):
        try:
            idl_types[topic] = resolve_idl_type(KNOWN_TOPICS[topic])
        except (ImportError, AttributeError) as error:
            if args.topics and topic in args.topics:
                print("[ERROR] {} の型 {} がSDKにありません: {}".format(
                    topic, KNOWN_TOPICS[topic], error), file=sys.stderr)
                return 1
            print("[WARN] {} の型 {} がSDKに無いので記録対象から外します".format(
                topic, KNOWN_TOPICS[topic]), file=sys.stderr)
            topics.remove(topic)
    if not topics:
        print("[ERROR] 記録できるトピックがありません", file=sys.stderr)
        return 1

    session_id, directory = create_session(
        args.runs_dir, args.name, backend, topics,
        args.g1_host, args.iface, args.domain_id)
    bag_dir = os.path.join(directory, "raw", "rosbag2")
    os.makedirs(bag_dir, exist_ok=True)
    bag_path = os.path.join(bag_dir, "{}_0.db3".format(session_id))

    connection = sqlite3.connect(bag_path)
    connection.executescript(ROSBAG2_SCHEMA)
    connection.execute("INSERT INTO schema VALUES (4, 'humble')")
    topic_ids = {}
    for index, topic in enumerate(topics, start=1):
        connection.execute(
            "INSERT INTO topics VALUES (?,?,?,?,?)",
            (index, topic, KNOWN_TOPICS[topic], "cdr", ""))
        topic_ids[topic] = index
    connection.commit()

    # 受信はDDSのコールバックスレッド、書き込みは本スレッド。sqlite3を跨がせない。
    records = queue.Queue(maxsize=2000)
    stopping = threading.Event()
    counts = dict((topic, 0) for topic in topics)
    dropped = [0]

    def make_handler(topic_id):
        def handle(message):
            try:
                records.put_nowait((topic_id, time.time_ns(), message.serialize()))
            except queue.Full:
                dropped[0] += 1
        return handle

    ChannelFactoryInitialize(args.domain_id, args.iface)
    subscribers = []
    for topic in topics:
        dds_name = "rt" + topic
        subscriber = ChannelSubscriber(dds_name, idl_types[topic])
        subscriber.Init(make_handler(topic_ids[topic]), 20)
        subscribers.append(subscriber)
    def request_stop(signum, frame):
        stopping.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # SSHが切れるとsshdがSIGHUPを送る。既定の動作はプロセス即死で、
    # finally節が走らない＝1802が飛ばない。WiFi運用では現実的に起こるので捕まえる。
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, request_stop)

    id_to_topic = dict((v, k) for k, v in topic_ids.items())
    print("session : {}".format(session_id))
    print("bag     : {}".format(bag_path))
    print("topics  : {}".format(", ".join(topics)))
    print("backend : {}".format(backend))
    print("建図    : {}".format(
        "1801/1802 を通す（保存先 {}）".format(args.map_path)
        if args.with_mapping else "使わない（生LiDARのみ）"))
    print("停止    : {}".format(
        "{:.0f}秒後".format(args.duration) if args.duration > 0 else "Ctrl-C"))
    print("-" * 60, flush=True)


    # 建図の開始。購読を張ってから投げることで、最初のスキャンを取りこぼさない。
    # チャネル初期化は既に済ませてあるので init_channel=False を渡す
    # （SDKのチャネル初期化はプロセスに一度しか通せない）。
    slam = None
    if args.with_mapping:
        slam = mapping_ctl.SlamOperateClient(
            interface=args.iface, domain_id=args.domain_id,
            timeout_s=10.0, init_channel=False)
        print("[1801] 建図を開始します ...", flush=True)
        rpc_code, response, raw = slam.call(
            mapping_ctl.API_START_MAPPING, mapping_ctl.start_mapping_request())
        print("[1801] rpc_code={} response={}".format(rpc_code, raw), flush=True)
        if rpc_code != 0 or response is None or not response["succeed"]:
            print("[ERROR] 建図を開始できませんでした。記録を中止します", file=sys.stderr)
            connection.close()
            # 中止したセッションをrunningのまま残さない
            state_path = os.path.join(directory, "state.json")
            with open(state_path) as f:
                state = json.load(f)
            state.update({"status": "failed",
                          "updated_at": datetime.now(timezone.utc).isoformat(),
                          "message": "1801(建図開始)に失敗したため記録しなかった: {}".format(raw)})
            with open(state_path, "w") as f:
                json.dump(state, f, ensure_ascii=False, indent=2)
            return 1
        print("[1801] 開始しました", flush=True)

    started = time.time()
    started_ns = time.time_ns()
    written = 0
    last_report = started
    stop_result = {"called": False, "ok": False, "raw": None}
    try:
        while not stopping.is_set():
            if args.duration > 0 and time.time() - started >= args.duration:
                break
            try:
                topic_id, stamp, payload = records.get(timeout=0.3)
            except queue.Empty:
                continue
            connection.execute(
                "INSERT INTO messages(topic_id, timestamp, data) VALUES (?,?,?)",
                (topic_id, stamp, sqlite3.Binary(payload)))
            counts[id_to_topic[topic_id]] += 1
            written += 1
            if written % 20 == 0:
                connection.commit()
            now = time.time()
            if now - last_report >= 2.0:
                elapsed = now - started
                size_mb = os.path.getsize(bag_path) / 1e6
                print("  {:5.1f}s  {}  {:.1f}MB{}".format(
                    elapsed,
                    "  ".join("{}={}".format(t.split('/')[-1], c)
                              for t, c in counts.items()),
                    size_mb,
                    "  drop={}".format(dropped[0]) if dropped[0] else ""), flush=True)
                last_report = now
    finally:
        # 取りこぼしを吐き切ってから閉じる
        while True:
            try:
                topic_id, stamp, payload = records.get_nowait()
            except queue.Empty:
                break
            connection.execute(
                "INSERT INTO messages(topic_id, timestamp, data) VALUES (?,?,?)",
                (topic_id, stamp, sqlite3.Binary(payload)))
            counts[id_to_topic[topic_id]] += 1
        connection.commit()
        connection.close()

        # 1802は何があっても投げる。2026-08-26の失敗は、停止時に通信が切れて
        # kEndMappingがG1へ届かず、PCDが書かれずセッションがfailedになったこと。
        # 例外やCtrl-Cで抜けた場合でもここを通す。
        if slam is not None:
            stop_result["called"] = True
            print("\n[1802] 建図を終了し保存します: {}".format(args.map_path), flush=True)
            print("       ※ これはPC1(192.168.123.161)上のパスです", flush=True)
            try:
                rpc_code, response, raw = slam.call(
                    mapping_ctl.API_END_MAPPING,
                    mapping_ctl.end_mapping_request(args.map_path))
                stop_result["raw"] = raw
                stop_result["ok"] = (
                    rpc_code == 0 and response is not None and response["succeed"])
                print("[1802] rpc_code={} response={}".format(rpc_code, raw), flush=True)
            except Exception as error:  # noqa: BLE001  通信断で落ちても記録は残す
                stop_result["raw"] = "{}: {}".format(type(error).__name__, error)
                print("[1802] 例外: {}".format(stop_result["raw"]), file=sys.stderr)
            if stop_result["ok"]:
                print("[1802] G1側へ保存しました", flush=True)
            else:
                print("[WARN] 1802が成功しませんでした。ただし記録は残っているので、"
                      "Macで ./mapctl rebuild すれば地図は作り直せます", file=sys.stderr)

    elapsed = time.time() - started
    write_metadata_yaml(bag_dir, os.path.basename(bag_path), topics,
                        KNOWN_TOPICS, counts, started_ns,
                        int(elapsed * 1e9))
    state_path = os.path.join(directory, "state.json")
    with open(state_path) as f:
        state = json.load(f)
    message = "{:.1f}秒記録した".format(elapsed)
    if stop_result["called"]:
        message += "。1802は{}".format("成功" if stop_result["ok"] else "失敗")
        state["end_mapping"] = {"map_path": args.map_path,
                                "succeeded": stop_result["ok"],
                                "response": stop_result["raw"]}
    state.update({"status": "completed",
                  "updated_at": datetime.now(timezone.utc).isoformat(),
                  "message": message})
    with open(state_path, "w") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

    print("-" * 60)
    for topic, count in counts.items():
        rate = count / elapsed if elapsed > 0 else 0.0
        print("  {:34} {:6}件  {:5.2f}Hz".format(topic, count, rate))
    print("  {:.1f}MB / {:.1f}秒{}".format(
        os.path.getsize(bag_path) / 1e6, elapsed,
        "  取りこぼし {}件".format(dropped[0]) if dropped[0] else ""))
    print("\nMacへ回収してから:")
    print("  ./mapctl rebuild {}".format(session_id))
    print("  ./mapctl validate {}".format(session_id))


if __name__ == "__main__":
    # main()の戻り値を捨てると、1801に失敗しても終了コードが0になり、
    # ssh越しやスクリプトから失敗を検知できない。
    sys.exit(main())
