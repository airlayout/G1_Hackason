# Jetson (Python 3.8) 上で動くヘルパー。server.py が ssh 経由で起動する。
# stdin に JSON 1行 {"op":"status"} / {"op":"set","id":N} を受け、stdout に JSON 1行を返す。
# __ALLOWED__ は server.py が許可 ID のリストに置換する。
import glob
import json
import os
import sys

# SDK は失敗時に stdout へ print する（デバッグモードでは "[ClientStub] send request error"）。
# JSON の応答が壊れないよう、応答専用に元の stdout を確保し、SDK の出力は stderr へ逃がす。
_reply = sys.stdout
sys.stdout = sys.stderr


def reply(obj):
    print(json.dumps(obj), file=_reply, flush=True)


import time

from cyclonedds.core import DDSException
from cyclonedds.domain import DomainParticipant
from cyclonedds.internal import InvalidSample
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_, SportModeState_, WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, IMUState_, LowCmd_, LowState_, MainBoardState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ALLOWED = set(__ALLOWED__)

ChannelFactoryInitialize(0, "eth0")
msc = MotionSwitcherClient()
msc.SetTimeout(2.0)
msc.Init()
loco = LocoClient()
loco.SetTimeout(2.0)
loco.Init()
audio = AudioClient()
audio.SetTimeout(3.0)
audio.Init()

# 読み取り専用のテレメトリ。コールバックは最新メッセージを保持するだけ（lowstate は高頻度）。
N_JOINTS = 29  # G1 29DoF。LowState の motor_state は 35 枠あるが先頭 29 が関節
# 購読はコールバックを使わない。lowstate などは毎秒数百通流れ、Python のコールバックで全部読むと
# プロセスが CPU 不足になり、音声などの RPC 応答を取りこぼす（3104 タイムアウト）ことを実測した。
# そこで DDS 側で 0.2 秒に 1 通へ間引き、履歴は最新 1 件だけにして、状態取得のたびに take する。
_latest = {}   # トピック名 -> (取得時刻, メッセージ)
_readers = {}
_participant = None  # ChannelFactoryInitialize の後に作る（同じ Domain 設定を共有する）
_READER_QOS = Qos(Policy.History.KeepLast(1), Policy.TimeBasedFilter(duration(milliseconds=200)))


def _watch(topic, msg_type):
    global _participant
    if _participant is None:
        _participant = DomainParticipant(0)
    _readers[topic] = DataReader(_participant, Topic(_participant, topic, msg_type), _READER_QOS)


def _refresh():
    """各トピックの最新メッセージを取り込む。新着が無ければ前回の値と取得時刻を保つ。"""
    now = time.time()
    for topic, reader in _readers.items():
        try:
            samples = reader.take(N=1)
        except DDSException:
            continue
        for sample in samples:
            if not isinstance(sample, InvalidSample):
                _latest[topic] = (now, sample)


BINARY_TOPICS = [
    ("rt/lf/bmsstate", BmsState_), ("rt/lowstate", LowState_), ("rt/odommodestate", SportModeState_),
    ("rt/lowcmd", LowCmd_), ("rt/secondary_imu", IMUState_), ("rt/lf/mainboardstate", MainBoardState_),
    ("rt/lf/emergency_stop", Error_), ("rt/wirelesscontroller", WirelessController_),
]
# JSON 文字列が流れてくるトピック（サービス状態・通信状態など）
STRING_TOPICS = ["rt/rtc/state", "rt/public_network_status", "rt/arm/action/state", "rt/gpt_state",
                 "rt/servicestate", "rt/servicestateactivate", "rt/lf/battery_alarm", "rt/rtc_status",
                 "rt/multiplestate", "rt/selftest"]
for _t, _ty in BINARY_TOPICS:
    _watch(_t, _ty)
for _t in STRING_TOPICS:
    _watch(_t, String_)
reply({"ready": True})


def _get(topic):
    got = _latest.get(topic)
    return (got[1], round(time.time() - got[0], 2)) if got else (None, None)


def _plain(msg):
    """単純な IDL メッセージを JSON にできる dict にする（リストは先頭 12 要素まで）。"""
    out = {}
    for a in dir(msg):
        if a.startswith("_") or a in ("serialize", "deserialize", "sample_info"):
            continue
        v = getattr(msg, a)
        if callable(v):
            continue
        out[a] = list(v)[:12] if hasattr(v, "__len__") and not isinstance(v, str) else v
    return out


def _system():
    """Jetson 自身の状態（/proc と /sys から読むだけ）。"""
    out = {}
    try:
        out["load"] = [float(x) for x in open("/proc/loadavg").read().split()[:3]]
        mem = dict(l.split(":") for l in open("/proc/meminfo").read().splitlines())
        kb = lambda k: int(mem[k].split()[0])
        out["mem_total_mb"], out["mem_avail_mb"] = kb("MemTotal") // 1024, kb("MemAvailable") // 1024
        out["uptime_s"] = float(open("/proc/uptime").read().split()[0])
        out["cpus"] = os.cpu_count()
        temps = {}
        for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
            temps[open(z + "/type").read().strip()] = int(open(z + "/temp").read()) / 1000.0
        out["temps"] = temps
        st = os.statvfs("/")
        out["disk_free_gb"] = round(st.f_bavail * st.f_frsize / 1e9, 1)
    except Exception as exc:
        out["error"] = "%s: %s" % (type(exc).__name__, exc)
    return out


def telemetry():
    _refresh()
    now = time.time()
    # 各トピックの最終受信からの経過秒（None は一度も受信していない）
    out = {"ages": {t: round(now - _latest[t][0], 2) if t in _latest else None for t in _readers}}
    bms, age = _get("rt/lf/bmsstate")
    out["battery"] = None if bms is None else {
        "soc": bms.soc, "current": bms.current, "voltage": list(bms.bmsvoltage),
        "temperature": list(bms.temperature), "age_s": age}
    low, age = _get("rt/lowstate")
    if low is None:
        out["imu"], out["joints"] = None, None
    else:
        imu = low.imu_state
        out["imu"] = {"rpy": list(imu.rpy), "gyro": list(imu.gyroscope), "accel": list(imu.accelerometer),
                      "temperature": imu.temperature, "age_s": age}
        out["joints"] = {"age_s": age, "mode_machine": low.mode_machine, "items": [
            {"q": m.q, "dq": m.dq, "tau": m.tau_est, "temperature": list(m.temperature), "lost": m.motorstate}
            for m in low.motor_state[:N_JOINTS]]}
    odom, age = _get("rt/odommodestate")
    out["odom"] = None if odom is None else {
        "position": list(odom.position), "velocity": list(odom.velocity), "yaw_speed": odom.yaw_speed,
        "mode": odom.mode, "gait_type": odom.gait_type, "body_height": odom.body_height,
        "error_code": odom.error_code, "age_s": age}
    cmd, age = _get("rt/lowcmd")
    out["lowcmd"] = None if cmd is None else {"age_s": age, "mode_machine": cmd.mode_machine, "items": [
        {"q": m.q, "dq": m.dq, "tau": m.tau, "kp": m.kp, "kd": m.kd, "mode": m.mode}
        for m in cmd.motor_cmd[:N_JOINTS]]}
    imu2, age = _get("rt/secondary_imu")
    out["imu2"] = None if imu2 is None else {"rpy": list(imu2.rpy), "gyro": list(imu2.gyroscope),
                                             "accel": list(imu2.accelerometer), "age_s": age}
    for key, topic in (("mainboard", "rt/lf/mainboardstate"), ("estop", "rt/lf/emergency_stop"),
                       ("remote", "rt/wirelesscontroller")):
        msg, age = _get(topic)
        out[key] = None if msg is None else {**_plain(msg), "age_s": age}
    strings = {}
    for topic in STRING_TOPICS:
        msg, age = _get(topic)
        if msg is not None:
            try:
                val = json.loads(msg.data)
            except ValueError:
                val = msg.data
            strings[topic] = {"value": val, "age_s": age}
    out["strings"] = strings
    out["system"] = _system()
    return out


def status():
    code, mode = msc.CheckMode()
    name = (mode or {}).get("name", "") if code == 0 else None
    fsm_code, fsm_id = loco.GetFsmId()
    return {"ok": True, "checkmode_code": code, "service": name,
            "fsm_code": fsm_code, "fsm_id": fsm_id if fsm_code == 0 else None,
            "telemetry": telemetry()}


def audio_op(req):
    """音声・LED。モーションサービスとは独立（デバッグモードでも動く）。値の範囲は server.py で検証済み。"""
    op = req["op"]
    if op == "audio_get":
        code, data = audio.GetVolume()
        return {"ok": True, "code": code, "volume": (data or {}).get("volume")}
    if op == "audio_volume":
        set_code = audio.SetVolume(int(req["volume"]))
        read_code, data = audio.GetVolume()  # 読み戻して反映を確かめる
        return {"ok": True, "set_code": set_code, "read_code": read_code,
                "volume_after": (data or {}).get("volume")}
    if op == "audio_led":
        return {"ok": True, "code": audio.LedControl(int(req["r"]), int(req["g"]), int(req["b"]))}
    if op == "audio_tts":
        return {"ok": True, "code": audio.TtsMaker(str(req["text"]), int(req["speaker_id"]))}
    return {"ok": False, "error": "unknown audio op"}


for line in sys.stdin:
    try:
        req = json.loads(line)
        if req["op"] == "status":
            res = status()
        elif req["op"] == "set":
            fsm_id = int(req["id"])
            if fsm_id not in ALLOWED:
                res = {"ok": False, "error": "許可されていない FSM ID: %d" % fsm_id}
            else:
                res = {"ok": True, "set_code": loco.SetFsmId(fsm_id)}
        elif req["op"].startswith("audio_"):
            res = audio_op(req)
        else:
            res = {"ok": False, "error": "unknown op"}
    except Exception as exc:  # 1リクエストの失敗でヘルパー全体を落とさない
        res = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}
    reply(res)
