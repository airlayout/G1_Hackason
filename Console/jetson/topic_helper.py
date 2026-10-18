# Jetson (Python 3.8) 上で動くトピック通信ヘルパー。g1console/topic_backend.py の HelperTopicBackend が子プロセスとして起動する。
# 許す操作は sub / unsub / poll / pub だけ。pub は __PUBLISH__（送信許可リスト）と __DENY__ をここでも検査する（ハブと二重）。
# stdin に JSON 1 行を受け、stdout に JSON 1 行を返す。運動・RPC クライアントは持たない。
import fnmatch
import json
import sys
import time

_reply = sys.stdout
sys.stdout = sys.stderr  # SDK の print で応答が壊れないよう、応答専用に元の stdout を確保する

from cyclonedds.core import DDSException
from cyclonedds.domain import DomainParticipant
from cyclonedds.internal import InvalidSample
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_, SportModeState_, WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, IMUState_

PUBLISH = __PUBLISH__  # {トピック: 最大バイト数}
DENY = __DENY__
TYPES = {"String_": String_, "Error_": Error_, "WirelessController_": WirelessController_, "BmsState_": BmsState_,
         "IMUState_": IMUState_, "SportModeState_": SportModeState_}
MAX_ITEMS, MAX_STR = 16, 512  # 返す配列・文字列の上限（大きなメッセージでハブを詰まらせない）
QOS = Qos(Policy.History.KeepLast(1), Policy.TimeBasedFilter(duration(milliseconds=200)))

ChannelFactoryInitialize(0, "eth0")
participant = DomainParticipant(0)
readers = {}   # トピック -> [型名, reader, 最新(取得時刻, msg), 件数]
writers = {}


def reply(obj):
    print(json.dumps(obj), file=_reply, flush=True)


def plain(value, depth=0):
    if isinstance(value, (bool, int, float)) or value is None:
        return value
    if isinstance(value, str):
        return value[:MAX_STR]
    if depth > 3:
        return None
    if hasattr(value, "__len__"):
        return [plain(v, depth + 1) for v in list(value)[:MAX_ITEMS]]
    out = {}
    for a in dir(value):
        if a.startswith("_") or a in ("serialize", "deserialize", "sample_info"):
            continue
        v = getattr(value, a)
        if not callable(v):
            out[a] = plain(v, depth + 1)
    return out


def denied(topic):
    return any(fnmatch.fnmatchcase(topic.lower(), pat) for pat in DENY)


def subscribe(topic, type_name):
    if type_name not in TYPES:
        raise ValueError("type")
    if topic in readers:
        return
    reader = DataReader(participant, Topic(participant, topic, TYPES[type_name]), QOS)
    readers[topic] = [type_name, reader, None, 0]


def poll():
    now = time.time()
    items = []
    for topic, slot in readers.items():
        try:
            samples = slot[1].take(N=1)
        except DDSException:
            samples = []
        for sample in samples:
            if not isinstance(sample, InvalidSample):
                slot[2], slot[3] = (now, sample), slot[3] + 1
        got = slot[2]
        items.append({"topic": topic, "type": slot[0], "count": slot[3],
                      "age_s": None if got is None else round(now - got[0], 2),
                      "message": None if got is None else plain(got[1])})
    return items


def publish(topic, data):
    if denied(topic) or topic not in PUBLISH:
        raise ValueError("not allowed")
    if not isinstance(data, str) or len(data.encode("utf-8")) > PUBLISH[topic]:
        raise ValueError("data")
    if topic not in writers:
        writers[topic] = DataWriter(participant, Topic(participant, topic, String_))
    writers[topic].write(String_(data=data))


def handle(req):
    op = req.get("op")
    if op == "sub":
        subscribe(req["topic"], req["type"])
        return {"ok": True}
    if op == "unsub":
        slot = readers.pop(req["topic"], None)
        if slot is not None:
            del slot[1]
        return {"ok": True}
    if op == "poll":
        return {"ok": True, "items": poll()}
    if op == "pub":
        publish(req["topic"], req["data"])
        return {"ok": True}
    raise ValueError("op")


reply({"ready": True})
for line in sys.stdin:
    try:
        reply(handle(json.loads(line)))
    except Exception as exc:  # 1 件の不正で止まらない。理由だけ返す
        reply({"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)})
