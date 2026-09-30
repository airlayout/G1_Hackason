"""G1 のライブ値を、このPCへ JSON Lines で保存する（読み取り専用）。

開発用 PC 側から起動する: python3 Console/live_logger.py [--host g1-ts] [--out DIR] [--period 0.5]
Jetson 上で DDS を購読して 1 行 1 メッセージで標準出力へ流し、ssh 越しにこの PC のファイルへ書く。
電源が切れて ssh が切れるまでの分が残る（行ごとに flush するので途中で切れても読める）。
lidar・カメラ・地図・点群は対象外。lowstate などの高頻度トピックは --period 秒に 1 通へ間引く。
イベント型（String_ 系）は間引かず、来た分をすべて記録する。
"""
import argparse
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path

REMOTE = r'''
import json, os, sys, time
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Policy, Qos
from cyclonedds.util import duration
from cyclonedds.internal import InvalidSample
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_, SportModeState_, WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, IMUState_, LowCmd_, LowState_, MainBoardState_, HandState_
PERIOD_MS = int(float(sys.argv[1]) * 1000)
TYPES = {"String_": String_, "Error_": Error_, "SportModeState_": SportModeState_, "WirelessController_": WirelessController_,
         "BmsState_": BmsState_, "IMUState_": IMUState_, "LowCmd_": LowCmd_, "LowState_": LowState_,
         "MainBoardState_": MainBoardState_, "HandState_": HandState_}
def plain(o, depth=0):
    if isinstance(o, (int, float, str, bool)) or o is None: return o
    if isinstance(o, (bytes, bytearray)): return "<bytes %d>" % len(o)
    if hasattr(o, "__len__"):
        l = list(o); return [plain(v, depth + 1) for v in l[:40]]
    if depth > 4: return str(o)
    return {a: plain(getattr(o, a), depth + 1) for a in dir(o) if not a.startswith("_") and not callable(getattr(o, a))}
dp = DomainParticipant(0)
pub = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
seen = {}
for _ in range(5):
    time.sleep(1)
    for x in pub.take(N=500):
        seen[x.topic_name] = x.type_name
readers = {}
for t, ty in seen.items():
    short = ty.split("::")[-1]
    if t.startswith("rt/api/") or short not in TYPES: continue
    if short == "String_":
        q = Qos(Policy.History.KeepLast(50))
    else:
        q = Qos(Policy.History.KeepLast(1), Policy.TimeBasedFilter(duration(milliseconds=PERIOD_MS)))
    readers[t] = DataReader(dp, Topic(dp, t, TYPES[short]), q)
print(json.dumps({"t": time.time(), "meta": {"topics": {t: seen[t] for t in readers}, "period_ms": PERIOD_MS}}), flush=True)
last_beat = 0
while True:
    for t, r in readers.items():
        for s in r.take(N=50):
            if not isinstance(s, InvalidSample):
                print(json.dumps({"t": time.time(), "topic": t, "v": plain(s)}, ensure_ascii=False, default=str), flush=True)
    now = time.time()
    if now - last_beat > 5:
        print(json.dumps({"t": now, "beat": True}), flush=True)
        last_beat = now
    time.sleep(0.05)
'''


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="g1-ts")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "docs" / "g1_logs"))
    ap.add_argument("--period", type=float, default=0.5, help="高頻度トピックを間引く間隔 [秒]")
    args = ap.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / time.strftime("live_%Y%m%d_%H%M%S.jsonl")
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2", args.host,
           "cd ~/unitree_sdk2_python && python3 -u -c %s %s" % (shlex.quote(REMOTE), args.period)]
    print("[logger] 保存先: %s" % path, flush=True)
    lines = 0
    with open(path, "w", encoding="utf-8") as f, subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True) as p:
        for line in p.stdout:
            if line.startswith("{"):
                f.write(line)
                f.flush()
                lines += 1
        code = p.wait()
    print("[logger] 終了 (ssh exit=%s, %d 行) %s" % (code, lines, path), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
