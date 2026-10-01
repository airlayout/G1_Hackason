#!/bin/bash
# G1(Jetson)から読み取り専用で資料を丸ごと保存する。電源が切れる前の退避用。
OUT="$(cd "$(dirname "$0")/.." && pwd)/docs/g1_snapshot"
mkdir -p "$OUT"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 g1-ts"

# 1) 最優先: SDK 一式（ソース・IDL・examples）。生成物は除く
$SSH 'cd ~ && tar czf - --exclude=__pycache__ --exclude="*.pyc" --exclude=.git unitree_sdk2_python' > "$OUT/unitree_sdk2_python.tgz" 2>"$OUT/tar_sdk.err"
echo "sdk tar: $(wc -c < "$OUT/unitree_sdk2_python.tgz") bytes"

# 2) ホーム直下と Jetson の環境情報
$SSH 'ls -la ~; echo ==; find ~ -maxdepth 2 -not -path "*/.cache*" -not -path "*/.local/*" -not -path "*/unitree_sdk2_python/*" -not -path "*/.git/*" | head -300' > "$OUT/home_listing.txt" 2>&1
$SSH 'uname -a; cat /etc/os-release; cat /etc/nv_tegra_release 2>&1; echo ==IP; ip -br addr; echo ==PY; python3 --version; pip3 list 2>/dev/null; echo ==DISK; df -h; echo ==MEM; free -m; echo ==PS; ps aux --sort=-%cpu | head -60; echo ==SYSTEMD; systemctl list-units --type=service --state=running --no-pager 2>&1 | head -80; echo ==UNITREE; ls -la /unitree /unitree/* 2>&1 | head -150; ls /opt 2>&1' > "$OUT/jetson_env.txt" 2>&1
$SSH 'aplay -l 2>&1; echo ==; cat /proc/asound/cards; echo ==; ls -la /dev/snd/by-path 2>&1; echo ==USB; lsusb 2>&1; echo ==I2C; ls /dev/i2c* 2>&1; echo ==VIDEO; ls /dev/video* 2>&1; echo ==CAN; ip -d link show 2>&1 | head -60' > "$OUT/jetson_hw.txt" 2>&1
$SSH 'ss -tulpn 2>&1 | head -80; echo ==; cat /etc/hosts; echo ==; cat ~/.bash_history 2>/dev/null | tail -100' > "$OUT/jetson_net.txt" 2>&1
echo "env done"

# 3) 各 DDS トピックの実サンプルと、サービスの応答（読み取りのみ）
# トピック一覧（dds_topics_discovered.txt）は tools/discover.py の出力を $OUT に置いておく
$SSH 'cd ~/unitree_sdk2_python && python3 -u -' > "$OUT/dds_samples.json" 2>"$OUT/dds_samples.err" <<'PYEOF'
import json, os, time
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
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
TYPES = {"String_": String_, "Error_": Error_, "SportModeState_": SportModeState_, "WirelessController_": WirelessController_,
         "BmsState_": BmsState_, "IMUState_": IMUState_, "LowCmd_": LowCmd_, "LowState_": LowState_,
         "MainBoardState_": MainBoardState_, "HandState_": HandState_}
dp = DomainParticipant(0)
pub = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
seen = {}
for _ in range(6):
    time.sleep(1)
    for x in pub.take(N=500):
        seen[x.topic_name] = x.type_name
def plain(o, depth=0):
    if isinstance(o, (int, float, str, bool)) or o is None: return o
    if isinstance(o, (bytes, bytearray)): return "<bytes %d>" % len(o)
    if hasattr(o, "__len__"):
        l = list(o); return [plain(v, depth+1) for v in l[:40]] + (["...(%d items)" % len(l)] if len(l) > 40 else [])
    if depth > 4: return str(o)
    return {a: plain(getattr(o, a), depth+1) for a in dir(o) if not a.startswith("_") and not callable(getattr(o, a))}
readers = {}
for t, ty in seen.items():
    short = ty.split("::")[-1]
    if t.startswith("rt/api/") or short not in TYPES: continue
    q = Qos(Policy.History.KeepLast(1), Policy.TimeBasedFilter(duration(milliseconds=500)))
    readers[t] = (short, DataReader(dp, Topic(dp, t, TYPES[short]), q))
out = {"topics_seen": seen, "samples": {}}
end = time.time() + 12
while time.time() < end:
    for t, (short, r) in readers.items():
        if t in out["samples"]: continue
        for s in r.take(N=1):
            if not isinstance(s, InvalidSample): out["samples"][t] = {"type": short, "value": plain(s)}
    time.sleep(0.2)
out["no_data_in_12s"] = sorted(set(readers) - set(out["samples"]))
print(json.dumps(out, ensure_ascii=False, indent=1, default=str))
PYEOF
echo "samples: $(wc -c < "$OUT/dds_samples.json") bytes"

# 4) 音声・腕・モードのサービス応答（読み取りのみ）
$SSH 'cd ~/unitree_sdk2_python && python3 -u -' > "$OUT/service_reads.txt" 2>&1 <<'PYEOF'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(0, "eth0")
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
for name, mk, calls in [
    ("audio", AudioClient, ["GetVolume"]),
    ("arm", G1ArmActionClient, ["GetActionList"]),
    ("motion_switcher", MotionSwitcherClient, ["CheckMode"]),
    ("loco", LocoClient, ["GetFsmId", "GetFsmMode", "GetBalanceMode", "GetSwingHeight", "GetStandHeight"])]:
    try:
        c = mk(); c.SetTimeout(3.0); c.Init()
        for m in calls:
            print(name, m, getattr(c, m)(), flush=True)
    except Exception as e:
        print(name, "error", repr(e), flush=True)
PYEOF
echo "services done"
ls -la "$OUT"
