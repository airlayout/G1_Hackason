"""読み取りのみ: rt/audio_msg 系を 40 秒購読し、GetActionList / GetSilent / 録音デバイスを調べる。"""
import shlex
import subprocess

SRC = r'''
import time
from cyclonedds.domain import DomainParticipant
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
ChannelFactoryInitialize(0, "eth0")
dp = DomainParticipant(0)
rs = {t: DataReader(dp, Topic(dp, t, String_)) for t in ("rt/audio_msg", "rt/audio_msg/filter")}
print("LISTENING 40s", flush=True)
end = time.time() + 40
while time.time() < end:
    for t, r in rs.items():
        for s in r.take(N=20):
            print(time.strftime("%H:%M:%S"), t, repr(s.data)[:300], flush=True)
    time.sleep(0.2)
print("done")
'''
r = subprocess.run(["ssh", "-o", "BatchMode=yes", "g1-ts",
                    "cd ~/unitree_sdk2_python && python3 -u -c " + shlex.quote(SRC)],
                   capture_output=True, text=True, timeout=120)
print(r.stdout)
print(r.stderr[-800:] if r.returncode else "")
