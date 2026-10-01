"""ダンピング検出と同時に teleop を起動する（人間の往復待ちを挟まない）。"""
import subprocess, sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_

TELEOP_LOG = sys.argv[1]
ChannelFactoryInitialize(0, "enp129s0")
st = {}; n = {"c": 0}
ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: st.__setitem__("m", m), 10)
ChannelSubscriber("rt/lowcmd", LowCmd_).Init(lambda m: n.__setitem__("c", n["c"] + 1), 10)

t0 = time.time()
while "m" not in st and time.time() - t0 < 25:
    time.sleep(0.05)
if "m" not in st:
    print("ERR lowstate 受信不可", flush=True); raise SystemExit(1)

print("WAIT リモコンでダンピング（FSM 1）に入れてください。検出と同時に teleop を起動します。", flush=True)
t0 = time.time(); hz = 0.0; tw = time.time(); last = None
while time.time() - t0 < 600:
    if time.time() - tw >= 1.0:
        hz = n["c"] / (time.time() - tw); n["c"] = 0; tw = time.time()
    m = st["m"]
    arm = sorted({m.motor_state[i].mode for i in range(15, 29)})
    key = (tuple(arm), hz > 50)
    if key != last:
        print(f"STATE t={time.time()-t0:6.1f}s arm_mode={arm} lowcmd={'busy' if hz>50 else 'free'}", flush=True)
        last = key
    if arm == [1] and hz <= 50:
        print(f"DETECT t={time.time()-t0:.1f}s -> teleop 起動", flush=True)
        with open(TELEOP_LOG, "ab") as fh:
            subprocess.Popen(["./scripts/teleop.sh", "--ipc", "--headless"],
                             stdout=fh, stderr=fh, start_new_session=True,
                             cwd="/home/ubuntu/g1-starter-kit")
        print("LAUNCHED", flush=True)
        break
    time.sleep(0.05)
else:
    print("TIMEOUT 変化なし", flush=True)
