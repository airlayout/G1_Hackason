"""ダンピング検出と同時に CMD_START を送る（通電が切れる前に追従を始める）。"""
import sys, time, uuid
import zmq
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

ChannelFactoryInitialize(0, "enp129s0")
st = {}
ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: st.__setitem__("m", m), 10)
t0 = time.time()
while "m" not in st and time.time() - t0 < 25:
    time.sleep(0.05)
if "m" not in st:
    print("ERR lowstate 受信不可", flush=True); raise SystemExit(1)

ctx = zmq.Context.instance()
req = ctx.socket(zmq.REQ); req.setsockopt(zmq.RCVTIMEO, 15000); req.setsockopt(zmq.LINGER, 0)
req.connect("ipc://@xr_teleoperate_data.ipc")

print("ARMED  リモコンでダンピングを入れた瞬間に CMD_START を送ります", flush=True)
t0 = time.time(); last = None
while time.time() - t0 < 600:
    m = st["m"]
    arm = sorted({m.motor_state[i].mode for i in range(15, 29)})
    if arm != last:
        print(f"STATE t={time.time()-t0:6.1f}s arm_mode={arm}", flush=True)
        last = arm
    if arm == [1]:
        print(f"DETECT t={time.time()-t0:.1f}s -> CMD_START 送信", flush=True)
        req.send_json({"reqid": str(uuid.uuid4()), "cmd": "CMD_START"})
        try:
            print("REPLY", req.recv_json(), flush=True)
        except zmq.Again:
            print("REPLY タイムアウト", flush=True)
        break
    time.sleep(0.02)
else:
    print("TIMEOUT 変化なし", flush=True)
