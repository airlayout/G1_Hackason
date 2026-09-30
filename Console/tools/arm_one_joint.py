"""腕を 1 関節だけ、ごく小さく動かし、指令に対する変化量を観察する（Jetson 上で実行）。

    python3 -u - [--execute] [--delta 0.05] < arm_one_joint.py     （既定はドライラン。何も送らない）

rt/arm_sdk（公式例と同じ経路）に、腕の全関節を「現在角のまま・現在と同じゲイン」で送り、
左手首ヨー（21）だけを --delta [rad] 動かして戻す。rt/lowcmd には送らない（内部の保持と競合するため）。
手順: weight 0→1（2 s, 動かさない）→ 保持 1 s → +delta（2 s）→ 保持 1.5 s → 元へ（2 s）→ 保持 1 s → weight 1→0（2 s）
中断条件: バッテリー <= 2% / 他の関節が初期位置から 0.1 rad 以上ずれた / 対象関節が指令から 0.15 rad 以上ずれた。
中断・例外でも必ず weight を 0 へ戻す（arm_sdk を解放する）。
"""
import argparse
import json
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.internal import InvalidSample
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

TARGET = 21                                  # LeftWristYaw
ARM = list(range(15, 29))                    # 左腕 7 + 右腕 7
WAIST = [12, 13, 14]
HELD = ARM + WAIST                           # arm_sdk で「今の姿勢のまま」保持する関節
WEIGHT = 29                                  # kNotUsedJoint: arm_sdk の重み
DT = 0.02
MIN_SOC = 2

ap = argparse.ArgumentParser()
ap.add_argument("--execute", action="store_true")
ap.add_argument("--delta", type=float, default=0.05)
args = ap.parse_args()
assert abs(args.delta) <= 0.1, "delta は 0.1 rad 以内"

ChannelFactoryInitialize(0, "eth0")
dp = DomainParticipant(0)
qos = Qos(Policy.History.KeepLast(1))
r_low = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos)
r_cmd = DataReader(dp, Topic(dp, "rt/lowcmd", LowCmd_), qos)
r_bms = DataReader(dp, Topic(dp, "rt/lf/bmsstate", BmsState_), qos)


def latest(reader, wait=3.0):
    end = time.time() + wait
    while time.time() < end:
        for s in reader.take(N=1):
            if not isinstance(s, InvalidSample):
                return s
        time.sleep(0.01)
    raise RuntimeError("受信できない")


low, cmd0, bms = latest(r_low), latest(r_cmd), latest(r_bms)
q0 = {j: low.motor_state[j].q for j in HELD}
kp = {j: cmd0.motor_cmd[j].kp for j in HELD}
kd = {j: cmd0.motor_cmd[j].kd for j in HELD}
print(json.dumps({"soc": bms.soc, "mode_machine": low.mode_machine, "target_q": q0[TARGET],
                  "kp_kd_target": [kp[TARGET], kd[TARGET]], "delta": args.delta,
                  "execute": args.execute}), flush=True)
assert bms.soc > MIN_SOC, "バッテリーが少なすぎる"
assert kp[TARGET] > 0, "対象関節に保持指令が出ていない（内部制御が動いていない）"

if not args.execute:
    print("ドライラン: 送信しません。--execute で実行。")
    raise SystemExit(0)

pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
pub.Init()
crc = CRC()
msg = unitree_hg_msg_dds__LowCmd_()
rows = []


def send(weight, offset):
    for j in HELD:
        m = msg.motor_cmd[j]
        m.q = q0[j] + (offset if j == TARGET else 0.0)
        m.dq, m.tau, m.kp, m.kd = 0.0, 0.0, kp[j], kd[j]
    msg.motor_cmd[WEIGHT].q = weight
    msg.crc = crc.Crc(msg)
    pub.Write(msg)


def step(phase, weight, offset):
    """1 周期送って観測する。異常なら例外。"""
    send(weight, offset)
    s = latest(r_low, 0.5)
    q = s.motor_state[TARGET].q
    rows.append((phase, time.time(), weight, offset, q))
    worst = max((abs(s.motor_state[j].q - q0[j]), j) for j in HELD if j != TARGET)
    if worst[0] > 0.1:
        raise RuntimeError("他の関節が動いた: %d %.3f rad" % (worst[1], worst[0]))
    if weight >= 0.99 and abs(q - (q0[TARGET] + offset)) > 0.15:
        raise RuntimeError("対象関節が指令に追従していない: %.3f rad" % (q - q0[TARGET] - offset))
    time.sleep(DT)


def ramp(phase, seconds, frm, to, fn):
    n = int(seconds / DT)
    for i in range(1, n + 1):
        v = frm + (to - frm) * i / n
        fn(v)


last_weight = 0.0
try:
    def w(v):
        global last_weight
        last_weight = v
        step("weight_up", v, 0.0)
    ramp("weight_up", 2.0, 0.0, 1.0, w)
    for _ in range(int(1.0 / DT)):
        step("hold0", 1.0, 0.0)
    ramp("out", 2.0, 0.0, args.delta, lambda v: step("out", 1.0, v))
    for _ in range(int(1.5 / DT)):
        step("hold_out", 1.0, args.delta)
    ramp("back", 2.0, args.delta, 0.0, lambda v: step("back", 1.0, v))
    for _ in range(int(1.0 / DT)):
        step("hold_back", 1.0, 0.0)
    print("正常終了", flush=True)
except Exception as exc:  # noqa: BLE001  どんな失敗でも解放処理へ
    print("中断:", repr(exc), flush=True)
finally:
    n = max(1, int(2.0 / DT))
    for i in range(1, n + 1):  # arm_sdk を段階的に解放（保持位置は今の指令のまま）
        send(last_weight * (1 - i / n), 0.0)
        time.sleep(DT)
    send(0.0, 0.0)

phases = {}
for ph, _, wt, off, q in rows:
    phases.setdefault(ph, []).append((q - q0[TARGET], off))
print("phase        n   指令[rad]   観測変化 min/max [rad]")
for ph, v in phases.items():
    print("%-10s %4d   %+.3f..%+.3f   %+.4f / %+.4f" % (ph, len(v), min(o for _, o in v), max(o for _, o in v),
                                                   min(d for d, _ in v), max(d for d, _ in v)))
