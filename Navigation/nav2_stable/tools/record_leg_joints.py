#!/usr/bin/env python3
"""U-07: LowState の関節位置を記録し、脚がいつ動き出し・いつ止まったかを数値で出す。

**PC2 の system python3.8 で動かす**(unitree_sdk2py が入っているのはこちら)。
**購読のみ。指令は一切送らない。**

安全支持具で固定していると胴体が並進しないので odom では測れない。
関節の動きを直接見れば、支持されたままでも「duration 満了後に脚が止まるか」が分かる。

    python3 record_joints.py 60 /tmp/u07_joints.csv
"""
import sys
import time

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/u07_joints.csv"
# G1 29DOF の脚は index 0..11 (左右6関節ずつ)
LEG_JOINTS = list(range(12))

rows = []


def on_lowstate(msg):
    t = time.time()
    q = [msg.motor_state[i].q for i in LEG_JOINTS]
    dq = [msg.motor_state[i].dq for i in LEG_JOINTS]
    rows.append((t, q, dq))


def main() -> int:
    ChannelFactoryInitialize(0, "eth0")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)

    print("[rec] %.0f 秒記録する。この間に SetVelocity を1回だけ送ってください" % DURATION, flush=True)
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < DURATION:
        time.sleep(0.05)
        el = time.time() - t0
        if el - last >= 2.0 and len(rows) > 5:
            # 直近0.5秒の関節速度の絶対値の最大 = 脚が動いているかの指標
            recent = [r for r in rows if r[0] > time.time() - 0.5]
            if recent:
                peak = max(max(abs(v) for v in r[2]) for r in recent)
                print("[rec] t=%5.1fs  脚の関節速度|max| = %.4f rad/s  %s"
                      % (el, peak, "★動いている" if peak > 0.05 else "静止"), flush=True)
            last = el

    with open(OUT, "w") as f:
        f.write("t," + ",".join("q%d" % i for i in LEG_JOINTS) +
                "," + ",".join("dq%d" % i for i in LEG_JOINTS) + "\n")
        for t, q, dq in rows:
            f.write("%.6f,%s,%s\n" % (t, ",".join("%.6f" % v for v in q),
                                      ",".join("%.6f" % v for v in dq)))
    print("\n[rec] %d サンプル記録した (%.1f Hz) -> %s"
          % (len(rows), len(rows) / DURATION, OUT), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
