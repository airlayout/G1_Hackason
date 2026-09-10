#!/usr/bin/env python3
"""この機体で rt/arm_sdk（制御権の委譲）が効くかを、最小の動きで確かめる。

    python3 tools/armsdk_probe.py --iface enp129s0

⚠️ 実機の腕が **1 関節だけ、0.05 rad（約 3°）** 動きます。支持された状態で、
   可動範囲をクリアにして実行してください。実行前に確認を求めます。

何をするか（公式 arm_sdk の作法どおり、腕 14 関節と制御権だけを送る）:
    ① 制御権 motor_cmd[29].q を 0→1 に 2 秒かけて上げる（他の腕関節は現在角で保持）
    ② 対象関節を +0.05 rad へ 2 秒かけて動かす
    ③ 1 秒保持 → ④ 元へ戻す → ⑤ 制御権を 1→0 に戻してコントローラに返す

判定:
    実測が 0.025 rad 以上動く → arm_sdk が効く（teleop --motion が使える）
    0.000 rad のまま         → 委譲に応じていない。デバッグ状態 + rt/lowcmd を使う

xr_teleoperate は arm_sdk モードでも全 29 関節に kp/kd/q を書く（脚は kp=300）。
このツールはそれをしないので、「効かないのは teleop の送り方のせいか、機体のせいか」
を切り分けられる。検証した機体（G1 29DoF, Regular モード）では 0.000 rad だった。
"""
import argparse
import sys
import threading
import time

ARM = list(range(15, 29))
WRIST = {19, 20, 21, 26, 27, 28}
WEIGHT_IDX = 29
DT = 1.0 / 250.0
NAMES = {15: "L_ShoulderPitch", 16: "L_ShoulderRoll", 17: "L_ShoulderYaw", 18: "L_Elbow",
         22: "R_ShoulderPitch", 23: "R_ShoulderRoll", 24: "R_ShoulderYaw", 25: "R_Elbow"}


def main():
    p = argparse.ArgumentParser(description="rt/arm_sdk が効くかを 3 度だけ動かして確かめる")
    p.add_argument("--iface", required=True)
    p.add_argument("--joint", type=int, default=15, choices=sorted(NAMES),
                   help="動かす関節のモータ番号（既定 15 = 左肩ピッチ）")
    p.add_argument("--amplitude", type=float, default=0.05, help="変位 [rad]（既定 0.05）")
    p.add_argument("--yes", action="store_true", help="確認を省略する")
    args = p.parse_args()
    if not (0 < args.amplitude <= 0.15):
        sys.exit("エラー: --amplitude は 0 より大きく 0.15 以下にしてください。")

    from unitree_sdk2py.core.channel import (ChannelPublisher, ChannelSubscriber,
                                             ChannelFactoryInitialize)
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    ChannelFactoryInitialize(0, args.iface)

    box, lock = {}, threading.Lock()

    def on_state(m):
        with lock:
            box["m"] = m

    ChannelSubscriber("rt/lowstate", LowState_).Init(on_state, 10)
    t0 = time.time()
    while "m" not in box and time.time() - t0 < 10:
        time.sleep(0.05)
    if "m" not in box:
        sys.exit("エラー: rt/lowstate を受信できません。有線接続と電源を確認してください。")

    with lock:
        m = box["m"]
        q0 = {i: m.motor_state[i].q for i in ARM}
        mode_machine = m.mode_machine
        modes = {m.motor_state[i].mode for i in ARM}

    def meas():
        with lock:
            return box["m"].motor_state[args.joint].q

    print(f"対象: {NAMES[args.joint]} (motor {args.joint})  現在 {q0[args.joint]:+.4f} rad "
          f"→ 目標 {q0[args.joint] + args.amplitude:+.4f} rad")
    print(f"mode_machine={mode_machine}  腕モータ mode={sorted(modes)}")
    if modes != {1}:
        print("⚠️ 腕モータが無効(mode=0)です。この状態では arm_sdk 以前に何も動きません。")
    if not args.yes:
        if input("\n腕が約 3° 動きます。支持と周囲を確認しましたか? [y/N] ").strip().lower() != "y":
            print("中止しました。")
            return 1

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = mode_machine
    for i in ARM:                      # 腕 14 関節だけ。他は既定値のまま（無指令）
        c = msg.motor_cmd[i]
        c.mode = 1
        c.kp = 40.0 if i in WRIST else 80.0
        c.kd = 1.5 if i in WRIST else 3.0
        c.q, c.dq, c.tau = q0[i], 0.0, 0.0

    def send(weight, target_q):
        msg.motor_cmd[WEIGHT_IDX].q = float(weight)
        msg.motor_cmd[args.joint].q = float(target_q)
        msg.crc = crc.Crc(msg)
        pub.Write(msg)

    base, amp = q0[args.joint], args.amplitude

    def phase(label, secs, wfun, qfun):
        n = int(secs / DT)
        for k in range(n):
            s = (k + 1) / n
            send(wfun(s), qfun(s))
            time.sleep(DT)
        print(f"  {label:<26} weight={msg.motor_cmd[WEIGHT_IDX].q:.2f}  "
              f"指令={msg.motor_cmd[args.joint].q:+.4f}  実測={meas():+.4f}  "
              f"偏差={meas() - base:+.4f}")

    peak = 0.0
    try:
        print("\n実行:")
        phase("① 制御権 0→1 (2秒)", 2.0, lambda s: s, lambda s: base)
        phase("② 目標へ (2秒)", 2.0, lambda s: 1.0, lambda s: base + amp * s)
        phase("③ 保持 (1秒)", 1.0, lambda s: 1.0, lambda s: base + amp)
        peak = abs(meas() - base)
        phase("④ 元の位置へ (2秒)", 2.0, lambda s: 1.0, lambda s: base + amp * (1 - s))
        phase("⑤ 制御権 1→0 (2秒)", 2.0, lambda s: 1 - s, lambda s: base)
    finally:
        for _ in range(50):
            send(0.0, base)
            time.sleep(DT)
        print("\n  制御権をコントローラに返しました。")

    print("\n=== 判定 ===")
    print(f"  指令した変位 : {amp:+.4f} rad  /  実測の最大変位: {peak:+.4f} rad")
    if peak > amp * 0.5:
        print("  ✅ arm_sdk が効いています。teleop --motion が使えます。")
        return 0
    if peak > 0.005:
        print("  △ わずかに動きました。コントローラと競合している可能性があります。")
        return 2
    print("  ❌ arm_sdk が効いていません。デバッグ状態 + rt/lowcmd を使ってください:")
    print(f"     python3 tools/mode_check.py --iface {args.iface} --release")
    return 3


if __name__ == "__main__":
    sys.exit(main())
