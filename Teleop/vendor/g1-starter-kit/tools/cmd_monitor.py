#!/usr/bin/env python3
"""指令と実測を同時に眺めて「送っているのに動かない」を切り分ける（読み取り専用）。

    python3 tools/cmd_monitor.py --iface enp129s0 --seconds 120

別の端末でテレオペや再生を動かしながら走らせる。1 秒ごとに次を出す。

    arm_sdk   … rt/arm_sdk  の受信数。teleop --motion / 再生(arm_sdk) が出す
    lowcmd    … rt/lowcmd   の受信数。teleop（--motion なし）/ 再生(lowcmd) が出す。
                **何も起動していないのに 1000Hz で増えていれば、それはロボットの
                モーションコントローラ自身の出力**で、この状態の rt/lowcmd は無視される
    lowstate  … rt/lowstate の受信数（実測）。約 1000Hz が正常
    weight    … arm_sdk の制御権 motor_cmd[29].q。1.00 で満額
    総移動量  … 開始時からの腕 14 関節の |Δq| 合計 [rad]。動いていれば増減する

読み方の例:
    lowcmd が増える / 総移動量が増えない → モータ無効(mode=0) or コントローラが上書き
    arm_sdk が増える / weight=1.00 / 総移動量が増えない → 委譲に応じていない
    どれも増えない → 送信側が起動していない
"""
import argparse
import threading
import time

ARM = list(range(15, 29))
WEIGHT_IDX = 29


def main():
    p = argparse.ArgumentParser(description="指令と実測の同時監視（読み取り専用）")
    p.add_argument("--iface", required=True)
    p.add_argument("--seconds", type=float, default=120.0)
    args = p.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    ChannelFactoryInitialize(0, args.iface)

    lock = threading.Lock()
    st = {"armsdk": 0, "lowcmd": 0, "state": 0, "weight": None, "cmd_q": None,
          "state_q": None, "q0": None}

    def on_armsdk(m):
        with lock:
            st["armsdk"] += 1
            st["weight"] = m.motor_cmd[WEIGHT_IDX].q
            st["cmd_q"] = [m.motor_cmd[i].q for i in ARM]

    def on_lowcmd(m):
        with lock:
            st["lowcmd"] += 1

    def on_state(m):
        with lock:
            st["state"] += 1
            q = [m.motor_state[i].q for i in ARM]
            st["state_q"] = q
            if st["q0"] is None:
                st["q0"] = list(q)

    for topic, typ, cb in (("rt/arm_sdk", LowCmd_, on_armsdk),
                           ("rt/lowcmd", LowCmd_, on_lowcmd),
                           ("rt/lowstate", LowState_, on_state)):
        ChannelSubscriber(topic, typ).Init(cb, 10)

    print(f"{'経過':>5} {'arm_sdk':>8} {'lowcmd':>7} {'lowstate':>9} {'weight':>7} {'総移動量(rad)':>14}")
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            time.sleep(1.0)
            with lock:
                moved = ""
                if st["state_q"] and st["q0"]:
                    moved = f"{sum(abs(a - b) for a, b in zip(st['state_q'], st['q0'])):.4f}"
                w = "―" if st["weight"] is None else f"{st['weight']:.2f}"
                print(f"{time.time()-t0:5.0f} {st['armsdk']:8d} {st['lowcmd']:7d} "
                      f"{st['state']:9d} {w:>7} {moved:>14}")
    except KeyboardInterrupt:
        pass

    with lock:
        print("\n=== まとめ ===")
        print(f"  rt/arm_sdk  {st['armsdk']} 件 / rt/lowcmd {st['lowcmd']} 件 / rt/lowstate {st['state']} 件")
        if st["weight"] is not None:
            print(f"  arm_sdk の制御権 weight = {st['weight']:.3f}")
        if st["state_q"] and st["q0"]:
            d = max(abs(a - b) for a, b in zip(st["state_q"], st["q0"]))
            print(f"  腕の最大変化 = {d:.4f} rad  （0.01 未満なら動いていない）")


if __name__ == "__main__":
    main()
