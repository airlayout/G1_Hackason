"""rt/lowstate の wireless_remote を読むだけ（機体には何も送らない）。ボタンかスティックが変わった瞬間を、PC2 の時計の時刻つきで 1 行ずつ出す。

段 1（2026-10-21）で、記録のどこで人がリモコンを動かしたかを後から切り出すのに使う（record_dds_to_bag.py の db3 の timestamp と同じ time.time_ns()）。
起動の直後に機体へ届いているかの確かめにも使える（再起動のあと数分は 0 のまま）。

使い方（PC2）: python3 watch_remote.py <秒>
出力: [remote] <time_ns> keys=0x.... lx=... ly=... rx=... ry=...  と、最後に件数
"""
import signal
import struct
import sys
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

STICK_STEP = 0.05  # これより動いたら 1 行出す


def main() -> None:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    state = {"last": None, "n": 0}

    def on(msg) -> None:
        wr = bytes(msg.wireless_remote)
        keys = wr[2] | (wr[3] << 8)
        lx, rx, ry, _l2, ly = struct.unpack_from("<5f", wr, 4)
        now = (keys, lx, ly, rx, ry)
        state["n"] += 1
        last = state["last"]
        if last is None or keys != last[0] or max(abs(a - b) for a, b in zip(now[1:], last[1:])) > STICK_STEP:
            print(f"[remote] {time.time_ns()} keys=0x{keys:04x} lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f}", flush=True)
            state["last"] = now

    ChannelFactoryInitialize(0, "eth0")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on, 10)
    # rec_stop.sh の SIGINT で早めに止める。⚠️ 裏で起こすと SIGINT が無視の設定で引き継がれ、
    # KeyboardInterrupt が出ない（10-21 に止まらなかった）。だから受け口を自分で付ける
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    t0 = time.time()
    stop.wait(secs)
    print(f"[remote] 終わり lowstate {state['n']} 件 / {time.time() - t0:.0f} 秒", flush=True)


if __name__ == "__main__":
    main()
