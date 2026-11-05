"""11-03: rt/dog_odom を別のプロセスで受け、(受けた時刻, x, y, yaw) だけを UDP で 127.0.0.1 へ約 100 Hz で流す。

なぜ: rt/dog_odom は 1008 Hz で届き、Python の SDK は受け取り関数を呼ぶ前に毎回 Odometry_ を組み立てる。
純正の口（door_run_stock.py）と同じプロセスで受けると、それだけで実行権（GIL）を食い、
1 周期の計算が 12 → 17 ms、SetVelocity の応答が 5 → 76 ms（時間切れ 3 s）に延びた（11-03 実機、PC2）。
ここで受けて小さな電文にすれば、主の側は UDP を読むだけになる。

時刻は time.monotonic()（Linux の CLOCK_MONOTONIC は同じ機械のプロセスどうしで共通）。親のプロセスが居なくなったら終わる。
使い方（door_run_stock.py --odom relay が自分で起こす）: python3 odom_relay.py --iface eth0 --port 47331
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

PORT = 47331
FMT = "<dddd"  # 受けた時刻（monotonic）, x, y, yaw
DECIMATE = 10  # 1008 Hz を約 100 Hz に（別のプロセスなので聞き役のままでよい。走らせ役の RobotIO は 11-05 から取りに行く形）
PARENT_CHECK_S = 0.5


def pack(t: float, x: float, y: float, yaw: float) -> bytes:
    return struct.pack(FMT, t, x, y, yaw)


def unpack(b: bytes) -> Optional[Tuple[float, Tuple[float, float, float]]]:
    """(時刻, (x, y, yaw))。長さが違えば None"""
    if len(b) != struct.calcsize(FMT):
        return None
    t, x, y, yaw = struct.unpack(FMT, b)
    if not all(math.isfinite(v) for v in (t, x, y, yaw)):
        return None
    return t, (x, y, yaw)


class UdpOdom:
    """主の側: 中継の電文を別の糸で受け、最新の (odom, 時刻) を返す。RobotIO.snapshot の odom・odom_t の代わり"""

    def __init__(self, port: int = PORT) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[float, Tuple[float, float, float]]] = None
        self.count = 0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop:
            try:
                b = self.sock.recv(64)
            except socket.timeout:
                continue
            except OSError:
                return
            got = unpack(b)
            if got is None:
                continue
            with self._lock:
                self._latest = got
                self.count += 1

    def latest(self) -> Tuple[Optional[Tuple[float, float, float]], Optional[float]]:
        with self._lock:
            if self._latest is None:
                return None, None
            t, odom = self._latest
            return odom, t

    def close(self) -> None:
        self._stop = True
        self.sock.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args(argv)

    import numpy as np
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

    import door_frame as dfr

    ChannelFactoryInitialize(0, a.iface)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n = [0]

    def on_odom(msg) -> None:
        n[0] += 1
        if n[0] % DECIMATE:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = dfr.yaw_from_quat_wxyz(np.array([o.w, o.x, o.y, o.z]))
        tx.sendto(pack(time.monotonic(), float(p.x), float(p.y), float(yaw)), ("127.0.0.1", a.port))

    sub = ChannelSubscriber("rt/dog_odom", Odometry_)
    sub.Init(on_odom, 1)
    parent = os.getppid()
    print(f"[odom_relay] rt/dog_odom → 127.0.0.1:{a.port}（{DECIMATE} 回に 1 回）・親 {parent}", flush=True)
    while os.getppid() == parent:
        time.sleep(PARENT_CHECK_S)
    print("[odom_relay] 親が居なくなったので終わる", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
