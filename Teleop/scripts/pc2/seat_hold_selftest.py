#!/usr/bin/env python3
"""2026-11-04: 保持係（seat_hold.py）を本物の DDS（unitree_sdk2py）で回す自己テスト。機体は要らない。

lo の上の別の domain（既定 7。機体は 0・シミュレータは 1）で、偽の機体（rt/lowstate STATE_HZ）と偽の遠隔操作
（rt/lowcmd 250 Hz）を送り、保持係を子のプロセスで起こして、rt/lowcmd に流れた保持係の指令を全部聞く。
偽の機体は本物（約 650 Hz）より遅い 200 Hz にする。PC2 では Python の直列化が重く（LowState_ 1 件 0.92 ms）、
500 Hz だと 1 つの GIL に収まらず、偽の状態が途切れて段 4・5 が落ちた（11-04）。保持係は状態を 50 Hz で取りに行くので足りる。
保持係の HOME を一時フォルダに替えるので、~/teleop_logs の状態・PID のファイルには触らない。
判断そのものの試験は tests/test_seat_hold.py（DDS なし）。ここでは DDS の口（印・CRC・送る速さ・止め方）を見る。

    python3 seat_hold_selftest.py

確かめること:
  0. 歩きの RPC が答えない（lo に純正は居ない）と確かめてから引き継ぐ・2 つ目の保持係は起きずに終わる
  1. 遠隔操作が送っている間、保持係は 1 件も送らない
  2. 遠隔操作が止まると数十 ms で引き継ぎ、最後の指令（角度・kp）をそのまま 250 Hz ほどで送る。印と CRC が正しい
  3. 遠隔操作が送り始めると、すぐに黙る
  4. 機体の状態が途切れると 0.1 秒ほどで送るのをやめる（力を抜かない）
  5. 止める指示（SIGTERM）で kp を 2 秒で 0 へ下げてから送るのをやめ、プロセスが終わる
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import seat_hold as S  # noqa: E402

Q_TELEOP, KP_TELEOP = 0.21, 80.0
STATE_HZ, TELEOP_HZ = 200.0, 250.0


class Bench:
    """偽の機体・偽の遠隔操作・聞き役。"""

    def __init__(self, domain: int) -> None:
        from cyclonedds.core import Listener
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.qos import Policy, Qos
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(domain, "lo")
        self.crc = CRC()
        self.state_on, self.teleop_on, self.quit = threading.Event(), threading.Event(), threading.Event()
        self.heard, self.lock = [], threading.Lock()   # 保持係の指令: (時刻, q[0], kp[0], CRC が正しいか)

        self.state = unitree_hg_msg_dds__LowState_()
        for i in range(S.N_BODY):
            m = self.state.motor_state[i]
            m.mode, m.q, m.temperature = 1, 0.2, [40, 40]
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        for i in range(S.N_MOTORS):
            c = self.cmd.motor_cmd[i]
            c.mode, c.q, c.kp, c.kd = 1, Q_TELEOP, (KP_TELEOP if i < S.N_BODY else 0.0), 3.0
        self.cmd.crc = self.crc.Crc(self.cmd)

        self.pub_state = ChannelPublisher("rt/lowstate", LowState_)
        self.pub_state.Init()
        self.pub_cmd = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub_cmd.Init()
        # 聞き役は自分（偽の遠隔操作）の指令を受けない（IgnoreLocal）。受けると 250 件/秒ぶん戻す手間が GIL に乗る
        self.dp = DomainParticipant(domain)
        self.sub = DataReader(self.dp, Topic(self.dp, "rt/lowcmd", LowCmd_),
                              Qos(Policy.History.KeepLast(64), Policy.IgnoreLocal.Process),
                              Listener(on_data_available=self._on_cmd))
        threading.Thread(target=self._loop, args=(self.state_on, self.pub_state, self.state, STATE_HZ), daemon=True).start()
        threading.Thread(target=self._loop, args=(self.teleop_on, self.pub_cmd, self.cmd, TELEOP_HZ), daemon=True).start()

    def _on_cmd(self, reader) -> None:
        try:
            msgs = reader.take(N=64)
        except Exception:  # noqa: BLE001  終わる間際に読み手が消えていることがある
            return
        for msg in msgs:
            if getattr(msg, "reserve", None) is None or msg.reserve[3] != S.MARK:
                continue
            ok = self.crc.Crc(msg) == msg.crc
            with self.lock:
                self.heard.append((time.monotonic(), msg.motor_cmd[0].q, msg.motor_cmd[0].kp, ok))

    def _loop(self, on: threading.Event, pub, msg, hz: float) -> None:
        while not self.quit.is_set():
            if on.is_set():
                pub.Write(msg)
            time.sleep(1.0 / hz)

    def holder_msgs(self, t0: float, t1: float) -> list:
        with self.lock:
            return [h for h in self.heard if t0 <= h[0] < t1]


def main() -> int:
    p = argparse.ArgumentParser(description="保持係を本物の DDS で回す自己テスト（機体不要・lo）")
    p.add_argument("--domain", type=int, default=7)
    a = p.parse_args()
    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    bench = Bench(a.domain)
    bench.state_on.set()
    home = tempfile.mkdtemp(prefix="seat_hold_selftest_")
    proc = subprocess.Popen([sys.executable, "-u", str(HERE / "seat_hold.py"), "run", "--iface", "lo", "--domain", str(a.domain)],
                            env={**os.environ, "HOME": home}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    pid_file = Path(home) / "teleop_logs" / "seat_hold.pid"
    t0 = time.monotonic()
    while not pid_file.exists() and time.monotonic() - t0 < 10.0 and proc.poll() is None:
        time.sleep(0.05)
    if not pid_file.exists():
        print(f"[自己テスト] ✗ 保持係が起きなかった:\n{proc.communicate(timeout=5)[0]}")
        return 1
    status_file = Path(home) / "teleop_logs" / "seat_hold.status.json"

    def holder_status() -> dict:
        try:
            return json.loads(status_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    try:
        # 0. 歩きの RPC が答えない（lo には純正が居ない）と確かめるまでは引き継がない。確かめ終わるのを待つ
        t0 = time.monotonic()
        while holder_status().get("stock_running", True) and time.monotonic() - t0 < 10.0:
            time.sleep(0.1)
        check(holder_status().get("stock_running") is False,
              f"歩きの RPC が {S.STOCK_SILENT_POLLS} 回答えないと「純正は止まっている」とする（{time.monotonic() - t0:.1f} 秒）")
        second = subprocess.run([sys.executable, "-u", str(HERE / "seat_hold.py"), "run", "--iface", "lo",
                                 "--domain", str(a.domain)], env={**os.environ, "HOME": home},
                                capture_output=True, text=True, timeout=10)
        check(second.returncode == 1 and "もう別の保持係" in second.stdout and pid_file.read_text().strip() == str(proc.pid),
              "2 つ目の保持係は起きずに終わる（排他の鍵）")

        # 1. 遠隔操作が送っている間
        bench.teleop_on.set()
        time.sleep(0.2)
        ta = time.monotonic()
        time.sleep(1.0)
        check(not bench.holder_msgs(ta, time.monotonic()), "遠隔操作が送っている間、保持係は送らない")

        # 2. 遠隔操作が止まる
        bench.teleop_on.clear()
        tb = time.monotonic()
        time.sleep(0.6)
        got = bench.holder_msgs(tb, time.monotonic())
        first = got[0][0] - tb if got else float("inf")
        hz = (len(got) - 1) / (got[-1][0] - got[0][0]) if len(got) > 1 else 0.0
        check(first < 0.1, f"遠隔操作が止まってから {first * 1000:.0f} ms で引き継ぐ（線 {S.GAP_S * 1000:.0f} ms＋周期）")
        check(150 < hz < 300, f"引き継いだ後は {hz:.0f} 件/秒で送る")
        check(bool(got) and all(abs(q - Q_TELEOP) < 1e-6 and abs(kp - KP_TELEOP) < 1e-6 for _, q, kp, _ in got),
              "最後の指令（角度・kp）をそのまま送る")
        check(bool(got) and all(ok for *_, ok in got), "印（reserve[3]）付きで、CRC が正しい")

        # 3. 遠隔操作が送り始める
        bench.teleop_on.set()
        tc = time.monotonic()
        time.sleep(0.4)
        late = bench.holder_msgs(tc + 0.03, time.monotonic())
        check(not late, f"遠隔操作が送り始めると黙る（30 ms より後に {len(late)} 件）")

        # 4. 機体の状態が途切れる
        bench.teleop_on.clear()
        time.sleep(0.3)
        bench.state_on.clear()
        te = time.monotonic()
        time.sleep(0.4)
        got = bench.holder_msgs(te, time.monotonic())
        last = got[-1][0] - te if got else 0.0
        check(bool(got) and last < 0.2 and all(abs(kp - KP_TELEOP) < 1e-6 for _, _, kp, _ in got),
              f"状態が途切れて {last * 1000:.0f} ms で送るのをやめる（力は抜かない）")

        # 5. 止める指示
        bench.state_on.set()
        bench.teleop_on.set()
        time.sleep(0.2)
        bench.teleop_on.clear()
        time.sleep(0.3)
        tg = time.monotonic()
        proc.send_signal(signal.SIGTERM)
        try:
            code = proc.wait(timeout=S.RELEASE_S + 3.0)
        except subprocess.TimeoutExpired:      # 落ちずに不合格として数え、下で保持係のログを出す
            code = None
        got = bench.holder_msgs(tg, time.monotonic())
        kps = [kp for _, _, kp, _ in got]
        span = got[-1][0] - tg if got else 0.0
        check(bool(kps) and kps[0] > 0.8 * KP_TELEOP and kps[-1] < 0.05 * KP_TELEOP
              and all(x >= y - 1e-9 for x, y in zip(kps, kps[1:])),
              f"止める指示で kp を {kps[0] if kps else 0:.0f}→{kps[-1] if kps else 0:.1f} へ {span:.1f} 秒で下げる")
        check(code == 0 and not pid_file.exists(), f"プロセスが終わり、PID のファイルを片付けた（{f'終了コード {code}' if code is not None else '5 秒待っても終わらない'}）")
    finally:
        bench.quit.set()
        if proc.poll() is None:
            proc.kill()
        out = proc.communicate(timeout=5)[0]
    print("  ― 保持係のログ ―")
    print("\n".join("    " + ln for ln in out.strip().splitlines()))
    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
