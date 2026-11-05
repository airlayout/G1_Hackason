"""11-04: 純正の歩行の RPC（LocoClient・SetVelocity）を別のプロセスで持ち、主の側からは UDP で最新の歩きの指令だけを受ける。

なぜ: 同じ失敗（SetVelocity の返事が待ちの上限までに来ない・3104）を、LocoClient を専用の子プロセスへ移して直した例がある
（4paradigm/phanthymotus-driver のコミット 24274c5。同じプロセスの多くの糸が実行権（GIL）を取り合い、DDS の受信の呼び戻しが
待たされる）。11-03 も turn_probe.py 単体では返事 1〜18 ms、door_run_stock.py の中では 56〜76 ms。11-04 の実機では、
待ち 1.0 s で 20 回中 5 回、0.25 s で 38 回中 18 回・73 回中 20 回が落ちた。

作り（odom_relay.py と同じく、主の側の door_run_stock.py --loco relay が自分で起こす）:
  主 → 子  127.0.0.1:CMD_PORT へ、変わった時の set（歩くか・vx・vy・wz）／stop／damp と、毎周期（50 回/秒）の state。
          state は「今の望み（歩くか・速さ・脱力）」を丸ごと載せる。UDP で set・stop・damp の 1 発が落ちても、次の state で届く
          （子の側の当てはめは何度受けても同じになる）。終わりは quit
  子      dsio.VelocitySender をそのまま回す（20 回/秒・1 回の効き目 0.5 s・歩くのをやめたら速度 0 を 3 回）。送り手の糸は
          最初の指令（set・stop・damp・歩くか脱力の state）で起こす（主の側が起動を断ったときは何も送らない。同じプロセスの作りと同じ）
  子 → 主  127.0.0.1:STATUS_PORT へ状態（送った数・続けて失敗した数・番号・応答の時間・今の指令・脱力・FSM・子の pid）を 50 回/秒
起動の時に子が FSM を読んで状態に入れる（主のプロセスは、ふだん LocoClient を持たない）。
安全:
  - 子は、主の電文が PING_TIMEOUT_S 来なければ歩くのをやめる（主の輪が固まっても歩き続けない）
  - 子は、親が居なくなったら止める指令を送り切ってから終わる。親は起こされた時の --parent-pid で見る（初期化の間に親が落ちても気づく）
  - 子は Ctrl+C・ssh の切断（SIGINT・SIGHUP）では終わらない。主の側が立ったまま止める・脱力を送るのに子が要るため。
    終わるのは主の側の quit・SIGTERM・親が居なくなったときだけ
  - 主の側は、子の pid の違う状態（居残った前の子など）を受け付けない。子が終わっていれば状態は「無限に古い」
  - 主の側は、脱力を求めたのに子が居ない（終わった・状態が DEAD_S 来ない）なら、自分で LocoClient を作って Damp を 1 回送る
使い方: python3 loco_relay.py --iface eth0 --cmd-port 47341 --status-port 47342 --rpc-timeout 1.0 --parent-pid <主の pid>
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

import door_stock_io as dsio  # noqa: E402

CMD_PORT = 47341
STATUS_PORT = 47342
PING_TIMEOUT_S = 0.3  # 主の側の電文がこれだけ来なければ歩くのをやめる（主の輪は 20 ms ごとに送る）
STATUS_PERIOD_S = 0.02  # 状態を主の側へ送る間隔
PARENT_CHECK_S = 0.5
RECV_TIMEOUT_S = 0.01
DEAD_S = 1.0  # 主の側: 子の状態がこれだけ来なければ、脱力は自分で送る
KINDS = ("set", "stop", "damp", "state", "quit")
ERROR_CHARS = 120  # 状態の電文に入れる失敗の文の長さの上限
NO_STATUS = dsio.VelStatus(0, 0, None, 0.0, (False, 0.0, 0.0, 0.0), False, "中継から状態が来ていない", math.inf)

Cmd = Tuple[str, bool, Tuple[float, float, float], bool]  # (種類, 歩くか, (vx, vy, wz), 脱力)


def pack_cmd(kind: str, walk: bool = False, v: Tuple[float, float, float] = (0.0, 0.0, 0.0), damp: bool = False) -> bytes:
    return json.dumps({"k": kind, "w": bool(walk), "v": [float(x) for x in v], "d": bool(damp)}).encode()


def unpack_cmd(b: bytes) -> Optional[Cmd]:
    """(種類, 歩くか, (vx, vy, wz), 脱力)。壊れていれば None（歩かせない）"""
    try:
        d = json.loads(b.decode())
        kind, walk, v, damp = d["k"], bool(d["w"]), tuple(float(x) for x in d["v"]), bool(d["d"])
    except (ValueError, KeyError, TypeError, UnicodeDecodeError):
        return None
    if kind not in KINDS or len(v) != 3 or not all(math.isfinite(x) for x in v):
        return None
    return kind, walk, v, damp


def pack_status(st: dsio.VelStatus, fsm: Optional[int], t: float, pid: int) -> bytes:
    return json.dumps({"t": t, "fsm": fsm, "pid": pid, "sent": st.sent, "failed": st.failed_in_row, "code": st.last_code,
                       "ms": st.last_rpc_ms, "want": list(st.want), "damped": st.damped,
                       "err": st.last_error[:ERROR_CHARS]}).encode()


def unpack_status(b: bytes) -> Optional[Tuple[float, dsio.VelStatus, Optional[int], int]]:
    """(子の時刻, 状態, FSM, 子の pid)。壊れていれば None"""
    try:
        d = json.loads(b.decode())
        w = d["want"]
        st = dsio.VelStatus(sent=int(d["sent"]), failed_in_row=int(d["failed"]),
                            last_code=None if d["code"] is None else int(d["code"]), last_rpc_ms=float(d["ms"]),
                            want=(bool(w[0]), float(w[1]), float(w[2]), float(w[3])), damped=bool(d["damped"]),
                            last_error=str(d["err"]))
        return float(d["t"]), st, None if d["fsm"] is None else int(d["fsm"]), int(d["pid"])
    except (ValueError, KeyError, TypeError, IndexError, UnicodeDecodeError):
        return None


def apply_cmd(sender: dsio.VelocitySender, cmd: Cmd) -> None:
    """指令を送り手に置く。state は何度受けても同じ結果（歩かないなら、歩いていた時だけ速度 0 を 3 回。脱力は 1 回）"""
    kind, walk, (vx, vy, wz), damp = cmd
    if kind == "damp" or (kind == "state" and damp):
        sender.damp()
    elif kind in ("set", "state"):
        sender.set(walk, vx, vy, wz)
    elif kind == "stop":
        sender.stop()


def serve(sender: dsio.VelocitySender, sock: socket.socket, status_addr: Tuple[str, int], fsm: Optional[int],
          parent_alive: Callable[[], bool], stop: Optional[threading.Event] = None,
          clock: Callable[[], float] = time.monotonic, pid: Optional[int] = None) -> str:
    """子の輪: 指令を受けて送り手に置き、状態を返す。終わった理由を返す。

    送り手（まだ起こしていないもの）は最初の set・stop・damp（か歩く・脱力の state）で起こす。閉じるのは呼んだ側
    （起こしていなければ何も送らない）
    """
    stop = stop or threading.Event()
    pid = os.getpid() if pid is None else pid
    sock.settimeout(RECV_TIMEOUT_S)
    last_rx = clock()
    last_status = last_parent = -1e9
    silent = started = False
    while not stop.is_set():
        try:
            b = sock.recv(512)
        except socket.timeout:
            b = None
        except OSError:
            return "受け口が閉じた"
        now = clock()
        cmd = unpack_cmd(b) if b else None
        if cmd is not None:
            last_rx, silent = now, False
            kind, walk, _, damp = cmd
            if kind == "quit":
                return "主の側から終わりの合図"
            if not started and (kind != "state" or walk or damp):
                sender.start()
                started = True
            apply_cmd(sender, cmd)
        elif not silent and now - last_rx > PING_TIMEOUT_S and sender.status().want[0]:
            sender.stop()
            silent = True
            print(f"[loco_relay] 主の側から {now - last_rx:.2f} s 電文が来ない: 歩くのをやめた", flush=True)
        if now - last_status >= STATUS_PERIOD_S:
            last_status = now
            try:
                sock.sendto(pack_status(sender.status(), fsm, now, pid), status_addr)
            except OSError:
                pass  # 主の側の受け口がまだ無い・もう無い。次の回に送る
        if now - last_parent >= PARENT_CHECK_S:
            last_parent = now
            if not parent_alive():
                return "親が居なくなった"
    return "止める合図"


class LocoRelay:
    """主の側: VelocitySender と同じ口（start・set・stop・damp・status・close）。指令は子へ送り、状態は子から届いた最新。

    child: 子のプロセス（Popen）。あれば、その pid の状態だけを受け付け、子が終わっていれば状態は無限に古い。
    fallback_client: 子が居ないときに脱力を送るための LocoClient を作る関数（主のプロセスの DDS は初期化済みのこと）
    """

    def __init__(self, cmd_port: int = CMD_PORT, status_port: int = STATUS_PORT, child=None,
                 fallback_client: Optional[Callable[[], object]] = None) -> None:
        self.cmd_addr = ("127.0.0.1", cmd_port)
        self.child = child
        self.fallback_client = fallback_client
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", status_port))
        self.rx.settimeout(0.2)
        self.status_port = self.rx.getsockname()[1]
        self.fsm: Optional[int] = None
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[float, dsio.VelStatus, Optional[int], int]] = None  # (受けた時刻, 状態, FSM, pid)
        self._want: Tuple[bool, float, float, float] = (False, 0.0, 0.0, 0.0)
        self._damp = False
        self._fallback_started = self._damp_confirmed = False
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="loco-relay-status", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._closed:
            try:
                b = self.rx.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            got = unpack_status(b)
            if got is None:
                continue
            _, st, fsm, pid = got
            with self._lock:
                self._latest = (time.monotonic(), st, fsm, pid)

    def _send(self, kind: str) -> None:
        walk, vx, vy, wz = self._want
        try:
            self.tx.sendto(pack_cmd(kind, walk, (vx, vy, wz), self._damp), self.cmd_addr)
        except OSError:
            pass  # 子が居なくなった。主の輪は状態の古さで気づく

    def _ours(self) -> Optional[Tuple[float, dsio.VelStatus, Optional[int], int]]:
        """自分の子からの最新の状態（子が終わっていれば None）"""
        with self._lock:
            latest = self._latest
        if self.child is not None and (self.child.poll() is not None or latest is None or latest[3] != self.child.pid):
            return None
        return latest

    def wait_ready(self, timeout_s: float) -> bool:
        """自分の子から最初の状態が届くまで待つ（届けば FSM も読める）。子が先に終われば False"""
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            if self.child is not None and self.child.poll() is not None:
                return False
            latest = self._ours()
            if latest is not None:
                self.fsm = latest[2]
                return True
            time.sleep(0.02)
        return False

    def start(self) -> None:
        """子は起こした時から回っている（VelocitySender の口に合わせるだけ）"""

    def set(self, walk: bool, vx: float, vy: float, wz: float) -> None:
        if self._damp:
            return  # 脱力の後は歩かない（VelocitySender と同じ）
        self._want = (True, vx, vy, wz) if walk else (False, 0.0, 0.0, 0.0)
        self._send("set")

    def stop(self) -> None:
        self._want = (False, 0.0, 0.0, 0.0)
        self._send("stop")

    def damp(self) -> None:
        self._want, self._damp = (False, 0.0, 0.0, 0.0), True
        self._send("damp")
        self._damp_if_child_gone()

    def status(self) -> dsio.VelStatus:
        """最新の状態（古さつき）。呼ぶたびに子へ今の望み（state）も送る（主の輪は毎周期これを呼ぶ）"""
        self._send("state")
        latest = self._ours()
        if self._damp:
            self._damp_if_child_gone(latest)
        if latest is None:
            return NO_STATUS
        t, st, _, _ = latest
        return dsio.VelStatus(st.sent, st.failed_in_row, st.last_code, st.last_rpc_ms, st.want, st.damped, st.last_error,
                              time.monotonic() - t)

    def _damp_if_child_gone(self, latest=None) -> None:
        """脱力を求めた後に呼ぶ。子が脱力を返していなくて、子が居ない（終わった・状態が DEAD_S 来ない）なら自分で 1 回送る"""
        if self._fallback_started or self._damp_confirmed or self.fallback_client is None:
            return
        latest = self._ours() if latest is None else latest
        if latest is not None and latest[1].damped:
            self._damp_confirmed = True  # 子が送った。後で子が終わっても二重には送らない
            return
        if latest is not None and time.monotonic() - latest[0] <= DEAD_S:
            return  # 子は生きている。state で届く
        self._fallback_started = True
        threading.Thread(target=self._fallback_damp, name="loco-relay-damp", daemon=True).start()  # 輪を待たせない

    def _fallback_damp(self) -> None:
        print("[loco_relay] 子が居ないので、主のプロセスから Damp を送る", flush=True)
        try:
            self.fallback_client().Damp()
        except Exception as e:  # noqa: BLE001  送れなければ知らせるだけ（リモコンの L2+B は純正のもの）
            print(f"[loco_relay] 主のプロセスからの Damp に失敗: {type(e).__name__}: {e}", flush=True)

    def close(self, timeout_s: float = 2.0) -> None:
        """子に止める指令を送り切らせてから終わらせる。終わらなければ SIGTERM、それでも終わらなければ強く止める"""
        self._send("quit")
        if self.child is not None:
            try:
                self.child.wait(timeout=timeout_s)
            except Exception:  # noqa: BLE001  終わらなければ TERM（子は TERM でも止めてから終わる）
                self.child.terminate()
                try:
                    self.child.wait(timeout=timeout_s)
                except Exception:  # noqa: BLE001
                    self.child.kill()
        self._closed = True
        self.tx.close()
        self.rx.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--cmd-port", type=int, default=CMD_PORT)
    ap.add_argument("--status-port", type=int, default=STATUS_PORT)
    ap.add_argument("--rpc-timeout", type=float, default=1.0, help="[s] SetVelocity の返事を待つ上限")
    ap.add_argument("--parent-pid", type=int, default=0, help="主のプロセスの pid（0 なら起こされた時の親）")
    a = ap.parse_args(argv)
    parent = a.parent_pid or os.getppid()  # 初期化（数秒）より前に取る。初期化の間に親が落ちても気づける
    if os.getppid() != parent:
        print(f"[loco_relay] 親 {parent} がもう居ない: 起こさない", flush=True)
        return 1

    stop = threading.Event()
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # 主の側が立ったまま止める・脱力を送るのに、子が要る
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", a.cmd_port))  # 居残った前の子がポートを持っていれば、ここで落ちる（主の側は起動を断る）
    ChannelFactoryInitialize(0, a.iface)
    client = dsio.make_loco_client(a.rpc_timeout)
    try:
        fsm = dsio.fsm_id(client)
    except Exception as e:  # noqa: BLE001  読めなければ None を返し、主の側が起動を断る
        print(f"[loco_relay] FSM を読めない: {type(e).__name__}: {e}", flush=True)
        fsm = None
    sender = dsio.VelocitySender(client)  # 糸は serve が最初の指令で起こす
    print(f"[loco_relay] SetVelocity をこのプロセスで送る（pid {os.getpid()}・待ち {a.rpc_timeout} s・FSM {fsm}）・"
          f"指令 127.0.0.1:{a.cmd_port}・状態 → 127.0.0.1:{a.status_port}・親 {parent}", flush=True)
    reason = serve(sender, sock, ("127.0.0.1", a.status_port), fsm, lambda: os.getppid() == parent, stop=stop)
    sender.close()
    st = sender.status()
    print(f"[loco_relay] 終わる（{reason}）: 送った {st.sent} 回・最後の番号 {st.last_code}", flush=True)
    sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
