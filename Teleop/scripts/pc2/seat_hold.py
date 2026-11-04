#!/usr/bin/env python3
"""2026-11-04: 座位の遠隔操作・再生の合間に、最後の姿勢を保つ保持係（PC2 で常駐する）。

座位（teleop_start.sh の TELEOP_LEGS=seated・pc2_teleop.sh の --seated）は純正を止めて、遠隔操作か再生が
rt/lowcmd で全身を指令する。その送り手が終わると指令が途切れてモータが無効（ゼロトルク）に落ち、腕が垂れ、
次の回の前にリモコンでダンピングに入れ直す必要があった。保持係は rt/lowcmd を見張り、ほかの送り手が
GAP_S 黙ったら、その最後の指令をそのまま送り続ける。ほかの送り手が送り始めたら、すぐに黙る。
遠隔操作・再生の作りは変えない（保持係の指令には印 MARK を付けて、見張りで自分の指令を外す）。

    python3 seat_hold.py ensure [--iface eth0]   # 動いていなければ切り離して起こす（teleop_start.sh・pc2_teleop.sh が呼ぶ）
    python3 seat_hold.py run [--iface eth0]      # 手前で動かす（ログは画面）
    python3 seat_hold.py status                  # 今の状態
    python3 seat_hold.py release                 # 保っていれば力を 2 秒で抜いてから止める（その後はゼロトルク）

引き継がない指令（黙ったまま）:
  - 純正が動いている（裏の糸が STOCK_POLL_S ごとに歩きの RPC の GetFsmId を聞き、STOCK_SILENT_POLLS 回続けて答えが
    無いときだけ「止まっている」とする。起動の直後は「動いている」として扱う）
  - 送る速さが FOREIGN_HZ_RANGE の外か、数えた件数が MIN_SAMPLES 未満（遠隔操作は 250・再生は 50。純正の制御は 1000 前後）
  - kp が全部 0（純正のダンピングなど。保つ姿勢が無い）
  - モータが無効の関節がある（後でリモコンで有効にされた瞬間に、古い姿勢へ跳ねる）
  - 実測の角度から MAX_DEV_RAD より離れた関節がある（引き継いだ瞬間に跳ねる）
保っている間に止めるとき:
  - rt/lowstate が STATE_STALE_S 途切れた・モータが無効になった・純正が答えた → すぐに送るのをやめる（力を抜かない）
  - モータの温度が TEMP_LIMIT_C 以上・MAX_HOLD_S 保った・release → RELEASE_S で kp を 0 へ下げてからやめる
保持係は 1 つだけ（LOCK_FILE の排他の鍵。2 つ目は起きずに終わる）。
⚠️ 送るのをやめた後の機体の振る舞い（何秒でゼロトルクに落ちるか）はまだ測っていない。ログの「モータの有効」の行が測った値になる。

PC2（Orin NX）の Python の DDS は重い（11-04 実測: LowState_ を戻すのに 0.81 ms・LowCmd_ は 0.56 ms で OMEN の約 5 倍。
機体の rt/lowstate 647 件/秒と、ダンピング中の純正の rt/lowcmd 609 件/秒を聞くだけで 1 コアを使い切る）。そこで:
  - rt/lowstate は聞き役を付けず、最新の 1 件だけ残す読み手から STATE_POLL_EVERY 周期に 1 回（50 Hz）取りに行く
  - rt/lowcmd の読み手は IgnoreLocal.Process で自分の指令を受けない（印 MARK の見分けは二重の守りとして残す）
  - 保っている間は同じ指令なので、詰め直しと CRC は指令が変わったときだけ
  - 終わるときは後片付けの後に os._exit で抜ける（PC2 で、止める指示の後に Python の後始末で 5 秒以上止まらなかった）
"""

import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Deque, List, Optional

import numpy as np

N_MOTORS = 35                 # unitree_hg の motor_cmd の数
N_BODY = 29                   # 0..28 が実関節
MARK = 0x53454154             # "SEAT"。保持係の指令の印（LowCmd_.reserve[3]）
SEND_HZ = 250.0               # xr_teleoperate の腕の制御と同じ
GAP_S = 0.03                  # ほかの送り手がこれだけ黙ったら引き継ぐ（再生は 50 Hz＝0.02 秒ごとに送る）
STATE_STALE_S = 0.1           # Door/real/door_run_real.py と同じ
STATE_POLL_EVERY = 5          # rt/lowstate を取りに行くのは 5 周期に 1 回（250/5＝50 Hz。途切れの線 0.1 秒より十分細かい）
CMD_TAKE_N = 8                # rt/lowcmd の読み手に残す件数（聞き役が遅れても速さを数え落とさない）
MAX_DEV_RAD = 0.3
FOREIGN_HZ_RANGE = (20.0, 400.0)
MIN_SAMPLES = 10
STOCK_POLL_S = 0.5
STOCK_SILENT_POLLS = 2
STOCK_RPC_TIMEOUT_S = 0.3
TEMP_LIMIT_C = 70             # 11-03 に左肩 roll が 84 °C まで上がった（胴への押し当て）
MAX_HOLD_S = 3600.0
RELEASE_S = 2.0
STATUS_EVERY_S = 0.5
START_WAIT_S = 15.0           # ensure が起動を待つ上限（PC2 は DDS の立ち上がりが遅いことがある）
LOG_DIR = Path.home() / "teleop_logs"
STATUS_FILE = LOG_DIR / "seat_hold.status.json"
PID_FILE = LOG_DIR / "seat_hold.pid"
LOCK_FILE = LOG_DIR / "seat_hold.lock"


@dataclass(frozen=True)
class Cmd:
    """rt/lowcmd の 1 件（全 35 モータ）。配列は書き換えない。"""
    mode_pr: int
    mode_machine: int
    mode: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray

    def held_joints(self) -> np.ndarray:
        """保つ（kp が 0 より大きい）実関節の番号。"""
        return np.flatnonzero(self.kp[:N_BODY] > 0)

    def with_kp_scale(self, s: float) -> "Cmd":
        return replace(self, kp=self.kp * float(s))


@dataclass(frozen=True)
class Body:
    """rt/lowstate のうち保持係が見る所（実関節 29）。"""
    q: np.ndarray
    mode: np.ndarray
    temp: np.ndarray


def refusal(cmd: Cmd, body: Optional[Body], foreign_hz: float, samples: int, stock_running: bool,
            max_dev: float = MAX_DEV_RAD, temp_limit: float = TEMP_LIMIT_C,
            hz_range: tuple = FOREIGN_HZ_RANGE) -> Optional[str]:
    """その指令を引き継がない理由（引き継いでよければ None）。"""
    if stock_running:
        return "純正が動いている（歩きの RPC が答えている・まだ確かめていない）"
    if samples < MIN_SAMPLES:
        return f"送り手の指令を {samples} 件しか見ていない（速さを決められない）"
    if not hz_range[0] <= foreign_hz <= hz_range[1]:
        return f"1 秒に {foreign_hz:.0f} 件の送り手（遠隔操作・再生の速さ {hz_range[0]:g}〜{hz_range[1]:g} の外。純正の制御は 1000 前後）"
    joints = cmd.held_joints()
    if len(joints) == 0:
        return "kp が全部 0（保つ姿勢が無い）"
    if body is None:
        return "機体の状態（rt/lowstate）が来ていない"
    if not (np.all(np.isfinite(cmd.q[joints])) and np.all(np.isfinite(cmd.kp[joints]))):
        return "指令の値が壊れている"
    off = [int(j) for j in joints if body.mode[j] != 1]
    if off:
        return f"モータが無効の関節がある（{off[:6]}）"
    dev = np.abs(cmd.q[joints] - body.q[joints])
    if dev.max() > max_dev:
        return f"指令と実測が {dev.max():.2f} rad 離れている（関節 {int(joints[dev.argmax()])}）"
    if body.temp[joints].max() >= temp_limit:
        return f"モータの温度が {int(body.temp[joints].max())} °C"
    return None


class HoldCore:
    """保持係の判断。DDS に触らない（試験はこれを直に回す）。時刻は time.monotonic() の秒。

    状態: idle（持っていない）・yield（ほかが送っている）・hold（保っている）・release（力を抜いている）・done（終わり）
    """

    def __init__(self, convert: Callable[[Any], Cmd] = lambda c: c, gap_s: float = GAP_S,
                 max_dev: float = MAX_DEV_RAD, temp_limit: float = TEMP_LIMIT_C,
                 max_hold_s: float = MAX_HOLD_S, hz_range: tuple = FOREIGN_HZ_RANGE) -> None:
        self.convert = convert
        self.gap_s, self.max_dev, self.temp_limit = gap_s, max_dev, temp_limit
        self.max_hold_s, self.hz_range = max_hold_s, hz_range
        self.state, self.reason = "idle", "起動した"
        self.stock_running = True        # 確かめるまでは「純正が動いている」として扱う
        self.stock_silent = 0            # 歩きの RPC に続けて答えが無かった回数
        self.foreign: Any = None
        self.foreign_t: Optional[float] = None
        self.foreign_times: Deque[float] = deque(maxlen=50)
        self.considered = True           # 今の foreign を引き継ぐか決めたか（次の foreign まで決め直さない）
        self.held: Optional[Cmd] = None
        self.hold_since: Optional[float] = None
        self.release_since: Optional[float] = None
        self.exit_after_release = False
        self.body: Optional[Body] = None
        self.body_t: Optional[float] = None
        self.enabled: Optional[int] = None
        self.events: List[str] = []

    # ---- 入力 ----
    def on_foreign(self, raw: Any, t: float) -> None:
        """ほかの送り手の rt/lowcmd（保持係の印のある指令は渡さない）。"""
        self.foreign, self.foreign_t, self.considered = raw, t, False
        self.foreign_times.append(t)

    def on_body(self, body: Body, t: float) -> None:
        self.body, self.body_t = body, t
        n = int(np.sum(body.mode[:N_BODY] == 1))
        if n != self.enabled:
            quiet = f"・ほかの送り手が黙ってから {t - self.foreign_t:.3f} 秒" if self.foreign_t is not None else ""
            self._event(f"モータの有効 {self.enabled if self.enabled is not None else '?'}→{n}/{N_BODY}（{self.state}{quiet}）")
            self.enabled = n

    def on_stock(self, answered: bool, t: float) -> None:
        """裏の糸が聞いた歩きの RPC（GetFsmId）の答え。STOCK_SILENT_POLLS 回続けて無ければ「止まっている」。"""
        self.stock_silent = 0 if answered else self.stock_silent + 1
        running = answered or self.stock_silent < STOCK_SILENT_POLLS
        if running != self.stock_running:
            self._event("純正が答えた（動いている）" if running else
                        f"純正が {STOCK_SILENT_POLLS} 回続けて答えない（止まっている）")
            self.stock_running = running

    def request_release(self, t: float, why: str, then_exit: bool = False) -> None:
        self.exit_after_release = self.exit_after_release or then_exit
        if self.state == "hold":
            self.release_since = t
            self._go("release", f"{why}。{RELEASE_S:g} 秒で力を抜く")
        elif self.state != "release":
            self._go("done" if self.exit_after_release else self.state, why)

    # ---- 1 周期 ----
    def foreign_hz(self) -> float:
        ts = self.foreign_times
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 2 and ts[-1] > ts[0] else 0.0

    def tick(self, t: float) -> Optional[Cmd]:
        """この周期に送る指令（送らなければ None）。"""
        if self.state == "done":
            return None
        # ほかの送り手が送っていれば、力を抜く途中でもすぐに黙る（2 つの指令を取り合わない）
        if self.foreign_t is not None and t - self.foreign_t <= self.gap_s:
            if self.exit_after_release:
                self._drop("done", "ほかの送り手が送り始めたので、黙って終わる")
            elif self.state != "yield":
                self._drop("yield", "ほかの送り手が送っている")
            return None
        stale = self.body_t is None or t - self.body_t > STATE_STALE_S
        if self.state == "release":
            s = 1.0 - (t - self.release_since) / RELEASE_S
            if stale or s <= 0.0 or self.stock_running:
                why = ("rt/lowstate が途切れたので、力を抜く途中でやめた" if stale else
                       "純正が答えたので、力を抜く途中でやめた" if self.stock_running else "力を抜き終えた")
                self._drop("done" if self.exit_after_release else "idle", why)
                return None
            return self.held.with_kp_scale(s)
        if self.state == "hold":
            return self._keep(t, stale)
        if not self.considered:
            self.considered = True
            return self._take_over(t, stale)
        if self.state == "yield":
            self._go("idle", "ほかの送り手が黙った")
        return None

    def _take_over(self, t: float, stale: bool) -> Optional[Cmd]:
        cmd = self.convert(self.foreign)
        why = refusal(cmd, None if stale else self.body, self.foreign_hz(), len(self.foreign_times),
                      self.stock_running, self.max_dev, self.temp_limit, self.hz_range)
        if why is not None:
            self._go("idle", f"引き継がない: {why}")
            return None
        self.held, self.hold_since = cmd, t
        self._go("hold", f"ほかの送り手が {self.gap_s * 1000:.0f} ms 黙ったので、最後の指令を保つ"
                         f"（{len(cmd.held_joints())} 関節）")
        return cmd

    def _keep(self, t: float, stale: bool) -> Optional[Cmd]:
        if stale:
            self._drop("idle", f"rt/lowstate が {STATE_STALE_S:g} 秒途切れたので、すぐに送るのをやめた")
            return None
        if self.stock_running:
            self._drop("idle", "純正が答えた（動き出した）ので、すぐに送るのをやめた")
            return None
        joints = self.held.held_joints()
        off = [int(j) for j in joints if self.body.mode[j] != 1]
        if off:
            self._drop("idle", f"モータが無効になったので送るのをやめた（{off[:6]}）")
            return None
        hot = int(self.body.temp[joints].max())
        if hot >= self.temp_limit:
            self.request_release(t, f"モータの温度が {hot} °C")
        elif t - self.hold_since >= self.max_hold_s:
            self.request_release(t, f"{self.max_hold_s / 60:.0f} 分保った")
        return self.held if self.state == "hold" else self.held.with_kp_scale(1.0)

    # ---- 状態の切り替え ----
    def _drop(self, state: str, why: str) -> None:
        self.held = self.hold_since = self.release_since = None
        self._go(state, why)

    def _go(self, state: str, why: str) -> None:
        if state != self.state or why != self.reason:
            self._event(f"{self.state}→{state}: {why}")
        self.state, self.reason = state, why

    def _event(self, text: str) -> None:
        self.events.append(text)

    def pop_events(self) -> List[str]:
        out, self.events = self.events, []
        return out

    def status(self, t: float) -> dict:
        b = self.body
        return {"state": self.state, "reason": self.reason, "stock_running": self.stock_running,
                "hold_s": round(t - self.hold_since, 1) if self.hold_since is not None else None,
                "foreign_hz": round(self.foreign_hz(), 1),
                "quiet_s": round(t - self.foreign_t, 3) if self.foreign_t is not None else None,
                "enabled": self.enabled,
                "max_temp_c": int(b.temp.max()) if b is not None else None}


# ---- DDS の口（PC2 の tv 環境の unitree_sdk2py） ----

def cmd_from_msg(msg: Any) -> Cmd:
    m = msg.motor_cmd
    arr = lambda name: np.array([getattr(m[i], name) for i in range(N_MOTORS)], dtype=np.float64)  # noqa: E731
    return Cmd(int(msg.mode_pr), int(msg.mode_machine), arr("mode").astype(np.int64),
               arr("q"), arr("dq"), arr("tau"), arr("kp"), arr("kd"))


def fill_msg(msg: Any, cmd: Cmd, crc: Any) -> None:
    msg.mode_pr, msg.mode_machine = cmd.mode_pr, cmd.mode_machine
    for i in range(N_MOTORS):
        c = msg.motor_cmd[i]
        c.mode, c.q, c.dq = int(cmd.mode[i]), float(cmd.q[i]), float(cmd.dq[i])
        c.tau, c.kp, c.kd = float(cmd.tau[i]), float(cmd.kp[i]), float(cmd.kd[i])
    msg.reserve[3] = MARK
    msg.crc = crc.Crc(msg)


def body_from_state(msg: Any) -> Body:
    s = msg.motor_state
    return Body(np.array([s[i].q for i in range(N_BODY)], dtype=np.float64),
                np.array([s[i].mode for i in range(N_BODY)], dtype=np.int64),
                np.array([s[i].temperature[0] for i in range(N_BODY)], dtype=np.int64))


_LOG_STREAM = None   # run() が元の標準出力をここへ取る（SDK の print は捨てる）


def log(text: str) -> None:
    print(f"[seat_hold] {time.strftime('%H:%M:%S')}.{int(time.time() * 1000) % 1000:03d} {text}",
          file=_LOG_STREAM or sys.stdout, flush=True)


def write_status(st: dict) -> None:
    tmp = STATUS_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps({**st, "pid": os.getpid(), "updated": time.time()}, ensure_ascii=False), encoding="utf-8")
    tmp.replace(STATUS_FILE)


def open_readers(domain: int, on_cmd: Callable[[Any], None]) -> tuple:
    """rt/lowcmd（聞き役つき・自分の指令は受けない）と rt/lowstate（最新の 1 件だけ）の読み手。

    (状態の読み手, 手元に残すもの) を返す。後者を捨てると、指令の読み手が回収されて聞き役が止まる。

    SDK の ChannelSubscriber は QoS を選べず、Read(0) は「0 は待たない」でなく永久に待つ（cyclonedds 0.10.2 の
    take_iter が `timeout or duration(weeks=99999)`）。ChannelFactoryInitialize の後に、自分の参加者で直に作る。
    """
    from cyclonedds.core import Listener
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    dp = DomainParticipant(domain)
    cmd_qos = Qos(Policy.History.KeepLast(CMD_TAKE_N), Policy.IgnoreLocal.Process)
    rd_cmd = DataReader(dp, Topic(dp, "rt/lowcmd", LowCmd_), cmd_qos,
                        Listener(on_data_available=lambda r: on_cmd(r)))
    rd_state = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), Qos(Policy.History.KeepLast(1)))
    return rd_state, (dp, rd_cmd)


def run(a: argparse.Namespace) -> int:
    from cyclonedds.internal import InvalidSample
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    global _LOG_STREAM
    # 純正が止まっている間、歩きの RPC を聞くたびに SDK が「[ClientStub] send request error」を print する（1 秒に 2 回）。
    # ログが埋もれるので、保持係のログは元の標準出力へ書き、SDK の print は捨てる（例外は標準エラーに出る）
    _LOG_STREAM, sys.stdout = sys.stdout, open(os.devnull, "w")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # 保持係は 1 つだけ。2 つ目は互いの印の付いた指令を無視して二重に送り、release で片方しか止まらない
    lock_file = open(LOCK_FILE, "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        log(f"もう別の保持係が動いている（PID {running_pid()}）ので起きずに終わる")
        return 1
    ChannelFactoryInitialize(a.domain, a.iface)
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    lock = threading.Lock()
    core = HoldCore(cmd_from_msg, gap_s=a.gap_ms / 1000.0, temp_limit=a.temp_limit, max_hold_s=a.max_hold_min * 60.0)

    def on_cmd(reader: Any) -> None:
        try:
            msgs = reader.take(N=CMD_TAKE_N)
        except Exception:  # noqa: BLE001  終わる間際に読み手が消えていることがある（次の件で読む）
            return
        for msg in msgs:
            if isinstance(msg, InvalidSample) or msg.reserve[3] == MARK:
                continue
            with lock:
                core.on_foreign(msg, time.monotonic())

    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda signum, _f: stop.set())

    def poll_stock(client: Any) -> None:
        """純正が動いているか（歩きの RPC が答えるか）を聞き続ける。答えれば引き継がない・保っていればすぐにやめる。"""
        while not stop.is_set():
            try:
                code, fsm = client.GetFsmId()
                answered = code == 0 and fsm is not None
            except Exception:  # noqa: BLE001  答えないことを「答えが無い」として数える
                answered = False
            with lock:
                core.on_stock(answered, time.monotonic())
            stop.wait(STOCK_POLL_S)

    # DDS の口は全部この糸で作ってから裏の糸を起こす。cyclonedds の型の登録は糸を跨ぐと壊れる
    # （11-04 に OMEN で、裏の糸の LocoClient.Init と rt/lowcmd の購読を同時に作って落ちた）
    rd_state, _readers = open_readers(a.domain, on_cmd)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    loco = LocoClient()
    loco.SetTimeout(STOCK_RPC_TIMEOUT_S)
    loco.Init()
    threading.Thread(target=poll_stock, args=(loco,), daemon=True).start()
    msg, crc = unitree_hg_msg_dds__LowCmd_(), CRC()
    PID_FILE.write_text(str(os.getpid()))     # DDS が立ってから（ensure はこれを見て「起きた」とする）
    log(f"起こした（PID {os.getpid()}・{a.iface}・引き継ぐまでの沈黙 {a.gap_ms:g} ms・温度の線 {a.temp_limit} °C・"
        f"最長 {a.max_hold_min:g} 分）。ほかの送り手（遠隔操作・再生）が止まるのを待つ")
    period, next_t, status_t, asked = 1.0 / SEND_HZ, time.monotonic(), 0.0, False
    n_tick, filled = 0, None                  # filled: msg に詰めてある指令（保っている間は同じものを送り直す）
    try:
        while True:
            now = time.monotonic()
            state = rd_state.take(N=1) if n_tick % STATE_POLL_EVERY == 0 else []
            body = body_from_state(state[0]) if state and not isinstance(state[0], InvalidSample) else None
            n_tick += 1
            with lock:
                if body is not None:
                    core.on_body(body, now)
                if stop.is_set() and not asked:
                    asked = True
                    core.request_release(now, "止める指示（release）", then_exit=True)
                out = core.tick(now)
                events, st = core.pop_events(), core.status(now)
            for e in events:
                log(e)
            if out is not None:
                if out is not filled:
                    fill_msg(msg, out, crc)
                    filled = out
                pub.Write(msg)
            if now - status_t >= STATUS_EVERY_S:
                write_status(st)
                status_t = now
            if st["state"] == "done":
                break
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:      # 大きく遅れたら追いつこうとしない
                next_t = time.monotonic()
    finally:
        write_status({**core.status(time.monotonic()), "state": "done"})
        if running_pid() == os.getpid():
            PID_FILE.unlink()
        log("終わった")
    return 0


# ---- 常駐の出し入れ ----

def running_pid() -> Optional[int]:
    """動いている保持係の PID（無ければ None）。PID の使い回しに当たらないよう、名前も見る。"""
    try:
        pid = int(PID_FILE.read_text().strip())
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace")
    except (OSError, ValueError):
        return None
    return pid if "seat_hold.py" in cmdline else None


def holder_state() -> Optional[str]:
    """動いている保持係の状態（hold など）。動いていない・状態が古い（3 秒より前）なら None。"""
    if running_pid() is None:
        return None
    try:
        st = json.loads(STATUS_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return st.get("state") if time.time() - st.get("updated", 0.0) < 3.0 else None


def ensure(a: argparse.Namespace) -> int:
    pid = running_pid()
    if pid is not None:
        print(f"[seat_hold] 動いている（PID {pid}）")
        return 0
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logf = LOG_DIR / f"seat_hold_{time.strftime('%Y%m%dT%H%M%S')}.log"
    cmd = [sys.executable, "-u", str(Path(__file__).resolve()), "run", "--iface", a.iface, "--domain", str(a.domain),
           "--gap-ms", str(a.gap_ms), "--temp-limit", str(a.temp_limit), "--max-hold-min", str(a.max_hold_min)]
    with open(logf, "w") as f:
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < START_WAIT_S:   # DDS の立ち上がりを待つ（PID のファイルが出るまで）
        time.sleep(0.1)
        if running_pid() == p.pid:
            print(f"[seat_hold] 起こした（PID {p.pid}・ログ {logf}）")
            return 0
        if p.poll() is not None:
            break
    if p.poll() is None:                          # 待ちきれなかった子を残さない（後で 2 つ目と並ばないよう）
        p.kill()
    print(f"[seat_hold] ✗ 起きなかった。ログの最後:\n{logf.read_text()[-1500:]}")
    return 1


def status(_a: argparse.Namespace) -> int:
    pid = running_pid()
    try:
        st = json.loads(STATUS_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        st = {}
    if pid is None:
        print("[seat_hold] 動いていない（座位の合間の姿勢は保たれない）")
        return 1
    names = {"idle": "待っている（保つ指令を持っていない）", "yield": "ほかの送り手が送っている（黙っている）",
             "hold": "姿勢を保っている", "release": "力を抜いている", "done": "終わるところ"}
    age = time.time() - st.get("updated", 0.0)
    print(f"[seat_hold] PID {pid}・{names.get(st.get('state'), st.get('state'))}"
          f"{'・' + str(st['hold_s']) + ' 秒' if st.get('hold_s') is not None else ''}・理由: {st.get('reason')}")
    print(f"            モータの有効 {st.get('enabled')}/{N_BODY}・最高温度 {st.get('max_temp_c')} °C・"
          f"ほかの送り手 {st.get('foreign_hz')} 件/秒"
          + (f"・⚠️ 状態が {age:.0f} 秒更新されていない" if age > 3 else ""))
    return 0


def release(a: argparse.Namespace) -> int:
    pid = running_pid()
    if pid is None:
        print("[seat_hold] 動いていない（止めるものが無い）")
        return 0
    os.kill(pid, signal.SIGTERM)
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.wait:
        if running_pid() is None:
            print(f"[seat_hold] 止めた（PID {pid}）。保っていた場合は力を {RELEASE_S:g} 秒で抜いた → 今はゼロトルク")
            return 0
        time.sleep(0.1)
    print(f"[seat_hold] ✗ {a.wait:g} 秒待っても止まらない（PID {pid}）")
    return 1


def main() -> int:
    p = argparse.ArgumentParser(description="座位の合間に最後の姿勢を保つ保持係")
    sub = p.add_subparsers(dest="action", required=True)
    for name in ("run", "ensure"):
        s = sub.add_parser(name)
        s.add_argument("--iface", default="eth0")
        s.add_argument("--domain", type=int, default=0, help="DDS の domain（機体は 0。試験は lo と別の番号で）")
        s.add_argument("--gap-ms", type=float, default=GAP_S * 1000.0, help="ほかの送り手がこれだけ黙ったら引き継ぐ")
        s.add_argument("--temp-limit", type=int, default=TEMP_LIMIT_C, help="この温度 [°C] 以上で力を抜いて止める")
        s.add_argument("--max-hold-min", type=float, default=MAX_HOLD_S / 60.0, help="これだけ保ったら力を抜いて止める")
    sub.add_parser("status")
    r = sub.add_parser("release")
    r.add_argument("--wait", type=float, default=RELEASE_S + 3.0)
    a = p.parse_args()
    if a.action != "run":
        return {"ensure": ensure, "status": status, "release": release}[a.action](a)
    # 状態・PID のファイルとログは run() の finally で片付け済み。Python の後始末（cyclonedds の読み手と聞き役の糸）を
    # 待たずに抜ける。PC2 では止める指示の後に、ここで 5 秒以上止まらないことがあった（11-04）
    try:
        code = run(a)
    except BaseException:  # noqa: BLE001  例外は出してから抜ける（後始末で止まらないよう os._exit で）
        traceback.print_exc()
        code = 1
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    sys.exit(main())
