"""11-04: loco_relay.py（SetVelocity を別のプロセスで送る中継）の試験。

子のプロセスの中身（loco_relay.serve）は糸で回し、偽の LocoClient（test_door_run_stock.FakeLoco）をつなぐ。
主の側（LocoRelay）との間は本物の UDP（127.0.0.1・空いているポート）。
"""
from __future__ import annotations

import importlib.util
import math
import os
import signal
import socket
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_stock_io as dsio  # noqa: E402
import loco_relay as lr  # noqa: E402
from test_door_run_stock import FakeLoco, _phase_time, _run, needs_policy  # noqa: E402


class Rig:
    """子の serve を糸で回し、主の側の LocoRelay とつなぐ（子のプロセスの main と同じく、送り手は serve が起こし、serve の後に閉じる）"""

    def __init__(self, loco: FakeLoco, fsm=501, parent_alive=lambda: True) -> None:
        self.loco = loco
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.relay = lr.LocoRelay(cmd_port=self.sock.getsockname()[1], status_port=0)
        self.sender = dsio.VelocitySender(loco)
        self.quit = threading.Event()
        self.result = []
        self.thread = threading.Thread(target=self._child, args=(fsm, parent_alive), daemon=True)
        self.thread.start()

    def _child(self, fsm, parent_alive) -> None:
        reason = lr.serve(self.sender, self.sock, ("127.0.0.1", self.relay.status_port), fsm, parent_alive, stop=self.quit)
        self.sender.close()
        self.sock.close()
        self.result.append(reason)

    def end(self) -> None:
        self.quit.set()
        self.thread.join(2.0)
        self.relay.close()


def _keep_walking(relay: lr.LocoRelay, seconds: float, v=(0.3, 0.0, 0.1)) -> None:
    """主の輪と同じく 50 回/秒で set と status（今の望みの state）を送る"""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        relay.set(True, *v)
        relay.status()
        time.sleep(0.02)


# ---- 電文 ----

def test_command_packets_roundtrip_and_reject_garbage():
    # Arrange / Act / Assert
    assert lr.unpack_cmd(lr.pack_cmd("set", True, (0.3, -0.1, 0.2))) == ("set", True, (0.3, -0.1, 0.2), False)
    assert lr.unpack_cmd(lr.pack_cmd("state", False, (0.0, 0.0, 0.0), True)) == ("state", False, (0.0, 0.0, 0.0), True)
    assert lr.unpack_cmd(lr.pack_cmd("stop")) == ("stop", False, (0.0, 0.0, 0.0), False)
    assert lr.unpack_cmd(b"\xff\x00") is None
    assert lr.unpack_cmd(b'{"k": "walk", "w": true, "v": [0, 0, 0], "d": false}') is None  # 知らない種類
    assert lr.unpack_cmd(b'{"k": "set", "w": true, "v": [NaN, 0, 0], "d": false}') is None  # 壊れた速さは歩かせない
    assert lr.unpack_cmd(b'{"k": "set", "w": true, "v": [0.1, 0], "d": false}') is None


def test_status_packets_roundtrip():
    # Arrange
    st = dsio.VelStatus(sent=5, failed_in_row=1, last_code=3104, last_rpc_ms=251.0, want=(True, 0.3, 0.0, 0.1),
                        damped=False, last_error="")

    # Act
    got = lr.unpack_status(lr.pack_status(st, 501, 12.5, 4321))

    # Assert
    assert got == (12.5, st, 501, 4321)
    assert lr.unpack_status(lr.pack_status(dsio.VelStatus(0, 0, None, 0.0, (False, 0.0, 0.0, 0.0), False, ""), None, 1.0, 1))[2] is None
    assert lr.unpack_status(b"{}") is None


# ---- 子（serve）と主の側（LocoRelay） ----

def test_relay_walks_then_stops_like_inprocess_sender():
    # Arrange
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0) and rig.relay.fsm == 501

        # Act: 0.32 秒歩いてから、歩くのをやめる
        _keep_walking(rig.relay, 0.32)
        walking = rig.loco.vel()
        rig.relay.set(False, 0.0, 0.0, 0.0)
        for _ in range(20):
            rig.relay.status()
            time.sleep(0.02)
        st = rig.relay.status()

        # Assert: 同じプロセスの送り手と同じ送り方（20 回/秒・やめたら速度 0 を 3 回）
        assert 5 <= len(walking) <= 8
        assert all(c[2] == (0.3, 0.0, 0.1, dsio.VEL_DURATION_S) for c in walking)
        after = rig.loco.vel()[len(walking):]
        assert [c[2] for c in after] == [(0.0, 0.0, 0.0, dsio.VEL_DURATION_S)] * dsio.STOP_REPEATS
        assert st.sent == len(rig.loco.vel()) and st.failed_in_row == 0 and st.last_code == 0
        assert st.want == (False, 0.0, 0.0, 0.0) and st.age_s < 0.1
    finally:
        rig.end()


def test_relay_reports_failures_from_child():
    # Arrange: 子の RPC が 3104 を返す
    rig = Rig(FakeLoco(code=3104))
    try:
        assert rig.relay.wait_ready(2.0)

        # Act
        _keep_walking(rig.relay, 0.35)
        st = rig.relay.status()

        # Assert: 続けて失敗した数と番号が主の側に届く（主の輪はこれで立ったまま止まる）
        assert st.failed_in_row >= 5 and st.last_code == 3104
    finally:
        rig.end()


def test_relay_damp_sends_damp_once():
    # Arrange
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0)

        # Act
        rig.relay.damp()
        for _ in range(10):
            rig.relay.set(True, 0.3, 0.0, 0.0)  # 脱力の後の歩く指令は効かない
            rig.relay.status()
            time.sleep(0.02)

        # Assert
        assert len(rig.loco.damps()) == 1 and rig.loco.vel() == []
        assert rig.relay.status().damped
    finally:
        rig.end()


def test_child_stops_walking_when_main_falls_silent():
    """主の輪が固まったら（電文が PING_TIMEOUT_S 来ない）子が自分で速度 0 を送る。同じプロセスなら輪と一緒に止まっていた所"""
    # Arrange
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0)
        _keep_walking(rig.relay, 0.2)
        t_silent = time.monotonic()

        # Act: 何も送らない
        time.sleep(0.7)

        # Assert
        zeros = [c for c in rig.loco.vel() if c[2][:3] == (0.0, 0.0, 0.0)]
        assert len(zeros) == dsio.STOP_REPEATS
        assert lr.PING_TIMEOUT_S - 0.05 < zeros[0][1] - t_silent < lr.PING_TIMEOUT_S + 0.15
        assert not [c for c in rig.loco.vel() if c[1] > zeros[0][1] and c[2][:3] != (0.0, 0.0, 0.0)]
    finally:
        rig.end()


def test_child_ends_and_stops_when_parent_is_gone():
    # Arrange: 0.3 秒後に親が居なくなる
    t_gone = time.monotonic() + 0.3
    rig = Rig(FakeLoco(), parent_alive=lambda: time.monotonic() < t_gone)
    try:
        assert rig.relay.wait_ready(2.0)

        # Act
        _keep_walking(rig.relay, 0.6 + lr.PARENT_CHECK_S)
        rig.thread.join(2.0)

        # Assert: 親を見失って終わり、送り手を閉じて速度 0 を送った
        assert rig.result and "親" in rig.result[0]
        assert rig.loco.vel()[-1][2][:3] == (0.0, 0.0, 0.0)
    finally:
        rig.end()


def test_quit_from_main_ends_child_after_stopping():
    # Arrange
    rig = Rig(FakeLoco())
    assert rig.relay.wait_ready(2.0)
    _keep_walking(rig.relay, 0.2)

    # Act: 主の側の close（door_run_stock.py の finally）
    rig.relay.close()
    rig.thread.join(2.0)

    # Assert
    assert rig.result == ["主の側から終わりの合図"]
    assert [c[2][:3] for c in rig.loco.vel()[-dsio.STOP_REPEATS:]] == [(0.0, 0.0, 0.0)] * dsio.STOP_REPEATS


def test_status_age_grows_when_child_is_gone():
    # Arrange
    rig = Rig(FakeLoco())
    assert rig.relay.wait_ready(2.0)

    # Act: 子だけが終わる
    rig.quit.set()
    rig.thread.join(2.0)
    time.sleep(0.4)

    # Assert: 主の側は状態の古さで気づける
    assert rig.relay.status().age_s > 0.3
    rig.relay.close()


def test_wait_ready_reports_unreadable_fsm():
    # Arrange: 子が FSM を読めなかった
    rig = Rig(FakeLoco(), fsm=None)
    try:
        # Act / Assert: 状態は届く（起動はできた）が FSM は None
        assert rig.relay.wait_ready(2.0) and rig.relay.fsm is None
    finally:
        rig.end()


# ---- 1 発の取りこぼし・子が居ない・居残った子（11-04 の見直しで足した） ----

def _drop(relay: lr.LocoRelay, kinds) -> None:
    """主の側の電文のうち、kinds の種類だけを落とす（UDP の取りこぼしの模擬）"""
    send = relay._send
    relay._send = lambda kind: None if kind in kinds else send(kind)


def test_lost_stop_packet_still_stops_by_next_state():
    # Arrange
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0)
        _keep_walking(rig.relay, 0.2)
        _drop(rig.relay, ("stop",))

        # Act: 止める 1 発が落ちる。毎周期の state（歩かない）だけが届く
        rig.relay.stop()
        t_stop = time.monotonic()
        for _ in range(10):
            rig.relay.status()
            time.sleep(0.02)

        # Assert
        zeros = [c for c in rig.loco.vel() if c[2][:3] == (0.0, 0.0, 0.0)]
        assert len(zeros) == dsio.STOP_REPEATS and zeros[0][1] - t_stop < 0.1
        assert not rig.relay.status().want[0]
    finally:
        rig.end()


def test_lost_damp_packet_still_damps_by_next_state():
    # Arrange
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0)
        _drop(rig.relay, ("damp",))

        # Act
        rig.relay.damp()
        for _ in range(10):
            rig.relay.status()
            time.sleep(0.02)

        # Assert: state の脱力を何度受けても、送るのは 1 回
        assert len(rig.loco.damps()) == 1 and rig.relay.status().damped
    finally:
        rig.end()


def test_damp_is_sent_from_main_when_child_is_gone():
    # Arrange: 子が居なくなった後に脱力を求める（傾きで DAMP に入った等）
    own = FakeLoco()
    rig = Rig(FakeLoco())
    rig.relay.fallback_client = lambda: own
    assert rig.relay.wait_ready(2.0)
    rig.quit.set()
    rig.thread.join(2.0)
    time.sleep(lr.DEAD_S + 0.1)

    # Act
    rig.relay.damp()
    for _ in range(10):
        rig.relay.status()
        time.sleep(0.02)

    # Assert: 主のプロセスの口から 1 回だけ
    assert len(own.damps()) == 1 and rig.loco.damps() == []
    rig.relay.close()


def test_damp_is_not_sent_twice_when_child_already_damped():
    # Arrange
    own = FakeLoco()
    rig = Rig(FakeLoco())
    rig.relay.fallback_client = lambda: own
    assert rig.relay.wait_ready(2.0)

    # Act: 子が脱力を送った後で、子が居なくなる
    rig.relay.damp()
    for _ in range(10):
        rig.relay.status()
        time.sleep(0.02)
    rig.quit.set()
    rig.thread.join(2.0)
    time.sleep(lr.DEAD_S + 0.1)
    for _ in range(5):
        rig.relay.status()
        time.sleep(0.02)

    # Assert
    assert len(rig.loco.damps()) == 1 and own.damps() == []
    rig.relay.close()


class FakeChild:
    """subprocess.Popen の代わり（pid と終わったかだけ）"""

    def __init__(self, pid: int, code=None) -> None:
        self.pid, self.code = pid, code

    def poll(self):
        return self.code

    def wait(self, timeout=None):
        return self.code

    def terminate(self) -> None:
        pass

    def kill(self) -> None:
        pass


def test_status_from_another_process_is_ignored():
    """居残った前の子（pid が違う）の状態では、準備できたことにしない"""
    rig = Rig(FakeLoco())
    try:
        # Arrange
        rig.relay.child = FakeChild(pid=-1)

        # Act / Assert
        assert not rig.relay.wait_ready(0.5)
        assert rig.relay.status().age_s == math.inf
    finally:
        rig.relay.child = None
        rig.end()


def test_finished_child_reads_as_infinitely_stale():
    rig = Rig(FakeLoco())
    try:
        assert rig.relay.wait_ready(2.0)

        # Act: 状態の電文は届き続けているが、子のプロセスはもう終わっている
        rig.relay.child = FakeChild(pid=os.getpid(), code=0)

        # Assert
        assert rig.relay.status().age_s == math.inf
    finally:
        rig.relay.child = None
        rig.end()


# ---- 本物の子のプロセス（機体なし・ループバック。SDK が無ければ飛ばす） ----

needs_sdk = pytest.mark.skipif(importlib.util.find_spec("unitree_sdk2py") is None, reason="unitree_sdk2py が無い")


def _loopback() -> str:
    return "lo0" if sys.platform == "darwin" else "lo"


@needs_sdk
def test_real_child_ignores_ctrl_c_and_ends_on_quit():
    import door_run_stock as drs

    # Arrange
    relay, fsm = drs.start_loco_relay(_loopback(), 0.2)
    assert relay is not None and fsm is None  # 機体が無いので FSM は読めない（主の側は起動を断る）
    try:
        # Act: Ctrl+C と ssh の切断の信号
        os.kill(relay.child.pid, signal.SIGINT)
        os.kill(relay.child.pid, signal.SIGHUP)
        time.sleep(0.3)

        # Assert: 子は終わらない
        assert relay.child.poll() is None and relay.status().age_s < 0.2
    finally:
        relay.close()
    assert relay.child.returncode == 0


@needs_sdk
def test_real_child_ends_when_parent_is_killed(tmp_path):
    # Arrange: 中継を起こした親を強く殺す（SIGKILL）
    pid_file = tmp_path / "child.pid"
    script = tmp_path / "parent.py"
    script.write_text(textwrap.dedent(f"""
        import sys, time
        sys.path.insert(0, {str(HERE)!r})
        import door_run_stock as drs
        relay, _ = drs.start_loco_relay({_loopback()!r}, 0.2)
        open({str(pid_file)!r}, "w").write(str(relay.child.pid))
        time.sleep(30)
    """))
    parent = subprocess.Popen([sys.executable, str(script)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t_end = time.monotonic() + 10.0
    while not pid_file.exists() or not pid_file.read_text():
        assert time.monotonic() < t_end and parent.poll() is None
        time.sleep(0.05)
    child_pid = int(pid_file.read_text())

    # Act
    parent.kill()
    parent.wait()

    # Assert: 子は親の見張り（0.5 s）で気づき、止める指令を送ってから終わる
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end:
        try:
            os.kill(child_pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)
    os.kill(child_pid, signal.SIGKILL)
    pytest.fail("親が殺された後も子が残った")


# ---- 主の関数（door_run_stock.main --loco relay） ----

def _patch_relay(monkeypatch, rig: Rig, calls: list) -> None:
    import door_run_stock as drs

    def fake_start(iface, rpc_timeout):
        calls.append((iface, rpc_timeout))
        return (rig.relay, rig.relay.fsm) if rig.relay.wait_ready(2.0) else (None, None)

    monkeypatch.setattr(drs, "start_loco_relay", fake_start)


@needs_policy
def test_main_walks_through_relay(monkeypatch, tmp_path):
    """--loco relay では主のプロセスに LocoClient を作らず、歩きの指令は中継の子へ行く"""
    # Arrange
    rig = Rig(FakeLoco())
    calls = []
    _patch_relay(monkeypatch, rig, calls)

    # Act
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.8, "r"), (8.2, "q")],
                                               extra_args=("--loco", "relay", "--rpc-timeout", "0.25"))
    rig.thread.join(2.0)

    # Assert
    assert rc == 0 and seen == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "WAIT"]
    assert calls == [("eth0", 0.25)] and loco.calls == []  # 主のプロセスの LocoClient は使わない
    t_door, t_post = _phase_time(z, phases, "DOOR"), _phase_time(z, phases, "POST")
    assert [c for c in rig.loco.vel() if t_door < c[1] < t_post]  # 方策の間は子が送った
    late = [c for c in rig.loco.vel() if c[1] > t_post + 0.3]
    assert all(c[2][:3] == (0.0, 0.0, 0.0) for c in late)
    assert rig.result == ["主の側から終わりの合図"]  # 終わりに子も閉じた
    door = [k for k, ph in enumerate(phases) if ph == "DOOR"]
    assert np.diff(z["vel_sent"][door]).sum() > 0 and (z["vel_status_age"][door] < 0.1).all()


@needs_policy
def test_main_refuses_when_relay_reports_non_walking_fsm(monkeypatch, tmp_path):
    # Arrange
    rig = Rig(FakeLoco(), fsm=802)
    _patch_relay(monkeypatch, rig, [])

    # Act
    rc, _, _, _, arms, loco, _ = _run(monkeypatch, tmp_path, [(0.3, "a")], extra_args=("--loco", "relay"))
    rig.thread.join(2.0)

    # Assert: 起動しない・腕を借りない・子も閉じた
    assert rc == 1 and arms == [] and rig.loco.calls == []
    assert rig.result == ["主の側から終わりの合図"]


@needs_policy
def test_main_stands_when_relay_goes_silent_in_door(monkeypatch, tmp_path, capsys):
    """方策の途中で子が居なくなったら（状態が古くなったら）、立ったまま止まる"""
    # Arrange
    rig = Rig(FakeLoco())
    _patch_relay(monkeypatch, rig, [])
    threading.Timer(4.6, rig.quit.set).start()  # d（3.8 秒）の後、方策の途中で子だけが終わる

    # Act
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (6.5, "INT"), (6.9, "INT")],
                                               run_seconds="3.0", extra_args=("--loco", "relay"))

    # Assert
    assert seen[:5] == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST"]
    t_post = _phase_time(z, phases, "POST") - t0
    assert 4.6 + 0.4 < t_post < 4.6 + 0.9  # 古さの線 0.5 秒
    assert "DOOR → POST（SetVelocity の中継（loco_relay.py）から状態が" in capsys.readouterr().out
    assert math.isfinite(float(z["vel_status_age"][-1]))


@needs_policy
def test_main_stands_when_relay_dies_before_door(monkeypatch, tmp_path, capsys):
    """READY の間に子が居なくなったら、d を受けずに立ったまま止まる（子が居ないまま歩かせない）"""
    # Arrange
    rig = Rig(FakeLoco())
    _patch_relay(monkeypatch, rig, [])
    threading.Timer(2.8, rig.quit.set).start()

    # Act
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (5.0, "INT"), (5.4, "INT")],
                                               extra_args=("--loco", "relay"))

    # Assert
    assert seen[:4] == ["WAIT", "ARMS_IN", "READY", "POST"] and "DOOR" not in seen
    assert "READY → POST（SetVelocity の中継（loco_relay.py）から状態が" in capsys.readouterr().out
