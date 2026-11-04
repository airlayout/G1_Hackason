"""seat_hold.py（座位の合間の保持係）と form_check.py（形ごとの始めてよいかの判定）の試験（2026-11-04）。
DDS に触らない所（判断の HoldCore と判定の関数）だけ。Mac の Navigation/.venv で:

    G1_Hackason/Navigation/.venv/bin/python -m pytest G1_Hackason/Teleop/scripts/pc2/tests -q
"""

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import form_check as F  # noqa: E402
import seat_hold as S  # noqa: E402

TELEOP_DT = 1 / 250   # 遠隔操作の腕の制御の周期
TICK = 1 / S.SEND_HZ


def make_cmd(q: float = 0.2, kp: float = 80.0) -> S.Cmd:
    n = S.N_MOTORS
    kps = np.zeros(n)
    kps[:S.N_BODY] = kp
    return S.Cmd(0, 1, np.ones(n, dtype=np.int64), np.full(n, q), np.zeros(n), np.zeros(n), kps, np.full(n, 3.0))


def make_body(q: float = 0.2, enabled: bool = True, temp: int = 40) -> S.Body:
    n = S.N_BODY
    return S.Body(np.full(n, q), np.full(n, 1 if enabled else 0, dtype=np.int64), np.full(n, temp, dtype=np.int64))


def make_core(**kw) -> S.HoldCore:
    """純正が止まっていると確かめた保持係（歩きの RPC に STOCK_SILENT_POLLS 回答えが無かった）。"""
    core = S.HoldCore(**kw)
    for _ in range(S.STOCK_SILENT_POLLS):
        core.on_stock(False, 0.0)
    return core


def feed_teleop(core: S.HoldCore, t0: float, seconds: float, cmd: S.Cmd, body: S.Body,
                dt: float = TELEOP_DT) -> float:
    """遠隔操作が dt ごとに送り、保持係が 250 Hz で回る間を流す。終わりの時刻を返す。"""
    t = t0
    while t < t0 + seconds:
        core.on_foreign(cmd, t)
        core.on_body(body, t)
        assert core.tick(t) is None, "ほかの送り手が送っている間は黙る"
        t += dt
    return t


def run_quiet(core: S.HoldCore, t0: float, seconds: float, body: S.Body):
    """ほかの送り手が黙った後を流し、送った指令の並びを返す。"""
    out, t = [], t0
    while t < t0 + seconds:
        core.on_body(body, t)
        out.append(core.tick(t))
        t += TICK
    return out, t


def test_takes_over_last_teleop_command_after_gap():
    # Arrange: 遠隔操作が 1 秒送ってから止まる
    core, cmd, body = make_core(), make_cmd(q=0.2), make_body(q=0.21)
    t = feed_teleop(core, 0.0, 1.0, cmd, body)

    # Act
    out, _ = run_quiet(core, t, 0.2, body)

    # Assert: 黙っている間（30 ms）は送らず、その後は最後の指令をそのまま送る
    first = next(i for i, c in enumerate(out) if c is not None)
    assert first * TICK == pytest.approx(S.GAP_S, abs=2 * TICK)
    assert core.state == "hold"
    assert np.array_equal(out[-1].q, cmd.q) and np.array_equal(out[-1].kp, cmd.kp)


def test_yields_immediately_when_teleop_starts_again():
    # Arrange: 保っている
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)
    assert core.state == "hold"

    # Act: 次の遠隔操作が送り始める
    core.on_foreign(make_cmd(q=0.25), t)
    out = core.tick(t)

    # Assert
    assert out is None and core.state == "yield" and core.held is None


def test_does_not_take_over_high_rate_sender_like_stock_controller():
    # Arrange: 1000 Hz で送る送り手（純正の制御の速さ）
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body, dt=0.001)

    # Act
    out, _ = run_quiet(core, t, 0.2, body)

    # Assert
    assert all(c is None for c in out)
    assert core.state == "idle" and "純正" in core.reason


def test_does_not_take_over_before_confirming_stock_is_stopped():
    # Arrange: 起こした直後（歩きの RPC をまだ聞いていない）
    core, cmd, body = S.HoldCore(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)

    # Act
    out, _ = run_quiet(core, t, 0.2, body)

    # Assert
    assert all(c is None for c in out) and "純正が動いている" in core.reason


def test_does_not_take_over_a_sender_seen_only_briefly():
    # Arrange: 純正の起動の直後などで、数件だけ届いて黙る（速さを決められない。11-04 の見直しの指摘）
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 5 * TELEOP_DT, cmd, body)

    # Act
    out, _ = run_quiet(core, t, 0.2, body)

    # Assert
    assert all(c is None for c in out) and "件しか" in core.reason


def test_stops_at_once_when_stock_answers_while_holding():
    # Arrange: 保っている
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)

    # Act: 純正が戻ってきて、歩きの RPC が答える（restore の後など）
    core.on_stock(True, t)
    out = core.tick(t)

    # Assert: 取り合わずに送るのをやめ、持っていた指令も捨てる
    assert out is None and core.state == "idle" and core.held is None


def test_stock_answering_during_release_stops_sending():
    # Arrange: 力を抜いている途中
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)
    core.request_release(t, "release の指示", then_exit=True)
    _, t = run_quiet(core, t, 0.5, body)

    # Act
    core.on_stock(True, t)

    # Assert
    assert core.tick(t) is None and core.state == "done"


def test_one_silent_poll_is_not_enough_to_call_stock_stopped():
    # Arrange
    core = S.HoldCore()

    # Act: 1 回だけ答えが無かった（散発の失敗かもしれない）
    core.on_stock(False, 0.0)

    # Assert
    assert core.stock_running


@pytest.mark.parametrize("cmd, body, why", [
    (make_cmd(kp=0.0), make_body(), "kp が全部 0"),
    (make_cmd(q=0.2), make_body(q=0.8), "離れている"),
    (make_cmd(), make_body(enabled=False), "無効"),
    (make_cmd(), make_body(temp=75), "温度"),
])
def test_refuses_commands_that_would_jump_or_hold_nothing(cmd, body, why):
    # Arrange
    core = make_core()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)

    # Act
    out, _ = run_quiet(core, t, 0.2, body)

    # Assert
    assert all(c is None for c in out)
    assert core.state == "idle" and why in core.reason


def test_stops_at_once_when_lowstate_goes_stale():
    # Arrange: 保っている
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)

    # Act: rt/lowstate が来なくなる
    outs = []
    while len(outs) < int(0.3 / TICK):
        outs.append(core.tick(t))
        t += TICK

    # Assert: 0.1 秒で送るのをやめ、力を抜かない（ゆっくり下げる指令も送らない）
    sent = [c for c in outs if c is not None]
    assert len(sent) * TICK == pytest.approx(S.STATE_STALE_S, abs=2 * TICK)
    assert all(np.array_equal(c.kp, cmd.kp) for c in sent)
    assert core.state == "idle" and "途切れた" in core.reason


def test_stops_when_motors_become_disabled():
    # Arrange: 保っている
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)

    # Act: モータが無効に落ちる
    core.on_body(make_body(enabled=False), t)
    out = core.tick(t)

    # Assert: 送るのをやめる（後で有効にされた瞬間に古い姿勢へ跳ねないよう、持っていた指令も捨てる）
    assert out is None and core.state == "idle" and core.held is None
    assert any("モータの有効 29→0" in e for e in core.pop_events())


def test_release_ramps_kp_to_zero_then_finishes():
    # Arrange: 保っている
    core, cmd, body = make_core(), make_cmd(kp=80.0), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)

    # Act
    core.request_release(t, "release の指示", then_exit=True)
    outs, t = run_quiet(core, t, S.RELEASE_S + 0.5, body)

    # Assert: kp は 80 から下がり続け、最後は送るのをやめて終わる
    kps = [c.kp[0] for c in outs if c is not None]
    assert kps[0] == pytest.approx(80.0, abs=1.0) and kps[-1] < 1.0
    assert all(a >= b for a, b in zip(kps, kps[1:]))
    assert outs[-1] is None and core.state == "done"


def test_release_while_yielding_finishes_without_sending():
    # Arrange: 遠隔操作が送っている
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)

    # Act
    core.request_release(t, "release の指示", then_exit=True)

    # Assert
    assert core.tick(t) is None and core.state == "done"


def test_teleop_starting_during_release_silences_holder():
    # Arrange: 力を抜いている途中（温度で抜いている。終わったら待ちに戻る）
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)
    core.request_release(t, "温度")
    _, t = run_quiet(core, t, 0.5, body)
    assert core.state == "release"

    # Act: 遠隔操作が送り始める
    core.on_foreign(cmd, t)

    # Assert: 取り合わずにすぐ黙る
    assert core.tick(t) is None and core.state == "yield"


def test_stop_request_is_kept_when_teleop_interrupts_release():
    # Arrange: release の指示で力を抜いている途中
    core, cmd, body = make_core(), make_cmd(), make_body()
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)
    core.request_release(t, "release の指示", then_exit=True)
    _, t = run_quiet(core, t, 0.5, body)

    # Act: 遠隔操作が割り込んで送り、また止まる
    t = feed_teleop(core, t, 0.2, cmd, body)
    out, _ = run_quiet(core, t, 0.3, body)

    # Assert: 止める指示は残り、割り込みの後に引き継ぎ直さない
    assert all(c is None for c in out) and core.state == "done"


def test_hot_motor_while_holding_starts_release():
    # Arrange: 保っている
    core, cmd, body = make_core(temp_limit=70), make_cmd(), make_body(temp=60)
    t = feed_teleop(core, 0.0, 0.5, cmd, body)
    _, t = run_quiet(core, t, 0.2, body)

    # Act: 温度が線を越える
    outs, _ = run_quiet(core, t, S.RELEASE_S + 0.5, make_body(temp=71))

    # Assert: 力を抜き終えたら待ちに戻る（次の遠隔操作の後にまた保てる）
    assert outs[0] is not None and outs[-1] is None
    assert core.state == "idle" and core.held is None


# ---- form_check の判定 ----

@pytest.mark.parametrize("fsm, name, holder, ok, word", [
    (500, None, None, True, "通常運転"),
    (501, "ai", None, True, "通常運転"),
    (0, "ai", None, False, "ゼロトルク"),          # 11-04 朝に go しても腕が動かなかった状態
    (1, "ai", None, False, "R1+X"),
    (None, None, None, False, "restore"),
    (500, "ai", 1234, False, "保持係"),
])
def test_standing_form_needs_walking_fsm_and_no_holder(fsm, name, holder, ok, word):
    # Act
    got_ok, text = F.judge_standing(fsm, name, holder)

    # Assert
    assert got_ok is ok and word in text


def test_standing_form_reports_missing_robot_state_first():
    # Act: 機体の電源が切れている（RPC も rt/lowstate も来ない）
    ok, text = F.judge_standing(None, None, None, state_seen=False)

    # Assert: 「座位の後」と取り違えない
    assert not ok and "rt/lowstate" in text and "座位" not in text


@pytest.mark.parametrize("fsm, name, enabled, holder, for_replay, ok, word", [
    (1, "ai", True, None, False, True, "ダンピング"),          # 遠隔操作は起動のときに自分で純正を止める
    (1, "ai", True, None, True, False, "quit した後"),        # 再生は純正を止めない（止めた後の空白に誰も保たない）
    (0, "ai", False, None, False, False, "L2+B"),
    (500, "ai", True, None, False, False, "崩れる"),           # 立っている機体の純正を止めない
    (None, "", True, None, False, True, "止まっていて"),        # 切り替えの RPC が空の名前を答えた
    (None, None, True, "hold", True, True, "保持係が保っている"),  # 座位の 2 回目
    (None, None, True, None, False, False, "確かめられない"),   # RPC が両方答えないだけ（立っていて混んでいるだけかも）
    (None, None, True, "idle", False, False, "確かめられない"),
    (None, "", False, None, False, False, "無効"),
])
def test_seated_form_needs_damping_or_proof_that_stock_is_stopped(fsm, name, enabled, holder, for_replay, ok, word):
    # Arrange
    modes = np.full(S.N_BODY, 1 if enabled else 0)

    # Act
    got_ok, text = F.judge_seated(fsm, name, modes, holder, for_replay=for_replay)

    # Assert
    assert got_ok is ok and word in text
