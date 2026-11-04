#!/usr/bin/env python3
"""[G1_Hackason 改変] 再生の歩き（replay_arm.py --legs・tools/loco_replay.py）の自己テスト（機体不要。2026-11-03）。

偽の DDS（unitree_sdk2py の口を差し替える）と偽の純正（SetVelocity を数えるだけ）で、再生の流れを丸ごと回す。
numpy だけで動く（OMEN の tv でも Mac でも）。約 20 秒。

    python3 tools/replay_legs_selftest.py

確かめること:
  1. 記録の読み出し: 歩きの指令・1 コマごとの時刻。足なしの記録は歩きなし、歩きの欄が無い記録は None
  2. 遠隔操作が送りえない歩きの値（前 1.0 超・後ろ／横／旋回 0.3 超・NaN）は断る
  3. 指令どおりに歩いたときの移動（前へ 0.5 m/s を 2 秒で 1 m・旋回しながら）
  4. 記録の時刻どおりのコマの位置（名目の fps とずれる記録で）
  5. 送り手: 歩く間だけ送り、やめたら 0 を 3 回、何もしない間は送らない。失敗を数える
  6. 再生の流れ（main() と同じ attach_legs のつなぎ方）: 歩きは再生の間だけ記録のコマの値で送り、
     終わりに腕を戻す前に 0 を送る。制御権は 0→1→0 を 1 回だけ（返した後に掴み直さない）。腰は kp 300 で保つ
  7. 歩いている最中の中断でも 0 を送り、制御権を 1 回で返す
  8. 後始末の途中で送信が落ちても、main() の finally の 2 回目が重み 0 まで返し直す
  9. SetVelocity が続けて拒まれたら再生ごと止める
 10. 記録の時刻どおりの腕の速さ（1 周期 20 ms を下限に 30 rad/s を超えたら断る）
 11. main() の入口: --legs と --waist none・歩きの無い記録に --legs は断る
"""

import contextlib
import io
import json
import math
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

import numpy as np

FPS = 30.0
N_FRAMES = 30          # 約 1 秒の記録
WALK = (10, 20)        # このコマの間だけ前へ 0.5 m/s


# ---- 偽の DDS（ArmReplayer が import する unitree_sdk2py の口） ---------------------

class FakeBus:
    def __init__(self) -> None:
        self.writes = []        # (時刻, 重み, 腰の kp)
        self.raise_below = None  # 重みがこれ未満（0 より大きい）の送信を 1 度だけ落とす
        self.lock = threading.Lock()
        self.state = types.SimpleNamespace(
            mode_machine=5,
            motor_state=[types.SimpleNamespace(q=0.0, dq=0.0, vol=24.0, temperature=[30, 30]) for _ in range(35)])


BUS = FakeBus()


class FakePublisher:
    def __init__(self, topic, _type) -> None:
        self.topic = topic

    def Init(self) -> None:  # noqa: N802  SDK と同じ名前
        pass

    def Write(self, msg) -> None:  # noqa: N802
        w = msg.motor_cmd[29].q
        if BUS.raise_below is not None and 0.0 < w < BUS.raise_below:
            BUS.raise_below = None        # 1 度だけ
            raise RuntimeError("偽の送信の失敗")
        with BUS.lock:
            BUS.writes.append((time.monotonic(), w, msg.motor_cmd[12].kp))


class FakeSubscriber:
    def __init__(self, topic, _type) -> None:
        self.quit = threading.Event()

    def Init(self, handler, _depth) -> None:  # noqa: N802
        def pump() -> None:
            while not self.quit.is_set():
                handler(BUS.state)
                time.sleep(0.005)
        threading.Thread(target=pump, daemon=True).start()


def fake_lowcmd():
    return types.SimpleNamespace(mode_pr=0, mode_machine=0, crc=0, motor_cmd=[
        types.SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(35)])


def install_fake_sdk() -> None:
    mods = {
        "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
        "unitree_sdk2py.core": types.ModuleType("unitree_sdk2py.core"),
        "unitree_sdk2py.core.channel": types.SimpleNamespace(ChannelPublisher=FakePublisher,
                                                             ChannelSubscriber=FakeSubscriber),
        "unitree_sdk2py.idl": types.ModuleType("unitree_sdk2py.idl"),
        "unitree_sdk2py.idl.default": types.SimpleNamespace(unitree_hg_msg_dds__LowCmd_=fake_lowcmd),
        "unitree_sdk2py.idl.unitree_hg": types.ModuleType("unitree_sdk2py.idl.unitree_hg"),
        "unitree_sdk2py.idl.unitree_hg.msg": types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg"),
        "unitree_sdk2py.idl.unitree_hg.msg.dds_": types.SimpleNamespace(LowCmd_=object, LowState_=object),
        "unitree_sdk2py.utils": types.ModuleType("unitree_sdk2py.utils"),
        "unitree_sdk2py.utils.crc": types.SimpleNamespace(CRC=lambda: types.SimpleNamespace(Crc=lambda _m: 0)),
    }
    sys.modules.update(mods)


class FakeLoco:
    """偽の純正。SetVelocity を (時刻, vx, vy, vyaw, 持続) で数える。fail=True なら失敗の戻り値を返す。"""

    def __init__(self, fail: bool = False) -> None:
        self.calls = []
        self.fail = fail

    def SetVelocity(self, vx, vy, vyaw, duration):  # noqa: N802
        self.calls.append((time.monotonic(), vx, vy, vyaw, duration))
        return 3104 if self.fail else 0


# ---- 記録を作る -------------------------------------------------------------------------

def write_episode(root: Path, name: str, body, dt_ms: float = 40.0, with_time: bool = True) -> Path:
    """腕は左肩 pitch だけ 0→0.3 rad、歩きは body（コマごとの [vx, vy, vyaw] か None）の記録を書く。"""
    ep = root / name
    ep.mkdir()
    frames = []
    for i in range(N_FRAMES):
        arm = [0.3 * i / (N_FRAMES - 1)] + [0.0] * 6
        fr = {"idx": i, "states": {"left_arm": {"qpos": arm}, "right_arm": {"qpos": [0.0] * 7}},
              "actions": {"left_arm": {"qpos": arm}, "right_arm": {"qpos": [0.0] * 7}}}
        if with_time:
            fr["time_ns"] = 1_000_000_000 + int(i * dt_ms * 1e6)
        if body is not None:
            fr["actions"]["body"] = {"qpos": list(body(i))}
        frames.append(fr)
    (ep / "data.json").write_text(json.dumps({"info": {"image": {"fps": FPS}}, "data": frames}), encoding="utf-8")
    return ep


def start_replay(ra, lr, cmd, fail: bool = False):
    """偽の DDS の記録を消し、main() と同じ attach_legs でつないだ再生と送り手を作る。フックは呼ばれた時刻も残す。"""
    BUS.writes.clear()
    BUS.raise_below = None
    rep = ra.ArmReplayer("lo", ra.ARM_JOINTS_29, ra.WAIST_JOINTS_29, ra.DEFAULT_KP, ra.DEFAULT_KD, "rt/arm_sdk")
    loco = FakeLoco(fail=fail)
    sender = lr.LocoSender(loco)
    ra.attach_legs(rep, sender, cmd)
    events = []
    real_play, real_stop = rep.play_hook, rep.stop_hook

    def play(i: int) -> None:
        events.append(("frame", time.monotonic(), i))
        real_play(i)

    def stop() -> None:
        events.append(("stop", time.monotonic(), None))
        real_stop()

    rep.play_hook, rep.stop_hook = play, stop
    sender.start()
    return rep, loco, sender, events


def check_stopped(check, loco, events, when: str) -> None:
    t_stop = next(e[1] for e in events if e[0] == "stop")
    zeros = [c for c in loco.calls if c[0] >= t_stop and c[1:4] == (0.0, 0.0, 0.0)]
    check(len(zeros) >= 3, f"{when}に 0 を送った（{len(zeros)} 回）")
    check(not [c for c in loco.calls if c[0] > t_stop + 0.06 and c[1] != 0.0], f"{when}の後に歩きを送っていない")


def check_weights(check, when: str) -> None:
    w = [x[1] for x in BUS.writes]
    peak = w.index(1.0)
    check(w[0] < 0.1 and abs(w[-1]) < 1e-9 and all(a >= b - 1e-9 for a, b in zip(w[peak:], w[peak + 1:])),
          f"{when}制御権は 0→1 に上げ、最後に 0 へ下ろして終わる")
    returned = next(i for i in range(peak, len(w)) if w[i] < 1e-9)
    check(all(x < 1e-9 for x in w[returned:]), f"{when}制御権を返した後に掴み直していない")


def exits(fn) -> bool:
    """fn が sys.exit（SystemExit）で止まったか。出力は捨てる。"""
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            fn()
            return False
        except SystemExit as e:
            return e.code not in (0, None)


def run_main(ra, ep: Path, *extra: str) -> None:
    argv = sys.argv
    sys.argv = ["replay_arm.py", "--episode", str(ep), "--no-plot", *extra]
    try:
        ra.main()
    finally:
        sys.argv = argv


def walk_body(i: int):
    return (0.5, 0.0, 0.0) if WALK[0] <= i < WALK[1] else (0.0, 0.0, 0.0)


def main() -> int:
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    install_fake_sdk()
    import loco_replay as lr
    import replay_arm as ra

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    def refused(cmd) -> bool:
        try:
            lr.check_loco(np.asarray(cmd, dtype=float))
            return False
        except SystemExit:
            return True

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        ep_walk = write_episode(root, "walk", walk_body)
        ep_none = write_episode(root, "noleg", lambda i: (0.0, 0.0, 0.0))
        ep_old = write_episode(root, "old", None, with_time=False)

        print("\n[1] 記録の読み出し")
        cmd = lr.load_loco(ep_walk)
        check(cmd is not None and cmd.shape == (N_FRAMES, 3), "歩きの指令を (コマ数, 3) で読める")
        check(int(lr.walking_mask(cmd).sum()) == WALK[1] - WALK[0], "歩いたコマの数が合う")
        check(not lr.walking_mask(lr.load_loco(ep_none)).any(), "足なしの記録は歩きなし")
        check(lr.load_loco(ep_old) is None, "歩きの欄が無い記録は None")
        times = ra.load_times(ep_walk, N_FRAMES)
        check(times is not None and abs(times[-1] - (N_FRAMES - 1) * 0.040) < 1e-9, "1 コマごとの時刻を秒で読める")
        check(ra.load_times(ep_old, N_FRAMES) is None, "time_ns が無い記録は None（名目の fps で流す）")

        print("\n[2] 送りえない値を断る")
        check(not refused([[1.0, 0.3, -0.3], [-0.3, -0.3, 0.3]]), "上限ちょうどは通す")
        for bad, label in (([[1.2, 0, 0]], "前 1.2"), ([[-0.5, 0, 0]], "後ろ 0.5"),
                           ([[0, 0.4, 0]], "横 0.4"), ([[0, 0, -0.6]], "旋回 0.6"), ([[math.nan, 0, 0]], "NaN")):
            check(refused(bad), f"{label} は断る")

        print("\n[3] 指令どおりに歩いたときの移動")
        x, y, yaw = lr.dead_reckon(np.array([[0.5, 0.0, 0.0]] * 20), np.full(20, 0.1))
        check(abs(x - 1.0) < 1e-9 and abs(y) < 1e-9, f"前へ 0.5 m/s を 2 秒で 1 m（{x:.3f}, {y:.3f}）")
        x, y, yaw = lr.dead_reckon(np.array([[0.0, 0.0, 0.3]] * 10 + [[0.3, 0.0, 0.0]] * 10), np.full(20, 0.5))
        check(abs(math.degrees(yaw) - 85.94) < 0.01 and y > 1.4 and abs(x - 0.3 * 5 * math.cos(1.5)) < 1e-9,
              f"左へ 86° 回ってから前へ 1.5 m は、ほぼ左へ進む（前 {x:.2f}・左 {y:.2f}）")

        print("\n[4] 記録の時刻どおりのコマの位置")
        check(ra.frame_pos(0.40, FPS) == 12.0, "名目 30 Hz なら 0.40 秒は 12 コマ目")
        check(abs(ra.frame_pos(0.40, FPS, times) - 10.0) < 1e-9, "40 ms ごとの記録なら 0.40 秒は 10 コマ目")
        check(abs(ra.frame_pos(9.9, FPS, times) - (N_FRAMES - 1)) < 1e-9, "終わりを過ぎても最後のコマ")

        print("\n[5] 送り手")
        loco = FakeLoco()
        s = lr.LocoSender(loco, rate_hz=100.0)
        s.start()
        time.sleep(0.05)
        check(not loco.calls, "何も置かない間は送らない")
        s.set(0.5, 0.0, 0.1)
        time.sleep(0.1)
        walking = [c for c in loco.calls if c[1:4] == (0.5, 0.0, 0.1)]
        check(len(walking) >= 5 and all(c[4] == lr.VEL_DURATION_S for c in walking), "歩く間は SetVelocity（持続 0.5 秒）を送り続ける")
        n_before = len(loco.calls)
        s.set(0.0, 0.0, 0.0)
        time.sleep(0.1)
        after = loco.calls[n_before:]
        check(len(after) == lr.STOP_REPEATS and all(c[1:4] == (0.0, 0.0, 0.0) for c in after), "歩くのをやめたら 0 を 3 回だけ")
        s.close()
        check(len(loco.calls) == n_before + 2 * lr.STOP_REPEATS, "閉じるときも 0 を 3 回送ってから終わる")
        bad = FakeLoco(fail=True)
        s2 = lr.LocoSender(bad, rate_hz=100.0)
        s2.start()
        s2.set(0.3, 0.0, 0.0)
        time.sleep(0.1)
        s2.close()
        check(s2.status()[1] >= lr.FAIL_TRIP and s2.status()[2] == 3104, "拒まれた数を続けて数える（再生の側が止める）")

        print("\n[6] 再生の流れ（偽の DDS・main() と同じつなぎ方 attach_legs・約 6 秒）")
        traj = np.array([[0.3 * i / (N_FRAMES - 1)] + [0.0] * 13 for i in range(N_FRAMES)])
        rep, loco, sender, events = start_replay(ra, lr, cmd)
        rep.run(traj, FPS, 0.2, 0.3, "hold", times)
        n_writes = len(BUS.writes)
        rep._release(None, 0.2)          # main() の finally が呼ぶ 2 回目
        sender.close()

        frames = [e for e in events if e[0] == "frame"]
        t_play0 = frames[0][1]
        idx = [e[2] for e in frames]
        check(idx == sorted(idx) and idx[-1] >= N_FRAMES - 2, f"再生の間、コマは戻らず終わりまで進む（最後 {idx[-1]}）")
        late = [abs((t - t_play0) - times[i]) for _, t, i in frames]
        check(max(late) < 0.1, f"コマは記録の時刻どおりに進む（ずれ最大 {max(late) * 1000:.0f} ms）")
        walk_calls = [c for c in loco.calls if c[1] == 0.5]
        check(bool(walk_calls), "記録の歩いたコマで前へ 0.5 m/s を送った")
        t_w0, t_w1 = walk_calls[0][0] - t_play0, walk_calls[-1][0] - t_play0
        check(times[WALK[0]] - 0.1 <= t_w0 and t_w1 <= times[WALK[1]] + 0.1,
              f"歩きは記録の歩いた時間の中だけ（{t_w0:.2f}〜{t_w1:.2f} 秒・記録 {times[WALK[0]]:.2f}〜{times[WALK[1]]:.2f}）")
        check_stopped(check, loco, events, "再生の終わり")
        check_weights(check, "")
        check(len(BUS.writes) == n_writes, "2 回目の後始末では何も送らない")
        check(all(abs(x[2] - ra.KP_HIGH) < 1e-9 for x in BUS.writes), f"腰は kp {ra.KP_HIGH:g} で保つ（遠隔操作と同じ）")

        print("\n[7] 歩いている最中に中断（Enter・シグナルと同じ abort）")
        rep, loco, sender, events = start_replay(ra, lr, cmd)
        threading.Timer(0.2 + 0.3 + times[15], rep.abort.set).start()   # 歩いている 15 コマ目のあたり
        rep.run(traj, FPS, 0.2, 0.3, "hold", times)
        n_writes = len(BUS.writes)
        rep._release(None, 0.2)
        sender.close()
        idx = [e[2] for e in events if e[0] == "frame"]
        check(bool(idx) and idx[-1] < WALK[1], f"再生は歩いている途中で止まった（最後 {idx[-1] if idx else None} コマ目）")
        check_stopped(check, loco, events, "中断")
        check_weights(check, "中断でも")
        check(len(BUS.writes) == n_writes, "中断の後の 2 回目の後始末でも何も送らない")

        print("\n[8] 後始末の途中で例外 → main() の finally の 2 回目がやり直す")
        rep, loco, sender, events = start_replay(ra, lr, cmd)
        real_stop, armed = rep.stop_hook, []

        def stop_then_arm() -> None:
            # 再生が終わった後（最初の stop）に 1 度だけ仕掛ける: 制御権を返す途中（重み 0.5 未満）で送信が 1 度落ちる
            if not armed:
                armed.append(True)
                BUS.raise_below = 0.5
            real_stop()

        rep.stop_hook = stop_then_arm
        raised = False
        try:
            rep.run(traj, FPS, 0.2, 0.3, "hold", times)
        except RuntimeError:
            raised = True
        w_at_fail = BUS.writes[-1][1]
        n_writes = len(BUS.writes)
        rep._release(None, 0.2)
        sender.close()
        check(raised and 0.0 < w_at_fail < 1.0, f"返す途中で落ちた（その時の重み {w_at_fail:.2f}）")
        check(len(BUS.writes) > n_writes and BUS.writes[-1][1] == 0.0, "2 回目の後始末が送り直し、重み 0 まで返した")

        print("\n[9] SetVelocity が続けて拒まれる → 再生ごと止める")
        rep, loco, sender, events = start_replay(ra, lr, cmd, fail=True)
        rep.run(traj, FPS, 0.2, 0.3, "hold", times)
        sender.close()
        check("続けて失敗" in rep.abort_reason, f"止めた理由: {rep.abort_reason}")
        check_weights(check, "拒まれて止めたときも")

        print("\n[10] 記録の時刻どおりの腕の速さ")
        q = np.zeros((4, 14))
        check(not exits(lambda: ra.validate_timing(q, np.array([0.0, 0.04, 0.08, 0.12]))), "動かない記録は通す")
        q[2, 0] = 0.35
        check(not exits(lambda: ra.validate_timing(q, np.array([0.0, 0.04, 0.043, 0.08]))),
              "3 ms で 0.35 rad（1 周期 20 ms で 17.5 rad/s）は警告だけ")
        q[2, 0] = 1.0
        check(exits(lambda: ra.validate_timing(q, np.array([0.0, 0.04, 0.043, 0.08]))), "3 ms で 1.0 rad（50 rad/s）は断る")
        check(exits(lambda: ra.validate_timing(q, np.array([0.0, 0.04, 0.07, 0.11]))), "30 ms で 1.0 rad（33 rad/s）も断る")

        print("\n[11] main() の入口で断るもの")
        check(exits(lambda: run_main(ra, ep_walk, "--legs", "--waist", "none")), "--legs と --waist none は断る（腰が脱力したまま歩かない）")
        check(exits(lambda: run_main(ra, ep_none, "--legs")), "歩きの無い記録に --legs は断る")
        check(not exits(lambda: run_main(ra, ep_walk, "--legs")), "歩きのある記録の確かめ（dry-run）は通る")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
