#!/usr/bin/env python3
"""[G1_Hackason 改変] stick-pause／record-loco-cmd の自己テスト（機体・Quest 不要）。

修正済みの teleop_hand_and_arm.py から該当の部分だけを抜き出し、偽の機体・偽の時計で
1 周期ずつ回して振る舞いを確かめる。setup/apply_patches.py で当てた後に流す。

    python3 tools/stick_pause_selftest.py                       # ~/xr_teleoperate を見る
    python3 tools/stick_pause_selftest.py --file <teleop_hand_and_arm.py>

確かめること:
  1. 0.5 秒未満の押し込みでは切り替わらない（触れただけ・短く押しただけでは止まらない）
  2. 0.5 秒押し続けると一時停止: 歩きの指令は 0、腕は止めた瞬間の実測の角度
  3. 押し続けても再び切り替わらない（離すまで次の切り替えをしない）
  4. 一時停止中は Damp も腕の帰還も呼ばれない
  5. 再開すると腕の速さを 2 rad/s に絞り、2 秒後に 30 rad/s へ戻す
  6. 記録の歩きの指令は、機体へ送った値と同じ（一時停止中は 0）
"""

import argparse
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

DEFAULT_FILE = Path.home() / "xr_teleoperate" / "teleop" / "teleop_hand_and_arm.py"
DT = 0.02  # 1 周期の秒数（テスト用）


def extract(text: str, start: str, end: str, include_end: bool) -> str:
    """start を含む行から end を含む行まで（include_end で end 行を含めるか）を字下げを外して返す。"""
    lines = text.splitlines(keepends=True)
    i = next(k for k, ln in enumerate(lines) if start in ln)
    j = next(k for k in range(i + 1, len(lines)) if end in lines[k])
    return textwrap.dedent("".join(lines[i:j + 1 if include_end else j]))


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def time(self) -> float:
        return self.now


class FakeArm:
    def __init__(self) -> None:
        self.measured = np.linspace(0.1, 1.4, 14)
        self.sent_q = None
        self.velocity_limit = 30.0
        self.go_home_called = False

    def get_current_dual_arm_q(self) -> np.ndarray:
        return self.measured.copy()

    def get_current_dual_arm_dq(self) -> np.ndarray:
        return np.zeros(14)

    def get_current_motor_q(self) -> np.ndarray:
        return np.zeros(29)

    def ctrl_dual_arm(self, q, tauff) -> None:
        self.sent_q = np.array(q)

    def set_arm_velocity_limit(self, v: float) -> None:
        self.velocity_limit = float(v)

    def ctrl_dual_arm_go_home(self) -> None:
        self.go_home_called = True


class FakeLoco:
    def __init__(self) -> None:
        self.moves = []
        self.damp_called = False

    def Move(self, vx: float, vy: float, vyaw: float) -> None:
        self.moves.append((vx, vy, vyaw))

    def Enter_Damp_Mode(self) -> None:
        self.damp_called = True


class FakeIK:
    def solve_ik(self, lw, rw, q, dq):
        # 操作者の手の位置に相当する解。止めた角度とは違う値にしておく。
        return np.full(14, 0.9), np.full(14, 0.05)


def make_tele(both_pressed: bool, stick_fwd: float) -> types.SimpleNamespace:
    return types.SimpleNamespace(
        right_ctrl_aButton=False,
        left_ctrl_thumbstick=both_pressed,
        right_ctrl_thumbstick=both_pressed,
        left_ctrl_thumbstickValue=np.array([0.0, -stick_fwd]),   # 前は (0, -1)
        right_ctrl_thumbstickValue=np.array([0.0, 0.0]),
        left_wrist_pose=None,
        right_wrist_pose=None,
    )


def main() -> int:
    p = argparse.ArgumentParser(description="stick-pause／record-loco-cmd の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    args_cli = p.parse_args()
    text = args_cli.file.read_text(encoding="utf-8")
    for marker in ("PATCH(stick-pause)", "PATCH(record-loco-cmd)"):
        if marker not in text:
            print(f"[自己テスト] ✗ {marker} が当たっていません: {args_cli.file}")
            return 1

    init_src = extract(text, "PATCH(stick-pause): 両スティック長押し", "while not STOP:", False)
    high_src = extract(text, 'if args.input_mode == "controller" and args.motion:',
                       "# get current robot state data.", False)
    ik_src = extract(text, "sol_q, sol_tauff  = arm_ik.solve_ik", "arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)", True)
    rec_src = extract(text, "PATCH(record-loco-cmd): 手なしでも", "current_body_action = []", True)
    rec_src = "if True:\n" + textwrap.indent(rec_src, "    ")

    clock, arm, loco = FakeClock(), FakeArm(), FakeLoco()
    ns = {
        "time": clock, "arm_ctrl": arm, "loco_wrapper": loco, "arm_ik": FakeIK(),
        "args": types.SimpleNamespace(input_mode="controller", motion=True),
        "logger_mp": types.SimpleNamespace(warning=lambda m: print(f"    （記録係）{m}"),
                                           debug=lambda m: None),
        "START": True, "STOP": False,
        "current_lr_arm_q": None, "current_lr_arm_dq": None,
        "time_ik_start": 0.0,
    }
    exec(init_src, ns)
    code_high, code_ik, code_rec = (compile(s, n, "exec") for s, n in
                                    ((high_src, "high"), (ik_src, "ik"), (rec_src, "rec")))

    def step(both: bool, fwd: float = 1.0):
        ns["tele_data"] = make_tele(both, fwd)
        exec(code_high, ns)
        exec(code_ik, ns)
        exec(code_rec, ns)
        clock.now += DT
        return loco.moves[-1], arm.sent_q.copy(), list(ns["current_body_action"])

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    # 1. 0.4 秒押して離す → 切り替わらない
    for _ in range(int(0.4 / DT)):
        mv, q, rec = step(True)
    mv, q, rec = step(False)
    check(not ns["stick_paused"], "0.4 秒の押し込みでは一時停止しない")
    check(abs(mv[0] - 0.3) < 1e-9, f"通常時は前に 0.3 m/s を送る（{mv[0]:.2f}）")
    check(rec == list(mv), "記録の歩きの指令＝送った値（通常時）")

    # 2. 0.6 秒押し続ける → 一時停止
    arm.measured = np.linspace(-0.3, 0.3, 14)          # 止めた瞬間の実測
    for _ in range(int(0.6 / DT)):
        mv, q, rec = step(True)
    check(ns["stick_paused"], "0.6 秒押し続けると一時停止する")
    check(mv == (0.0, 0.0, 0.0), "一時停止中は歩きの指令が 0")
    check(np.allclose(q, np.linspace(-0.3, 0.3, 14)), "腕は止めた瞬間の実測の角度を送る（IK の解ではない）")
    check(rec == [0.0, 0.0, 0.0], "記録の歩きの指令も 0")

    # 3. さらに 2 秒押し続ける → 切り替わらない
    for _ in range(int(2.0 / DT)):
        mv, q, rec = step(True)
    check(ns["stick_paused"], "押し続けても再び切り替わらない")

    # 4. 離して操作者が手を動かしても腕は固定のまま
    arm.measured = np.linspace(-0.25, 0.35, 14)        # 扉に押されて少し動いた
    for _ in range(10):
        mv, q, rec = step(False)
    check(np.allclose(q, np.linspace(-0.3, 0.3, 14)), "離しても一時停止中は止めた角度のまま")
    check(not loco.damp_called and not arm.go_home_called, "脱力も腕の帰還も呼ばれない")

    # 5. 再開 → 2 rad/s に絞り、2 秒後に 30 rad/s
    for _ in range(int(0.6 / DT)):
        mv, q, rec = step(True)
    check(not ns["stick_paused"], "もう一度 0.6 秒押すと再開する")
    check(arm.velocity_limit == 2.0, "再開直後は腕を 2 rad/s に絞る")
    check(np.allclose(q, 0.9), "再開後は IK の解（操作者の手）に追従する")
    for _ in range(int(2.2 / DT)):
        mv, q, rec = step(False)
    check(arm.velocity_limit == 30.0, "2 秒後に 30 rad/s へ戻す")
    check(abs(mv[0] - 0.3) < 1e-9 and rec == list(mv), "再開後は歩きの指令が戻り、記録とも一致")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
