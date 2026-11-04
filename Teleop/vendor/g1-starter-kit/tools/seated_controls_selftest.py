#!/usr/bin/env python3
"""[G1_Hackason 改変] seated-controls（座位: --motion なしでも一時停止・A の終了を効かせる）の自己テスト（機体・Quest 不要。2026-11-04）。

修正済みの teleop_hand_and_arm.py から、高い段の操作の塊・IK の後の数行・記録の数行を抜き出し、
--motion なし（純正を止めて rt/lowcmd の形）で偽の機体・偽の時計・偽の Quest で回す。
抜き出し方と偽物は stick_pause_selftest.py と同じものを使う。setup/apply_patches.py で当てた後に流す。

    python3 tools/seated_controls_selftest.py
    python3 tools/seated_controls_selftest.py --file <teleop_hand_and_arm.py>

確かめること（--motion なし）:
  1. 歩きの口（loco_wrapper）に一度も触らない（偽の名前空間に置かないので、触れば NameError で落ちる）
  2. 腕は IK の解に追従し、記録の歩きは 0・全身の角度（29）は残る
  3. 両スティック 0.6 秒で一時停止（腕は止めた瞬間の実測の角度）、もう一度で再開（2 秒は 2 rad/s）
  4. Quest のデータが 0.3 秒届かないと一時停止
  5. 右の A で終わる（START=False・STOP=True）
"""

import argparse
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

from stick_pause_selftest import DT, FakeArm, FakeClock, FakeIK, extract, make_tele

DEFAULT_FILE = Path.home() / "xr_teleoperate" / "teleop" / "teleop_hand_and_arm.py"


def main() -> int:
    p = argparse.ArgumentParser(description="seated-controls の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    a = p.parse_args()
    text = a.file.read_text(encoding="utf-8")
    if "PATCH(seated-controls)" not in text:
        print(f"[自己テスト] ✗ seated-controls が当たっていません: {a.file}")
        return 1

    init_src = extract(text, "PATCH(stick-pause): 両スティック長押し", "while not STOP:", False)
    high_src = extract(text, "# high level control", "# get current robot state data.", False)
    ik_src = extract(text, "sol_q, sol_tauff  = arm_ik.solve_ik", "arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)", True)
    rec_src = "if True:\n" + textwrap.indent(
        extract(text, "PATCH(record-loco-cmd): 手なしでも", "current_body_action = []", True), "    ")

    clock, arm = FakeClock(), FakeArm()
    tvuer = types.SimpleNamespace(last_motion_time=0.0)
    ns = {
        "time": clock, "arm_ctrl": arm, "arm_ik": FakeIK(),   # loco_wrapper は置かない（--motion なしでは作られない）
        "tv_wrapper": types.SimpleNamespace(tvuer=tvuer),
        "args": types.SimpleNamespace(input_mode="controller", motion=False, loco_fwd_max=0.3, no_loco=False),
        "logger_mp": types.SimpleNamespace(warning=lambda m: print(f"    （記録係）{m}"),
                                           info=lambda m: print(f"    （記録係）{m}"), debug=lambda m: None),
        "START": True, "STOP": False,
        "current_lr_arm_q": None, "current_lr_arm_dq": None, "time_ik_start": 0.0,
    }
    exec(init_src, ns)
    code_high, code_ik, code_rec = (compile(s, n, "exec") for s, n in
                                    ((high_src, "high"), (ik_src, "ik"), (rec_src, "rec")))

    def step(both: bool, fresh: bool = True, a_button: bool = False):
        if fresh:
            tvuer.last_motion_time = clock.now
        tele = make_tele(both, 1.0, True)    # 左スティックは前へ倒しきったまま（座位では歩かない）
        tele.right_ctrl_aButton = a_button
        ns["tele_data"] = tele
        exec(code_high, ns)
        exec(code_ik, ns)
        exec(code_rec, ns)
        clock.now += DT
        return arm.sent_q.copy(), list(ns["current_body_action"]), list(ns["current_body_state"])

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    try:
        q, act, body = step(False)
    except NameError as e:
        check(False, f"--motion なしで歩きの口に触った（{e}）")
        print(f"\n[自己テスト] 不合格 {len(fails)} 件")
        return 1
    check(True, "--motion なしで歩きの口（loco_wrapper）に触らない")
    check(np.allclose(q, 0.9), "腕は IK の解（操作者の手）に追従する")
    check(act == [0.0, 0.0, 0.0], f"スティックを前へ倒しても記録の歩きは 0（{act}）")
    check(len(body) == 29, f"記録に全身の角度が残る（{len(body)} 関節）")

    arm.measured = np.linspace(-0.3, 0.3, 14)
    for _ in range(int(0.6 / DT)):
        q, act, body = step(True)
    check(ns["stick_paused"], "両スティック 0.6 秒で一時停止する")
    check(np.allclose(q, np.linspace(-0.3, 0.3, 14)), "一時停止中は止めた瞬間の実測の角度を送る")
    q, act, body = step(False)
    for _ in range(int(0.6 / DT)):
        q, act, body = step(True)
    check(not ns["stick_paused"] and arm.velocity_limit == 2.0, "もう一度 0.6 秒で再開し、腕を 2 rad/s に絞る")
    for _ in range(int(2.2 / DT)):
        q, act, body = step(False)
    check(arm.velocity_limit == 30.0 and np.allclose(q, 0.9), "2 秒後に 30 rad/s へ戻り、手に追従する")

    arm.measured = np.linspace(0.2, 0.5, 14)
    for _ in range(int(0.5 / DT)):
        q, act, body = step(False, fresh=False)
    check(ns["stick_paused"], "Quest のデータが 0.3 秒を超えて届かないと一時停止する")
    check(np.allclose(q, np.linspace(0.2, 0.5, 14)), "途切れの一時停止でも腕は止めた瞬間の角度")

    q, act, body = step(False, a_button=True)
    check(ns["STOP"] and not ns["START"], "右の A で終わる（STOP）")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
