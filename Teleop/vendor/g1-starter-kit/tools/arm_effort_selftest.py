#!/usr/bin/env python3
"""[G1_Hackason 改変] arm-tau-getter／arm-tau-store／record-arm-effort の自己テスト（機体不要）。

修正済みの robot_arm.py から MotorState・G1_29_LowState・状態を写す行・get_current_dual_arm_tau を、teleop_hand_and_arm.py から
速さとトルクを取り出す部分を抜き出し、偽の機体の状態で呼んで、記録に左右 7 関節ずつの速さとトルクが入るかを見る。

    python3 tools/arm_effort_selftest.py
    python3 tools/arm_effort_selftest.py --arm <robot_arm.py> --teleop <teleop_hand_and_arm.py>
"""

import argparse
import ast
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

TELEOP = Path.home() / "xr_teleoperate" / "teleop"
ARM_IDS = list(range(15, 29))     # G1_29_JointArmIndex（左腕 15〜21・右腕 22〜28）


def method_src(text: str, cls: str, name: str) -> str:
    for node in ast.parse(text).body:
        if isinstance(node, ast.ClassDef) and node.name == cls:
            for fn in node.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == name:
                    return textwrap.dedent(ast.get_source_segment(text, fn))
    raise SystemExit(f"[自己テスト] ✗ {cls}.{name} が見つからない（arm-tau-getter が当たっていない？）")


def main() -> int:
    p = argparse.ArgumentParser(description="arm-tau-getter／record-arm-effort の自己テスト")
    p.add_argument("--arm", type=Path, default=TELEOP / "robot_control" / "robot_arm.py")
    p.add_argument("--teleop", type=Path, default=TELEOP / "teleop_hand_and_arm.py")
    a = p.parse_args()
    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    # 1. 腕の制御の部品のトルクの口（本物の道筋: 受け取った状態 → 自前の入れ物へ写す → 読む）
    # 10-29 の実機で、自前の入れ物（MotorState）に tau_est が無く AttributeError で落ちた。
    # 偽の入れ物ではなく、修正済みのファイルの MotorState・G1_29_LowState と写しの行をそのまま使う。
    arm_text = a.arm.read_text(encoding="utf-8")
    tree = ast.parse(arm_text)
    cls_src = "".join(ast.get_source_segment(arm_text, n) + "\n" for n in tree.body
                      if isinstance(n, ast.ClassDef) and n.name in ("MotorState", "G1_29_LowState"))
    ns = {"np": np, "G1_29_JointArmIndex": ARM_IDS, "G1_29_Num_Motors": 35}
    exec(cls_src, ns)
    sub_src = method_src(arm_text, "G1_29_ArmController", "_subscribe_motor_state")
    sub_lines = sub_src.splitlines(keepends=True)
    i0 = next(k for k, ln in enumerate(sub_lines) if "lowstate = G1_29_LowState()" in ln)
    i1 = next(k for k in range(i0, len(sub_lines)) if "self.lowstate_buffer.SetData(lowstate)" in sub_lines[k])
    copy_src = textwrap.dedent("".join(sub_lines[i0:i1]))
    msg = types.SimpleNamespace(motor_state=[types.SimpleNamespace(q=0.1 * i, dq=0.01 * i, tau_est=float(i) + 0.5)
                                             for i in range(35)])
    ns["msg"] = msg
    exec(copy_src, ns)
    lowstate = ns["lowstate"]
    check(hasattr(lowstate.motor_state[0], "tau_est"), "自前の入れ物（MotorState）に tau_est の欄がある")
    check(getattr(lowstate.motor_state[20], "tau_est", None) == 20.5, "受け取った状態を写すとき tau_est も写す")
    if fails:
        print("\n[自己テスト] 不合格（arm-tau-store が当たっていない）")
        return 1
    exec(method_src(arm_text, "G1_29_ArmController", "get_current_dual_arm_tau"), ns)
    fake = types.SimpleNamespace(lowstate_buffer=types.SimpleNamespace(GetData=lambda: lowstate))
    tau = ns["get_current_dual_arm_tau"](fake)
    check(tau.shape == (14,) and np.allclose(tau, np.array(ARM_IDS) + 0.5), "腕 14 関節の tau_est を左→右の順で返す（写した入れ物から）")

    # 2. 記録を組み立てる部分
    text = a.teleop.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    i = next(k for k, ln in enumerate(lines) if "PATCH(record-arm-effort): 腕の速さと" in ln)
    j = next(k for k in range(i, len(lines)) if "left_arm_tau, right_arm_tau = [], []" in lines[k])
    block = textwrap.dedent("".join(lines[i:j + 1]))
    env = {"half": 7, "current_lr_arm_dq": np.arange(14, dtype=float) * 0.1,
           "arm_ctrl": types.SimpleNamespace(get_current_dual_arm_tau=lambda: np.arange(14, dtype=float))}
    exec(block, env)
    check(np.allclose(env["left_arm_dq"], np.arange(7) * 0.1) and np.allclose(env["right_arm_dq"], np.arange(7, 14) * 0.1),
          "速さを左右 7 関節ずつに分ける")
    check(env["left_arm_tau"] == list(range(7)) and env["right_arm_tau"] == list(range(7, 14)),
          "トルクを左右 7 関節ずつのリストにする")
    env2 = {"half": 7, "current_lr_arm_dq": np.zeros(14), "arm_ctrl": types.SimpleNamespace()}
    exec(block, env2)
    check(env2["left_arm_tau"] == [] and env2["right_arm_tau"] == [], "トルクの口が無い腕では空欄のまま（落ちない）")
    check('"qvel":   left_arm_dq.tolist(),' in text and '"torque": left_arm_tau,' in text
          and '"qvel":   right_arm_dq.tolist(),' in text and '"torque": right_arm_tau,' in text,
          "記録の左右の腕の qvel・torque に入れている")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
