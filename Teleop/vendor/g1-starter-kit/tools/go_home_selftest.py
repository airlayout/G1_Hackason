#!/usr/bin/env python3
"""[G1_Hackason 改変] safe-exit／go-home-release の自己テスト（機体不要）。

修正済みの robot_arm.py から G1_29_ArmController.ctrl_dual_arm_go_home だけを抜き出し、
偽の self・待たない sleep で呼んで、腕を戻しきれた／戻しきれないの両方で制御（arm_sdk の重み）が
2 秒かけて 0 へ返るかを確かめる。teleop_hand_and_arm.py の後始末で速さを絞るかも見る。

    python3 tools/go_home_selftest.py
    python3 tools/go_home_selftest.py --arm <robot_arm.py> --teleop <teleop_hand_and_arm.py>
"""

import argparse
import ast
import sys
import textwrap
import threading
import types
from pathlib import Path

import numpy as np

TELEOP = Path.home() / "xr_teleoperate" / "teleop"
WEIGHT_INDEX = 29      # G1_29_JointIndex.kNotUsedJoint0（arm_sdk の重み）


def load_go_home(path: Path):
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text)
    for cls in tree.body:
        if isinstance(cls, ast.ClassDef) and cls.name == "G1_29_ArmController":
            for fn in cls.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == "ctrl_dual_arm_go_home":
                    return textwrap.dedent(ast.get_source_segment(text, fn)), text
    raise SystemExit(f"[自己テスト] ✗ G1_29_ArmController.ctrl_dual_arm_go_home が見つからない: {path}")


def run_case(src: str, measured: np.ndarray, motion_mode: bool = True):
    """戻す処理を 1 回呼び、送った重みの並びと記録係の文言を返す。"""
    weights, logs = [], []
    msg = types.SimpleNamespace(motor_cmd=[types.SimpleNamespace(q=1.0) for _ in range(35)])

    fake_time = types.SimpleNamespace(sleep=lambda s: weights.append(msg.motor_cmd[WEIGHT_INDEX].q))
    ns = {
        "np": np, "time": fake_time,
        "G1_29_JointIndex": types.SimpleNamespace(kNotUsedJoint0=WEIGHT_INDEX),
        "logger_mp": types.SimpleNamespace(info=logs.append, warning=lambda m: logs.append("警告: " + m)),
    }
    exec(src, ns)
    self = types.SimpleNamespace(
        ctrl_lock=threading.Lock(), q_target=None, motion_mode=motion_mode, msg=msg,
        get_current_dual_arm_q=lambda: measured.copy())
    ns["ctrl_dual_arm_go_home"](self)
    return msg.motor_cmd[WEIGHT_INDEX].q, weights, logs, self.q_target


def main() -> int:
    p = argparse.ArgumentParser(description="safe-exit／go-home-release の自己テスト")
    p.add_argument("--arm", type=Path, default=TELEOP / "robot_control" / "robot_arm.py")
    p.add_argument("--teleop", type=Path, default=TELEOP / "teleop_hand_and_arm.py")
    args = p.parse_args()
    src, text = load_go_home(args.arm)
    if "PATCH(go-home-release)" not in src:
        print(f"[自己テスト] ✗ go-home-release が当たっていません: {args.arm}")
        return 1

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    # 戻しきれた場合（上流どおり）
    w, ws, logs, target = run_case(src, np.zeros(14))
    check(np.allclose(target, 0), "目標は 0 の姿勢")
    check(w == 0.0 and len(ws) >= 100, f"戻しきれたら重みを 0 まで下げる（{len(ws)} 回 × 0.02 秒）")
    check(not any("警告" in m for m in logs), "戻しきれたら警告は出ない")

    # 戻しきれない場合（10-27 に起きた形）
    w, ws, logs, _ = run_case(src, np.full(14, 0.5))
    ramp = [x for x in ws if x < 1.0]
    check(w == 0.0, "戻しきれなくても重みを 0 まで下げる（純正へ返す）")
    check(len(ramp) >= 99 and all(a >= b for a, b in zip(ramp, ramp[1:])), "重みは少しずつ下がる（いきなり 0 にしない）")
    check(any("戻りきれなかった" in m for m in logs), "戻りきれなかったことを記録係に残す")

    # 腕を借りていない（--motion なし）ときは重みに触らない
    w, ws, logs, _ = run_case(src, np.full(14, 0.5), motion_mode=False)
    check(w == 1.0, "--motion なしでは重みに触らない")

    # 後始末で速さを絞る（safe-exit）
    teleop_text = args.teleop.read_text(encoding="utf-8")
    i = teleop_text.find("arm_ctrl.ctrl_dual_arm_go_home()")
    j = teleop_text.rfind("arm_ctrl.set_arm_velocity_limit(2.0)", 0, i)
    check("PATCH(safe-exit)" in teleop_text and 0 <= j and i - j < 120,
          "終わるときは腕を戻す直前に 2 rad/s に絞る")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
