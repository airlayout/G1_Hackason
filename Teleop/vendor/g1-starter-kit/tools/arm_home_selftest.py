#!/usr/bin/env python3
"""[G1_Hackason 改変] arm-home-hold／arm-home-wait の自己テスト（機体不要）。

修正済みの robot_arm.py から G1_29_ArmController.clip_arm_q_target だけを抜き出し、重さのある偽の腕
（14 関節・1 関節ずつ PD と重さのモーメント）を 250 Hz で回して、次を確かめる。

  - 起動と終わりの 2 rad/s で、垂れた姿勢から 0 の姿勢（腕を下ろした初期位置）まで届く
    （上流の寄せ方は届かないことも並べて出す。11-04 座位で start をくり返すと腕の止まる場所がずれた）
  - 腕がぶつかって動けないとき、目標と実測の差を 0.1 rad までにする（押し続けない）
  - 追従中（30 rad/s）は上流と同じ値を返す
  - teleop_hand_and_arm.py の「Press [r]」の手前で、腕が 0 に着くのを待ち、着いた／着かないをログに出す

    python3 tools/arm_home_selftest.py
    python3 tools/arm_home_selftest.py --arm <robot_arm.py> --teleop <teleop_hand_and_arm.py>
"""

import argparse
import ast
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

TELEOP = Path.home() / "xr_teleoperate" / "teleop"
KP, KD, DT = 80.0, 3.0, 1.0 / 250.0      # 腕の kp_low・kd_low・制御の周期（xr_teleoperate の G1_29）
INERTIA = 0.05                            # 1 関節の慣性 [kg·m²]（腕の質量と減速機の分の大まかな値）
# 0 の姿勢で垂れる向き（+）にかかる重さのモーメントの最大 [N·m]（URDF から: 肩 pitch 2.2・肘 1.9。ほかは小さく 0.3）
GRAVITY = np.array([2.2, 0.3, 0.3, 1.9, 0.3, 0.3, 0.3] * 2)
SLOW, FAST = 2.0, 30.0


def load_clip(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    for cls in ast.parse(text).body:
        if isinstance(cls, ast.ClassDef) and cls.name == "G1_29_ArmController":
            for fn in cls.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == "clip_arm_q_target":
                    return textwrap.dedent(ast.get_source_segment(text, fn))
    raise SystemExit(f"[自己テスト] ✗ G1_29_ArmController.clip_arm_q_target が見つからない: {path}")


def upstream_clip(self, target_q, velocity_limit):
    """上流 817fb00 の寄せ方（比べるため）。"""
    current_q = self.get_current_dual_arm_q()
    delta = target_q - current_q
    motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
    return current_q + delta / max(motion_scale, 1.0)


class Arm:
    """重さのある偽の腕。stop があれば、その角度より 0 の側へは動けない（ぶつかった）。"""

    def __init__(self, q0: np.ndarray, stop: dict = None) -> None:
        self.q, self.dq, self.stop = q0.astype(float).copy(), np.zeros(14), stop or {}
        self.control_dt = DT

    def get_current_dual_arm_q(self) -> np.ndarray:
        return self.q.copy()

    def step(self, cmd: np.ndarray, sub: int = 4) -> None:
        h = DT / sub
        for _ in range(sub):
            # 振り子と同じく、0（前腕が水平）で最大・垂れきる（+π/2）と 0
            tau = KP * (cmd - self.q) - KD * self.dq + GRAVITY * np.cos(np.clip(self.q, -np.pi / 2, np.pi / 2))
            self.dq += tau / INERTIA * h
            self.q += self.dq * h
            for j, lo in self.stop.items():
                if self.q[j] < lo:
                    self.q[j], self.dq[j] = lo, 0.0


def run(clip, arm: Arm, velocity_limit: float, seconds: float) -> list:
    """目標 0 で seconds 秒回す。毎周期の (指令, 実測) を返す。"""
    log = []
    for _ in range(int(seconds / DT)):
        cmd = clip(arm, np.zeros(14), velocity_limit)
        log.append((cmd.copy(), arm.q.copy()))
        arm.step(cmd)
    return log


def drooped() -> np.ndarray:
    """垂れた姿勢（肩 pitch・肘が重さの向きへ流れた所）。"""
    q = np.full(14, 0.05)
    q[[0, 7]], q[[3, 10]] = 0.4, 0.9
    return q


def check_wait(teleop: Path, check) -> None:
    text = teleop.read_text(encoding="utf-8")
    i = text.find("# PATCH(arm-home-wait)")
    j = text.find('logger_mp.info("🟢  Press [r] to start', i)
    check(0 <= i < j, "「Press [r]」の手前に、腕が初期位置に着くのを待つ所がある")
    if not 0 <= i < j:
        return
    k = text.rfind('logger_mp.info("-----', i, j)
    block = textwrap.dedent(text[text.rfind("\n", 0, i) + 1:k])

    def wait_case(seq: list) -> tuple:
        clock, logs, it = [0.0], [], iter(seq)
        last = [seq[-1]]

        def q() -> list:
            last[0] = next(it, last[0])
            return last[0]
        ns = {"time": types.SimpleNamespace(time=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0] + s)),
              "arm_ctrl": types.SimpleNamespace(get_current_dual_arm_q=q),
              "logger_mp": types.SimpleNamespace(info=lambda m: logs.append(m), warning=lambda m: logs.append("警告: " + m))}
        exec(block, ns)
        return clock[0], logs

    t, logs = wait_case([[0.6] * 14, [0.3] * 14, [0.01] * 14])
    check(any("初期位置に戻した" in m for m in logs) and t < 0.2, f"腕が 0 に着いたら待つのをやめて「戻した」と出す（{t:.2f} 秒）")
    t, logs = wait_case([[0.5] * 14])
    check(any(m.startswith("警告") and "戻りきらない" in m for m in logs) and 4.9 <= t <= 5.2,
          f"着かないときは 5 秒で先へ進み、警告を出す（{t:.2f} 秒）")


def main() -> int:
    p = argparse.ArgumentParser(description="arm-home-hold／arm-home-wait の自己テスト")
    p.add_argument("--arm", type=Path, default=TELEOP / "robot_control" / "robot_arm.py")
    p.add_argument("--teleop", type=Path, default=TELEOP / "teleop_hand_and_arm.py")
    args = p.parse_args()
    src = load_clip(args.arm)
    if "PATCH(arm-home-hold)" not in src:
        print(f"[自己テスト] ✗ arm-home-hold が当たっていません: {args.arm}")
        return 1
    ns = {"np": np}
    exec(src, ns)
    clip = ns["clip_arm_q_target"]
    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    # 1. 垂れた姿勢から 2 rad/s で 0 へ（起動・終わり）
    log = run(clip, Arm(drooped()), SLOW, 3.0)
    reach = next((n * DT for n, (_, q) in enumerate(log) if np.max(np.abs(q)) < 0.05), None)
    err = np.max(np.abs(log[-1][1]))
    check(reach is not None and err < 0.05,
          f"垂れた姿勢（肘 0.9・肩 0.4 rad）から 0 の姿勢に届く（{reach if reach is not None else '—'} 秒・3 秒後のずれ {err:.3f} rad）")
    old = run(upstream_clip, Arm(drooped()), SLOW, 3.0)
    print(f"    参考: 上流の寄せ方では 3 秒後のずれ {np.max(np.abs(old[-1][1])):.2f} rad（0 へ届かない）")
    check(max(np.max(np.abs(c - q)) for c, q in log) <= 0.1 + 1e-9, "寄せる間も、目標と実測の差は 0.1 rad まで")

    # 2. 0 に着いた後、保ち続ける（保持係が保つのはこの指令）
    held = log[-1][0]
    check(np.max(np.abs(held)) < 1e-6, f"着いた後の指令は 0 の姿勢そのもの（{np.max(np.abs(held)):.1e} rad）")

    # 3. ぶつかって動けない（左肘が 0.5 rad より 0 の側へ行けない）
    arm = Arm(drooped(), stop={3: 0.5})
    blocked = run(clip, arm, SLOW, 3.0)
    gap = max(abs(c[3] - q[3]) for c, q in blocked)
    check(gap <= 0.1 + 1e-9 and abs(arm.q[3] - 0.5) < 1e-6,
          f"ぶつかったら目標と実測の差を 0.1 rad までにする（最大 {gap:.3f} rad＝{KP * gap:.1f} N·m）")
    check(np.max(np.abs(np.delete(arm.q, 3))) < 0.05, "ぶつかった関節のほかは 0 に届く")

    # 4. 追従中（30 rad/s）は上流と同じ
    rng = np.random.default_rng(0)
    same = True
    for _ in range(200):
        a = Arm(rng.uniform(-1, 1, 14))
        target = rng.uniform(-1.5, 1.5, 14)
        same &= np.allclose(clip(a, target, FAST), upstream_clip(a, target, FAST))
    check(bool(same), "追従中（30 rad/s）は上流と同じ値を返す（200 通り）")

    # 5. 速いときから遅いときへ切り替わった最初の周期は、前に送った指令から寄せる
    a = Arm(np.full(14, 0.3))
    first = clip(a, np.full(14, 0.3), FAST)          # 追従中の最後の指令（実測と同じ）
    a.q = np.full(14, 0.35)                           # 重さで少し垂れた
    nxt = clip(a, np.zeros(14), SLOW)
    check(np.allclose(nxt, first - SLOW * DT), f"遅いときは前の指令から寄せる（実測 0.35 でなく 0.30 から {SLOW * DT:.3f} rad）")

    check_wait(args.teleop, check)
    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
