#!/usr/bin/env python3
"""案 A・A4 の一部（2026-10-27）: 腕の逆運動学（G1_29_ArmIK.solve_ik）の時間を、機体なしで測る。

遠隔操作の 1 周期でいちばん重い計算。PC2（Orin NX）で 30 Hz（33 ms）に収まるかを見る。
機体には何も送らない（DDS を使わない）。

    ~/miniforge3/envs/tv/bin/python ik_bench.py [--seconds 20] [--hz 30]

目標の手首は、腕の関節を可動域の中でゆっくり動かしたときの手首の位置（順運動学）から作る
（届く姿勢だけにし、遠隔操作のように前の周期の近くを追う形にするため）。
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

XR = Path.home() / "xr_teleoperate"
sys.path.insert(0, str(XR))
sys.path.insert(0, str(XR / "teleop"))


def main() -> int:
    p = argparse.ArgumentParser(description="腕の逆運動学の時間を測る（機体なし）")
    p.add_argument("--seconds", type=float, default=20.0, help="模擬する操作の長さ")
    p.add_argument("--hz", type=float, default=30.0, help="遠隔操作の周期（既定 30 Hz）")
    args = p.parse_args()

    # 逆運動学はモデルを '../assets/g1/...'・写しを 'g1_29_model_cache.pkl' と、今いるフォルダからの場所で
    # 読むので、遠隔操作と同じ teleop/ に移る（写しも遠隔操作と共用になる）
    os.chdir(XR / "teleop")
    import pinocchio as pin
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

    t0 = time.perf_counter()
    ik = G1_29_ArmIK()
    print(f"[ik] 逆運動学の準備 {time.perf_counter() - t0:.1f} 秒（初回はモデルの写しを作るので長い）")

    model = ik.reduced_robot.model
    data = model.createData()
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    mid, amp = (lo + hi) / 2, (hi - lo) / 2 * 0.4            # 可動域の真ん中 ±40%
    l_id, r_id = model.getFrameId("L_ee"), model.getFrameId("R_ee")
    rng = np.random.default_rng(0)
    freq = rng.uniform(0.05, 0.3, model.nq)                    # 関節ごとにゆっくり違う速さで動かす
    phase = rng.uniform(0, 2 * math.pi, model.nq)

    n = int(args.seconds * args.hz)
    q_now = mid.copy()
    times = []
    for k in range(n):
        t = k / args.hz
        q_goal = mid + amp * np.sin(2 * math.pi * freq * t + phase)
        pin.framesForwardKinematics(model, data, q_goal)
        left = data.oMf[l_id].homogeneous.copy()
        right = data.oMf[r_id].homogeneous.copy()
        s = time.perf_counter()
        sol_q, _ = ik.solve_ik(left, right, q_now, np.zeros(model.nq))
        times.append((time.perf_counter() - s) * 1000)
        q_now = sol_q

    a = np.array(times)
    budget = 1000 / args.hz
    print(f"[ik] {n} 回（{args.seconds:.0f} 秒ぶん・{args.hz:.0f} Hz）: "
          f"中央 {np.median(a):.1f} ms・95% {np.percentile(a, 95):.1f} ms・最大 {a.max():.1f} ms")
    print(f"[ik] 1 周期の持ち時間 {budget:.1f} ms に対して、超えた回 {int((a > budget).sum())} 回・"
          f"半分（{budget / 2:.1f} ms）を超えた回 {int((a > budget / 2).sum())} 回")
    return 0


if __name__ == "__main__":
    sys.exit(main())
