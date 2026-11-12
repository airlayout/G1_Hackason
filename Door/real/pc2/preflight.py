"""11-12 夜（作業ログ 2026-11-12 の 6 章 ㋓・7 章の 3）: 走らせ役を起こす直前の、機体の状態の確かめ（読むだけ。何も送らない）。

rt/lowstate を SECS 秒受けて、腕の関節の温度・状態の札、脚の関節の揺れ、IMU の傾きを見る。preflight.sh が呼ぶ。
11-12 の機体では、右足首 roll が腕を借りる前から断続的に 0.5〜0.7 rad/s で揺れ、d を十数回断られた（約 4 分）。

  python3 preflight.py [--iface eth0] [--secs 2] [--arm-temp-warn 70]
終わりの行が「[preflight] 機体 OK」か「[preflight] 機体 ⚠️ …」。ほかの台本が読む
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

import door_run_real as drr
import door_run_stock as drs

LEG_SHAKE = 0.2      # [rad/s] 足の荷重の確かめが「動いている」とみなす線（door_foot_load と同じ）
SHAKE_FRAC = 0.05    # この割合を越える周期で脚のどれかが線を越えていたら「揺れている」


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--secs", type=float, default=2.0)
    ap.add_argument("--arm-temp-warn", type=float, default=70.0)
    a = ap.parse_args(argv)
    io = drr.RobotIO(a.iface, send=False, use_odom=False)
    t0 = time.monotonic()
    temps, flags, legs, tilts = [], [], [], []
    while time.monotonic() - t0 < a.secs:
        st, st_t, _, _ = io.snapshot()
        if st is not None and time.monotonic() - st_t < 0.2:
            temp, j_t, j_flag = drs.arm_motor_health(st)
            temps.append((temp, j_t))
            flags.append(j_flag)
            legs.append([abs(st.motor_state[j].dq) for j in range(12)])
            g = drr.dc.quat_wxyz_to_gravity(np.array(st.imu_state.quaternion, dtype=float))
            tilts.append(math.degrees(math.acos(max(-1.0, min(1.0, -g[2])))))
        time.sleep(0.02)
    if not temps:
        print("[preflight] 機体 ⚠️ rt/lowstate が来ない（電源・eth0・純正の制御を確かめる）", flush=True)
        return 2
    legs = np.array(legs)
    tmax, jt = max(temps)
    shake = (legs > LEG_SHAKE).any(axis=1).mean()
    worst = int(np.argmax((legs > LEG_SHAKE).mean(axis=0)))
    flag = next((f for f in flags if f is not None), None)
    print(f"[preflight] 腕の温度 最高 {tmax:.0f} °C（関節 {jt}）・状態の札 {'関節 ' + str(flag) if flag is not None else '無し'}・"
          f"脚が {LEG_SHAKE} rad/s を越えた周期 {100 * shake:.0f}%（多いのは関節 {worst}）・傾き 中央 {np.median(tilts):.1f}°", flush=True)
    bad = []
    if tmax >= a.arm_temp_warn:
        bad.append(f"腕が {tmax:.0f} °C（{a.arm_temp_warn:.0f} °C 以上。冷えるまで待つ）")
    if flag is not None:
        bad.append(f"腕の関節 {flag} の状態の札が立っている")
    if shake > SHAKE_FRAC:
        bad.append(f"脚の関節 {worst} が揺れている（{100 * shake:.0f}% の周期。機体を少し動かすと止まることがある・11-12）")
    if np.median(tilts) > 8.0:
        bad.append(f"傾き {np.median(tilts):.1f}°")
    print("[preflight] 機体 " + ("OK" if not bad else "⚠️ " + "・".join(bad)), flush=True)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
