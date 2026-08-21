#!/usr/bin/env python
"""前回の検証(脚関節dq/gyro.z)は、立位バランス制御自体の揺れが大きく指令の有無で
有意差が出なかった(RESULT_INCONCLUSIVE)。そこで、GrootLocomotionController内部の
self.cmd = [vx, vy, theta_dot] を直接読み取り、send_action() で送った
remote.lx/ly/rx が実際にコントローラへ反映されているかを一次情報として確認する。
(gr00t_locomotion.py: cmd[0]=ly, cmd[1]=-lx, cmd[2]=-rx)
"""
import os
import sys
import time

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_utils import default_remote_input


def send_and_read(robot, label, overrides, settle_s=0.5):
    action = default_remote_input()
    action.update(overrides)
    t_end = time.perf_counter() + settle_s
    while time.perf_counter() < t_end:
        robot.send_action(dict(action))
        time.sleep(0.02)
    cmd = getattr(robot.controller, "cmd", None)
    cmd_list = list(cmd) if cmd is not None else None
    print(f"[{label}] sent={overrides}  controller.cmd(vx,vy,theta_dot)={cmd_list}", flush=True)
    return cmd_list


def main():
    cfg = UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController")
    robot = UnitreeG1(cfg)
    print("Connecting...", flush=True)
    robot.connect()
    print("Connected.", flush=True)
    time.sleep(1.0)

    results = {}
    results["zero"] = send_and_read(robot, "ZERO", {})
    results["forward"] = send_and_read(robot, "FORWARD ly=0.4", {"remote.ly": 0.4})
    results["strafe"] = send_and_read(robot, "STRAFE lx=0.3", {"remote.lx": 0.3})
    results["turn"] = send_and_read(robot, "TURN rx=-0.4", {"remote.rx": -0.4})
    results["zero2"] = send_and_read(robot, "ZERO-2", {})

    print("\n=== SUMMARY ===", flush=True)
    for k, v in results.items():
        print(f"{k:10s} -> {v}", flush=True)

    ok = (
        results["zero"] == [0.0, 0.0, 0.0]
        and abs(results["forward"][0] - 0.4) < 1e-6
        and abs(results["strafe"][1] - (-0.3)) < 1e-6
        and abs(results["turn"][2] - 0.4) < 1e-6
        and results["zero2"] == [0.0, 0.0, 0.0]
    )
    print(f"\nremote.*入力がcontroller.cmd(vx,vy,theta_dot)に期待通り反映された: {ok}", flush=True)
    print("RESULT_OK" if ok else "RESULT_MISMATCH", flush=True)

    print("Disconnecting...", flush=True)
    try:
        robot.disconnect()
    except Exception as e:  # noqa: BLE001
        print(f"disconnect() error (ignored): {e}", flush=True)
    print("DONE", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
