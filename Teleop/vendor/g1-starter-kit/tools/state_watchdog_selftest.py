#!/usr/bin/env python3
"""state-watchdog パッチが実際に発火するかを、実機なしで確かめる。

    python3 tools/state_watchdog_selftest.py

ロボットも DDS の相手も要らない。パッチ後の `_ctrl_motor_state()` を
そのまま動かし、lowstate の鮮度だけを人工的に古くして挙動を見る。

確認する 2 点:
  ① 鮮度が新しい間は rt/lowcmd の送信が続く（誤発火しない）
  ② 鮮度が STATE_WATCHDOG_TIMEOUT_S を超えると送信が止まり、
     プロセスが終了コード 1 で強制終了する

子プロセスで実行するのは、パッチが os._exit() でプロセスを落とすため。
"""
import os
import subprocess
import sys
import textwrap
from pathlib import Path

XR = Path.home() / "xr_teleoperate"

CHILD = r'''
import sys, threading, time
sys.path.insert(0, %(xr)r)
from teleop.robot_control import robot_arm as RA
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

assert hasattr(RA, "_state_watchdog_fail_closed"), "パッチ未適用: helper が無い"
assert hasattr(RA, "STATE_WATCHDOG_TIMEOUT_S"), "パッチ未適用: 定数が無い"
TIMEOUT = RA.STATE_WATCHDOG_TIMEOUT_S
print("THRESHOLD=%%.3f" %% TIMEOUT, flush=True)

sent = {"n": 0}
class PubStub:
    def Write(self, msg):
        sent["n"] += 1

# __init__ を通さずに、送信ループが使う属性だけを与える
c = RA.G1_29_ArmController.__new__(RA.G1_29_ArmController)
c.motion_mode = False
c.simulation_mode = True          # clip_arm_q_target を通さない（lowstate 不要）
c.ctrl_lock = threading.Lock()
c.q_target = [0.0] * 14
c.tauff_target = [0.0] * 14
c.control_dt = 0.002
c.crc = CRC()
c.msg = unitree_hg_msg_dds__LowCmd_()
c.lowcmd_publisher = PubStub()
c.lowstate_last_time = time.time()          # まずは新鮮な状態

threading.Thread(target=c._ctrl_motor_state, daemon=True).start()

# ① 新鮮な間は送信が続くこと
time.sleep(0.15)
fresh = sent["n"]
print("FRESH_SENT=%%d" %% fresh, flush=True)
if fresh <= 0:
    print("FAIL: 新鮮なのに送信されていない", flush=True); sys.exit(9)

# ② 鮮度を古くすると止まって落ちること
c.lowstate_last_time = time.time() - (TIMEOUT + 0.5)
t0 = time.time()
while time.time() - t0 < 3.0:
    time.sleep(0.01)
# ここに到達したら強制終了していない
print("STALE_SENT=%%d" %% sent["n"], flush=True)
print("FAIL: 途絶させてもプロセスが生きている", flush=True)
sys.exit(9)
''' % {"xr": str(XR)}


def main() -> int:
    proc = subprocess.run([sys.executable, "-c", CHILD],
                          capture_output=True, text=True, timeout=120)
    out = proc.stdout + proc.stderr
    print(textwrap.indent(out.strip(), "    "))
    print()

    ok = True

    def check(label, cond, detail=""):
        nonlocal ok
        print(f"  {'✅' if cond else '❌'} {label}" + (f"  … {detail}" if detail else ""))
        if not cond:
            ok = False

    check("パッチが適用されている", "THRESHOLD=" in out)
    check("鮮度が新しい間は送信される（誤発火しない）",
          any(l.startswith("FRESH_SENT=") and int(l.split("=")[1]) > 0
              for l in out.splitlines()))
    check("途絶で強制終了する（終了コード 1）", proc.returncode == 1,
          f"実際の終了コード={proc.returncode}")
    check("途絶を知らせるメッセージが出る", "[state-watchdog]" in out)
    check("送信を止めてから落ちている（STALE_SENT が出ない）",
          "STALE_SENT=" not in out)

    print()
    print("  結果: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
