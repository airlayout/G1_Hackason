#!/usr/bin/env python3
"""案 A（2026-10-26）: PC2 の上で、切り離して動く xr_teleoperate に IPC で r・s・q を送る。

Mac の scripts/pc2_teleop.sh から ssh で呼ばれる。IPC は同じ機械の中だけの口
（ipc://@xr_teleoperate_data.ipc）なので、PC2 の上で動かす必要がある。

    python3 teleop_ipc.py state      # 状態を見るだけ
    python3 teleop_ipc.py start      # 追従の開始（キーボードの r）
    python3 teleop_ipc.py record     # 記録の開始／保存（s。追従の開始の後だけ効く）
    python3 teleop_ipc.py stop       # 終える（q。⚠️ 遠隔操作の後始末で腕を下ろす）

送った後に、心拍（1 秒に 10 回）で状態が本当に変わったかを確かめる。変わらなければ終了コード 1。
"""

import argparse
import sys
import time
from pathlib import Path

XR = Path.home() / "xr_teleoperate"
sys.path.insert(0, str(XR))
from teleop.utils.ipc import IPC_Client  # noqa: E402

CMDS = {"start": "CMD_START", "record": "CMD_RECORD_TOGGLE", "stop": "CMD_STOP"}
SETTLE_S = 0.6   # 送ってから状態を読み直すまで（心拍 10 Hz の数回ぶん）


def describe(state: dict) -> str:
    if not state:
        return "（応答なし）"
    return (f"追従 {'中' if state.get('START') else 'まだ'}・"
            f"記録の準備 {'よし' if state.get('READY') else 'まだ'}・"
            f"記録 {'中' if state.get('RECORD_RUNNING') else 'していない'}"
            f"{'・終了の途中' if state.get('STOP') else ''}")


def wait_online(client: IPC_Client, timeout: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if client.is_online():
            return True
        time.sleep(0.05)
    return False


def main() -> int:
    p = argparse.ArgumentParser(description="xr_teleoperate に IPC で r・s・q を送る")
    p.add_argument("action", choices=["state", *CMDS])
    p.add_argument("--timeout", type=float, default=3.0, help="遠隔操作の応答を待つ秒")
    args = p.parse_args()

    client = IPC_Client()
    try:
        if not wait_online(client, args.timeout):
            print("[ipc] ✗ 遠隔操作から応答が無い（動いていないか、起動の途中）")
            return 2
        before = client.latest_state()
        if args.action == "state":
            print(f"[ipc] 状態: {describe(before)}")
            return 0
        if args.action == "record" and not before.get("START"):
            print("[ipc] ✗ 追従がまだ始まっていない。先に go（r）")
            return 1

        reply = client.send_data(CMDS[args.action])
        if reply.get("status") != "ok":
            print(f"[ipc] ✗ 受け付けられなかった: {reply.get('msg')}")
            return 1
        time.sleep(SETTLE_S)
        after = client.latest_state() if client.is_online() else {}

        if args.action == "start":
            ok = bool(after.get("START"))
        elif args.action == "record":
            ok = bool(after) and after.get("RECORD_RUNNING") != before.get("RECORD_RUNNING")
        else:
            ok = (not after) or bool(after.get("STOP"))
        mark = "✓" if ok else "⚠️"
        print(f"[ipc] {mark} {args.action} を送った。状態: {describe(after) if after else '（終了した）'}")
        if args.action == "record" and ok:
            print(f"[ipc]   → 記録を{'始めた' if after.get('RECORD_RUNNING') else '保存した'}")
        return 0 if ok else 1
    finally:
        client.stop()


if __name__ == "__main__":
    sys.exit(main())
