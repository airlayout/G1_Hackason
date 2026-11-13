#!/usr/bin/env python3
"""登録済みの腕プリセットを、制御モードを切り替えずに実行するツール用 CLI。"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

PRESETS = {"high wave": 26}
SUPPORTED_FSM_IDS = {500, 501, 801}
LOCK_DIRECTORY = Path(tempfile.gettempdir()) / f"g1-voice-{os.getuid()}"
# 完了通知ではなく、連続要求を重ねないための待ち時間。
HOLD_SECONDS = 5.0


@contextmanager
def arm_lock():
    LOCK_DIRECTORY.mkdir(mode=0o700, parents=True, exist_ok=True)
    directory = LOCK_DIRECTORY.stat()
    if directory.st_uid != os.getuid() or directory.st_mode & 0o077:
        raise ValueError("腕ツールのロックは自分が所有する mode 700 のディレクトリに置いてください")
    with (LOCK_DIRECTORY / "arm-action.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("別の腕プリセットを実行中です") from exc
        yield


def connect_sdk(interface: str):
    # dry-run とヘルプは、SDK のインストールも DDS 接続も不要。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, interface)
    loco, arm = LocoClient(), G1ArmActionClient()
    for client in (loco, arm):
        client.SetTimeout(3.0)
        client.Init()
    return loco, arm


def run_preset(preset: str, interface: str, *, execute: bool = False,
               connect=None, wait=None) -> dict:
    if preset not in PRESETS:
        raise ValueError(f"未登録の腕プリセットです: {preset}")
    if not isinstance(interface, str) or not interface.strip():
        raise ValueError("ネットワークインターフェースを指定してください")
    result = {"preset": preset, "action_id": PRESETS[preset], "interface": interface,
              "status": "dry_run", "motion_completed": False}
    if not execute:
        return result
    with arm_lock():
        loco, arm = (connect or connect_sdk)(interface)
        code, fsm_id = loco.GetFsmId()
        if code != 0 or type(fsm_id) is not int or fsm_id not in SUPPORTED_FSM_IDS:
            raise RuntimeError(f"腕プリセットを実行できるモードではありません: code={code}, fsm_id={fsm_id}")
        code = arm.ExecuteAction(PRESETS[preset])
        if code != 0:
            raise RuntimeError(f"ExecuteAction が失敗しました: code={code}")
        (wait or time.sleep)(HOLD_SECONDS)
    # code=0 は SDK が要求を受理した結果。物理動作の完了とは区別する。
    return {**result, "status": "accepted", "sdk_code": code, "fsm_id": fsm_id}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=sorted(PRESETS), default="high wave")
    parser.add_argument("--network-interface", default=os.environ.get("VOICE_SDK_INTERFACE", "eth0"))
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_preset(args.preset, args.network_interface, execute=args.execute)
    except KeyboardInterrupt:
        print(json.dumps({"status": "interrupted", "motion_completed": False}), flush=True)
        return 130
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "error", "message": str(exc)}, ensure_ascii=False), file=sys.stderr, flush=True)
        return 2
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    def stop_on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_on_signal)
    raise SystemExit(main())
