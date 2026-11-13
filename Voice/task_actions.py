"""登録済みの処理を、ロボット制御側の Python で起動する。"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
AUTO_PRESS_SCRIPT = REPO_ROOT / "Button_Press/J1-gen/real/auto_press.py"
DEFAULT_ROBOT_PYTHON = REPO_ROOT / "G1_HuggingFace/venv/bin/python"

# 設定ファイルはこの名前だけを指定する。コマンドや任意の引数はコード側で管理する。
TASK_ARGS: dict[str, tuple[str, ...]] = {
    "press_bottle": ("--target", "bottle"),
    "press_down": ("--target", "down", "--button-set", "wheelchair", "--detector", "yolo"),
}


@dataclass(frozen=True)
class ActionResult:
    command: tuple[str, ...]
    status: str
    returncode: int | None


def build_command(
    action: str, robot_python: Path = DEFAULT_ROBOT_PYTHON,
    network_interface: str | None = None, *, execute: bool = False,
) -> tuple[str, ...]:
    if action not in TASK_ARGS:
        raise ValueError(f"未登録の実行処理です: {action}")
    # venv/bin/python の symlink を解決すると仮想環境が外れるので absolute() を使う。
    command = [str(robot_python.expanduser().absolute()), str(AUTO_PRESS_SCRIPT), *TASK_ARGS[action]]
    if network_interface:
        command.extend(("--network-interface", network_interface))
    if execute:
        command.append("--execute")
    return tuple(command)


def run_action(
    action: str, robot_python: Path = DEFAULT_ROBOT_PYTHON,
    network_interface: str | None = None, *, execute: bool = False,
) -> ActionResult:
    command = build_command(action, robot_python, network_interface, execute=execute)
    # dry-run はコマンド表示まで。既存タスクの起動・DDS 接続も行わない。
    if not execute:
        return ActionResult(command, "dry_run", None)
    if not Path(command[0]).is_file():
        raise ValueError("制御側の Python がありません。--robot-python で指定してください")
    import fcntl

    lock_path = REPO_ROOT / "_local/voice/task.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("別の音声指示がタスクを実行中です") from exc
        # 同じ foreground process group に置き、Ctrl+C を既存の停止処理にも届ける。
        process = subprocess.Popen(command, cwd=REPO_ROOT)
        while True:
            try:
                code = process.wait()
                return ActionResult(command, "process_exited", code)
            except KeyboardInterrupt:
                print("[voice] タスクの停止処理が終わるのを待ちます", file=sys.stderr)
