"""柱に寄って（real/approach.py）、続けてボタンを押す（real/auto_press.py）を 1 コマンドで行う。

    # 送らずに確かめる（既定。approach は 1 回観測するだけ。もう押せる位置のときだけ auto_press も dry-run で計画まで）
    python real/approach_and_press.py --target down --button-set wheelchair --detector yolo
    # 実機で寄って押す（PC2 の上。approach の Enter 待ち・auto_press の事前確認はそのまま出る）
    ~/button_press/venv/bin/python real/approach_and_press.py --target down --button-set wheelchair --detector yolo --arms right left --network-interface eth0 --camera-config camera_onboard.yaml --execute
    # Enter なしで、寄って押すまで続ける（リモコンを持った人がそばにいるときだけ）
    ~/button_press/venv/bin/python real/approach_and_press.py --target down --button-set wheelchair --detector yolo --arms right left --network-interface eth0 --camera-config camera_onboard.yaml --no-confirm --execute
    # 声つき（寄る・押すの両方。先に real/voice_phrases.py make・send）
    ~/button_press/venv/bin/python real/approach_and_press.py --target down --button-set wheelchair --detector yolo --arms right left --network-interface eth0 --camera-config camera_onboard.yaml --voice local --execute
    # 寄り終わってから、押す前にもう 1 回 Enter を待つ
    python real/approach_and_press.py --target up --execute --press-confirm
    # auto_press.py に追加の引数を渡す（--press-args 以降はすべて auto_press.py へ）
    python real/approach_and_press.py --target up --execute --press-args --voice local --max-joint-speed 0.8

押すのは approach が ready（押せる位置・腕が決まった）で終わったときだけ。中止・見つからない・Ctrl+C のときは押さない。
Ctrl+C は子プロセス（同じプロセスグループ）に届く。親は子を止めずに終わるのを待ち、summary.json を書いてから終わる。
記録は _local/button_press/approach_press/<時刻>/ （approach_result.json と summary.json）。
子の安全確認（Enter 待ち・事前確認）は消さない・飛ばさない（--skip-prechecks は渡さない）。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT  # noqa: E402

REAL_DIR = Path(__file__).resolve().parent
APPROACH_PY = REAL_DIR / "approach.py"
AUTO_PRESS_PY = REAL_DIR / "auto_press.py"


def build_approach_cmd(args: argparse.Namespace, result_json: Path, python: str = sys.executable) -> list[str]:
    """approach.py の argv。--webcam は approach にだけ渡す。"""
    cmd = [python, str(APPROACH_PY), "--target", args.target, "--result-json", str(result_json)]
    if args.button_set:
        cmd += ["--button-set", args.button_set]
    if args.detector:
        cmd += ["--detector", args.detector]
    if args.arms:
        cmd += ["--arms", *args.arms]
    if args.network_interface:
        cmd += ["--network-interface", args.network_interface]
    if args.camera_config:
        cmd += ["--camera-config", args.camera_config]
    if args.webcam:
        cmd.append("--webcam")
    if args.no_confirm:
        cmd.append("--no-confirm")
    if args.voice:
        cmd += ["--voice", args.voice]
    if args.execute:
        cmd.append("--execute")
    return cmd


def build_press_cmd(args: argparse.Namespace, arm: str, python: str = sys.executable) -> list[str]:
    """auto_press.py の argv。腕は approach が決めたものを渡す。"""
    cmd = [python, str(AUTO_PRESS_PY), "--target", args.target, "--arm", arm]
    if args.button_set:
        cmd += ["--button-set", args.button_set]
    if args.detector:
        cmd += ["--detector", args.detector]
    if args.network_interface:
        cmd += ["--network-interface", args.network_interface]
    if args.camera_config:
        cmd += ["--camera-config", args.camera_config]
    if args.voice:
        cmd += ["--voice", args.voice]
    if args.execute:
        cmd.append("--execute")
    cmd += list(args.press_args or [])
    return cmd


def decide_press(result: dict[str, Any] | None, rc: int, execute: bool) -> tuple[bool, str]:
    """approach の結果から、押す側を実行するか（実行するか, 表示する理由）。

    ready（common/approach.py の Approacher.run が decide の action == "ready" で返す）は、--execute の有無にかかわらず
    「もう押せる位置」の意味で arm が入る。dry-run で歩く必要があるときは outcome == "dry_run"（次の 1 回を表示）で終わる。
    """
    if result is None:
        return False, f"approach の結果 JSON を読めない（終了コード {rc}）"
    outcome, arm = result.get("outcome"), result.get("arm")
    if result.get("interrupted") or rc == 130:
        return False, "Ctrl+C で止めた"
    if outcome == "ready":
        if arm in ("right", "left"):
            return True, f"押せる位置（腕: {arm}）"
        return False, "ready だが押す腕が入っていない"
    if outcome == "dry_run":
        return False, f"寄る必要がある（次の動き: {result.get('reason')}）ので押す dry-run はしない"
    return False, f"{outcome}: {result.get('reason')}"


def read_result(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def run_child(cmd: list[str]) -> int:
    """子を端末につないで実行する。Ctrl+C は子にも届くので、親は止めずに子の終了を待つ。"""
    print(f"[approach_press] 実行: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd)
    interrupted = False
    while True:
        try:
            return proc.wait()
        except KeyboardInterrupt:
            if not interrupted:
                print("[approach_press] ⚠️ Ctrl+C: 子の終了を待つ（子が安全に止まる）")
            interrupted = True


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", choices=["up", "down"], required=True, help="押すボタン")
    p.add_argument("--button-set", choices=["general", "wheelchair"], help="押す組（既定は elevator.yaml）")
    p.add_argument("--detector", choices=["classic", "yolo"], help="▲▼ の見つけ方（既定は elevator.yaml）")
    p.add_argument("--arms", nargs="+", choices=["right", "left"], help="approach で計画を試す腕")
    p.add_argument("--network-interface", help="G1 につないでいる NIC")
    p.add_argument("--camera-config", help="カメラの接続先の設定（例 camera_onboard.yaml）")
    p.add_argument("--execute", action="store_true", help="実機で歩いて押す（付けなければ dry-run）")
    p.add_argument("--webcam", action="store_true", help="approach にだけ渡す（前方の Web カメラも使う）")
    p.add_argument("--voice", choices=["local", "ssh"],
                   help="寄る（歩き始め・着いた・中止）と押す（押します・点灯成功など）の両方で喋らせる。PC2 の上なら local")
    p.add_argument("--no-confirm", action="store_true",
                   help="approach の歩き始める前の Enter も待たない（--press-confirm を付けなければ、Enter なしで押すまで続く）")
    p.add_argument("--press-confirm", action="store_true", help="寄り終わってから、押す前に Enter を待つ（既定は続けて押す）")
    p.add_argument("--press-args", nargs=argparse.REMAINDER, default=[],
                   help="以降の引数をそのまま auto_press.py に渡す（最後に書く）")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out = REPO_ROOT / "_local" / "button_press" / "approach_press" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    result_json = out / "approach_result.json"
    summary: dict[str, Any] = {"execute": args.execute, "approach": {}, "press": {"ran": False}}

    def save(rc: int) -> int:
        (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        print(f"[approach_press] 保存: {out / 'summary.json'}")
        return rc

    cmd = build_approach_cmd(args, result_json)
    rc = run_child(cmd)
    result = read_result(result_json)
    summary["approach"] = {"argv": cmd, "returncode": rc,
                           **{k: (result or {}).get(k) for k in ("outcome", "reason", "arm", "interrupted")}}
    go, why = decide_press(result, rc, args.execute)
    summary["press"]["reason"] = why
    if not go:
        print(f"[approach_press] 押さずに終わる: {why}")
        return save(rc if rc != 0 else (0 if result and result.get("outcome") == "dry_run" else 1))
    print(f"[approach_press] {why}")
    if args.execute and args.press_confirm:
        try:
            input("[approach_press] 押す前の確認。押してよければ Enter（やめるなら Ctrl+C）: ")
        except (KeyboardInterrupt, EOFError):
            print("[approach_press] 押さずに終わる（確認で止めた）")
            summary["press"]["reason"] = "押す前の確認で止めた"
            return save(130)
    press_cmd = build_press_cmd(args, str(result["arm"]))
    prc = run_child(press_cmd)
    summary["press"] = {"ran": True, "argv": press_cmd, "returncode": prc, "reason": why}
    return save(prc)


if __name__ == "__main__":
    sys.exit(main())
