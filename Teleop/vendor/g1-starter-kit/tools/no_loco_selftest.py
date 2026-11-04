#!/usr/bin/env python3
"""[G1_Hackason 改変] no-loco（足なし）の自己テスト（機体・Quest 不要。2026-11-03）。

修正済みの teleop_hand_and_arm.py から、--no-loco の add_argument の行・--motion なしで断る行・
歩きの指令を作る数行（loco_vx = … から loco_wrapper.Move まで）を抜き出し、偽の Quest・偽の機体で回す。
setup/apply_patches.py で当てた後に流す。歩きの数行の回し方は loco_fwd_max_selftest.py と同じものを使う。

    python3 tools/no_loco_selftest.py
    python3 tools/no_loco_selftest.py --file <teleop_hand_and_arm.py>

確かめること:
  1. 既定（--no-loco なし）は足あり。前へ倒しきると上流と同じ 0.3 m/s を送る
  2. --no-loco では、どの向きに倒しても Move を一度も呼ばず、記録の歩き（loco_cmd_sent）は 0
  3. --no-loco でも --loco-fwd-max は読める（足なしでは使われないだけ）
  4. --no-loco を --motion なしで渡すと、起動の時点で断る
"""

import argparse
import sys
import types
from pathlib import Path

from loco_fwd_max_selftest import extract, step

DEFAULT_FILE = Path.home() / "xr_teleoperate" / "teleop" / "teleop_hand_and_arm.py"


def load_flag_parser(text: str) -> argparse.ArgumentParser:
    """--no-loco の add_argument の行だけで引数の読み手を作る（--motion は上流と同じ形で足す）。"""
    ns = {"parser": argparse.ArgumentParser()}
    ns["parser"].add_argument("--motion", action="store_true")
    line = next(ln for ln in text.splitlines() if "'--no-loco'" in ln)
    exec(line.strip(), ns)
    return ns["parser"]


def refused_without_motion(text: str, motion: bool, no_loco: bool) -> bool:
    """--motion なしで --no-loco を断る 2 行を回し、parser.error が呼ばれたかを返す。"""
    src = extract(text, "if args.no_loco and not args.motion:", "parser.error(")
    called = []
    ns = {"args": types.SimpleNamespace(motion=motion, no_loco=no_loco),
          "parser": types.SimpleNamespace(error=called.append)}
    exec(src, ns)
    return bool(called)


def main() -> int:
    p = argparse.ArgumentParser(description="no-loco の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    a = p.parse_args()
    text = a.file.read_text(encoding="utf-8")
    if "PATCH(no-loco)" not in text:
        print(f"[自己テスト] ✗ no-loco が当たっていません: {a.file}")
        return 1
    src = extract(text, "loco_vx = -tele_data.left_ctrl_thumbstickValue[1]", "loco_wrapper.Move(*loco_cmd_sent)")
    parser = load_flag_parser(text)

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    check(parser.parse_args(["--motion"]).no_loco is False, "既定は足あり（no_loco=False）")
    check(parser.parse_args(["--motion", "--no-loco"]).no_loco is True, "--no-loco を読める")

    sent, rec = step(src, 0.3, (0.0, -1.0))
    check(sent == (0.3, 0.0, 0.0) and rec == sent, f"足ありで前へ倒しきると 0.3 m/s を送り、記録も同じ（{sent}）")

    for left, right_x, label in (((0.0, -1.0), 0.0, "前"), ((0.0, 1.0), 0.0, "後ろ"),
                                 ((-1.0, 0.0), 0.0, "横"), ((0.0, 0.0), 1.0, "旋回")):
        sent, rec = step(src, 0.6, left, right_x=right_x, no_loco=True)
        check(sent is None and rec == (0.0, 0.0, 0.0), f"足なしで{label}へ倒しても Move を呼ばず、記録の歩きは 0（{sent}, {rec}）")
    sent, rec = step(src, 0.3, (0.0, -1.0), paused=True, no_loco=True)
    check(sent is None and rec == (0.0, 0.0, 0.0), "足なしの一時停止中も Move を呼ばない")

    check(refused_without_motion(text, motion=False, no_loco=True), "--motion なしの --no-loco は起動の時点で断る")
    check(not refused_without_motion(text, motion=True, no_loco=True), "--motion ありの --no-loco は通す")
    check(not refused_without_motion(text, motion=False, no_loco=False), "--no-loco なしは今までどおり通す")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
