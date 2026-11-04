#!/usr/bin/env python3
"""[G1_Hackason 改変] loco-fwd-max の自己テスト（機体・Quest 不要）。

修正済みの teleop_hand_and_arm.py から、--loco-fwd-max の値の確かめ（_loco_fwd_max と add_argument の行）と、
歩きの指令を作る数行（loco_vx = … から loco_wrapper.Move まで）を抜き出し、偽の Quest・偽の機体で回す。
setup/apply_patches.py で当てた後に流す。

    python3 tools/loco_fwd_max_selftest.py
    python3 tools/loco_fwd_max_selftest.py --file <teleop_hand_and_arm.py>

確かめること:
  1. 既定（--loco-fwd-max を渡さない）では、前・後ろ・横・旋回とも上流と同じ 0.3
  2. --loco-fwd-max 0.6 で、スティックを前へ倒しきると前へ 0.6 m/s、半分なら 0.3 m/s
  3. 後ろ・横・旋回は 0.3 のまま（前だけが変わる）
  4. 一時停止中は 0（stick-pause）
  5. 0 以下・1.0 を超える値・数でない値は、起動の時点で断る
"""

import argparse
import ast
import contextlib
import io
import sys
import textwrap
import types
from pathlib import Path

DEFAULT_FILE = Path.home() / "xr_teleoperate" / "teleop" / "teleop_hand_and_arm.py"


def extract(text: str, start: str, end: str) -> str:
    """start を含む行から end を含む行までを、字下げを外して返す。"""
    lines = text.splitlines(keepends=True)
    i = next(k for k, ln in enumerate(lines) if start in ln)
    j = next(k for k in range(i, len(lines)) if end in lines[k])
    return textwrap.dedent("".join(lines[i:j + 1]))


def load_parser(text: str) -> argparse.ArgumentParser:
    """_loco_fwd_max の定義と --loco-fwd-max の add_argument の行だけで引数の読み手を作る。"""
    ns = {"argparse": argparse}
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.FunctionDef) and node.name == "_loco_fwd_max":
            exec(ast.get_source_segment(text, node), ns)
    ns["parser"] = argparse.ArgumentParser()
    line = next(ln for ln in text.splitlines() if "'--loco-fwd-max'" in ln)
    exec(line.strip(), ns)
    return ns["parser"]


class FakeLoco:
    def __init__(self) -> None:
        self.sent = []

    def Move(self, vx: float, vy: float, vyaw: float) -> None:  # noqa: N802  上流と同じ名前
        self.sent.append((vx, vy, vyaw))


def step(src: str, fwd_max: float, left: tuple, right_x: float = 0.0, paused: bool = False,
         no_loco: bool = False, motion: bool = True) -> tuple:
    """スティックの値（televuer の形: 前は y が負）を 1 周期ぶん流し、機体へ送った指令と記録の値を返す。"""
    loco = FakeLoco()
    ns = {"tele_data": types.SimpleNamespace(left_ctrl_thumbstickValue=list(left), right_ctrl_thumbstickValue=[right_x, 0.0]),
          # no_loco は修正 no-loco（11-03）、motion は修正 seated-controls（11-04。--motion なしは座位）
          "args": types.SimpleNamespace(loco_fwd_max=fwd_max, no_loco=no_loco, motion=motion),
          "stick_paused": paused, "loco_wrapper": loco}
    exec(src, ns)
    # 足なし（no-loco）では Move を一度も呼ばないので、送った値は None で返す
    return (loco.sent[-1] if loco.sent else None), tuple(ns["loco_cmd_sent"])


def main() -> int:
    p = argparse.ArgumentParser(description="loco-fwd-max の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    a = p.parse_args()
    text = a.file.read_text(encoding="utf-8")
    if "PATCH(loco-fwd-max)" not in text:
        print(f"[自己テスト] ✗ loco-fwd-max が当たっていません: {a.file}")
        return 1
    src = extract(text, "loco_vx = -tele_data.left_ctrl_thumbstickValue[1]", "loco_wrapper.Move(*loco_cmd_sent)")
    parser = load_parser(text)

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    def close(x: tuple, y: tuple) -> bool:
        return all(abs(u - v) < 1e-9 for u, v in zip(x, y))

    default = parser.parse_args([]).loco_fwd_max
    check(default == 0.3, f"既定の上限は上流と同じ 0.3（{default}）")
    check(close(step(src, default, (0.0, -1.0))[0], (0.3, 0.0, 0.0)), "既定で前へ倒しきると 0.3 m/s")
    check(close(step(src, default, (0.0, 1.0))[0], (-0.3, 0.0, 0.0)), "既定で後ろへ倒しきると −0.3 m/s")

    v = parser.parse_args(["--loco-fwd-max", "0.6"]).loco_fwd_max
    check(v == 0.6, "--loco-fwd-max 0.6 を読める")
    sent, rec = step(src, v, (0.0, -1.0))
    check(close(sent, (0.6, 0.0, 0.0)), f"前へ倒しきると 0.6 m/s（{sent}）")
    check(close(rec, sent), "記録（loco_cmd_sent）は機体へ送った値と同じ")
    check(close(step(src, v, (0.0, -0.5))[0], (0.3, 0.0, 0.0)), "前へ半分なら 0.3 m/s")
    check(close(step(src, v, (0.0, 1.0))[0], (-0.3, 0.0, 0.0)), "後ろは 0.3 のまま")
    check(close(step(src, v, (-1.0, 0.0))[0], (0.0, 0.3, 0.0)), "横（左）は 0.3 のまま")
    check(close(step(src, v, (0.0, 0.0), right_x=1.0)[0], (0.0, 0.0, -0.3)), "旋回は 0.3 のまま")
    check(close(step(src, v, (0.0, -1.0), paused=True)[0], (0.0, 0.0, 0.0)), "一時停止中は前へ倒しても 0")

    for bad in ("0", "-0.2", "1.5", "nan", "fast"):
        with contextlib.redirect_stderr(io.StringIO()):
            try:
                parser.parse_args(["--loco-fwd-max", bad])
                refused = False
            except SystemExit:
                refused = True
        check(refused, f"--loco-fwd-max {bad} は起動の時点で断る")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
