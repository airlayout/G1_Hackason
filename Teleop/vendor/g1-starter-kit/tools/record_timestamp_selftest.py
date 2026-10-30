#!/usr/bin/env python3
"""[G1_Hackason 改変] record-timestamp の自己テスト（機体・Quest・cv2 不要）。

修正済みの utils/episode_writer.py から EpisodeWriter.add_item だけを抜き出し、偽の self で
2 回呼んで、1 コマごとに time_ns が入るかを確かめる。setup/apply_patches.py で当てた後に流す。

    python3 tools/record_timestamp_selftest.py
    python3 tools/record_timestamp_selftest.py --file <episode_writer.py>
"""

import argparse
import ast
import queue
import sys
import textwrap
import time
import types
from pathlib import Path

DEFAULT_FILE = Path.home() / "xr_teleoperate" / "teleop" / "utils" / "episode_writer.py"


def load_add_item(path: Path):
    """episode_writer.py から add_item の定義だけを取り出して関数にする。"""
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "add_item":
            src = textwrap.dedent(ast.get_source_segment(text, node))
            ns = {"time": time}
            exec(src, ns)
            return ns["add_item"], text
    raise SystemExit(f"[自己テスト] ✗ add_item が見つかりません: {path}")


def main() -> int:
    p = argparse.ArgumentParser(description="record-timestamp の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    args = p.parse_args()
    add_item, text = load_add_item(args.file)
    if "PATCH(record-timestamp)" not in text:
        print(f"[自己テスト] ✗ record-timestamp が当たっていません: {args.file}")
        return 1

    fake = types.SimpleNamespace(item_id=-1, item_data_queue=queue.Queue())
    before = time.time_ns()
    add_item(fake, colors={})
    time.sleep(0.01)
    add_item(fake, colors={})
    after = time.time_ns()
    a, b = fake.item_data_queue.get(), fake.item_data_queue.get()

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    check(isinstance(a.get("time_ns"), int), "1 コマに time_ns（整数）が入る")
    check(before <= a["time_ns"] <= b["time_ns"] <= after, "時刻は呼んだ時点の time.time_ns() で、増えていく")
    check(b["time_ns"] - a["time_ns"] >= 10_000_000, "間隔が実際の経過（10 ms 以上）を映す")
    check((a["idx"], b["idx"]) == (0, 1), "番号（idx）は今までどおり")
    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
