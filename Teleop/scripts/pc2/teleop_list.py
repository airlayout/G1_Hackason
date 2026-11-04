#!/usr/bin/env python3
"""2026-11-03: 遠隔操作で録った記録（~/teleop_data/<課題名>/episode_*）の一覧。PC2 の上で動く（numpy だけ）。

scripts/pc2_teleop.sh の list・replay から ssh で呼ばれる（操作するのは Mac でも OMEN でもよい）。

    python3 teleop_list.py                      # 全部の課題（新しい順に 20 本）
    python3 teleop_list.py <課題名>              # その課題だけ
    python3 teleop_list.py --has-walk <episode のフォルダ>   # 歩きがあれば 0、無ければ 1、読めなければ 2（再生の入口が使う）

足（あり／なし／座位）は、録った時の課題の説明（text.desc の legs=…。teleop_start.sh が入れる）と、歩きの指令が 0 でない
コマがあるかで出す。説明が無い古い記録は、歩きの指令から推した値に「?」を付ける。
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
# 配った先（~/teleop_tools/）では同じフォルダ、リポジトリではキットの tools/ に loco_replay.py がある
for p in (HERE, HERE.parents[1] / "vendor" / "g1-starter-kit" / "tools"):
    if (p / "loco_replay.py").exists():
        sys.path.insert(0, str(p))
        break
import loco_replay  # noqa: E402

DATA = Path.home() / "teleop_data"
SHOW_MAX = 20


def describe(ep: Path) -> str:
    """1 本を 1 行にする。読めない記録もその旨を 1 行で返す（一覧は止めない）。"""
    try:
        doc = json.loads((ep / "data.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return f"  {ep.name}   （data.json が読めない: {type(e).__name__}）"
    frames = doc.get("data", [])
    if not frames:
        return f"  {ep.name}   ← 空（記録されていない）"
    stamps = [f.get("time_ns") for f in frames]
    fps = float(doc.get("info", {}).get("image", {}).get("fps", 30.0)) or 30.0
    # 再生（replay_arm.py）と同じく、時刻があれば time_ns、無ければ名目の fps で数える
    times = (np.asarray(stamps, dtype=np.int64) - stamps[0]) / 1e9 if all(isinstance(s, int) for s in stamps) \
        else np.arange(len(frames)) / fps
    secs = float(times[-1])
    cmd = loco_replay.load_loco(ep)
    walk_frames = int(loco_replay.walking_mask(cmd).sum()) if cmd is not None else 0
    desc = str((doc.get("text") or {}).get("desc", ""))
    if desc.startswith("legs="):
        legs = {"legs=walk": "あり", "legs=none": "なし", "legs=seated": "座位"}.get(desc, "?")  # 座位は 11-04
    else:
        legs = ("あり?" if walk_frames else "なし?")
    walk = f"歩き {loco_replay.walking_seconds(cmd, times):5.1f} 秒" if cmd is not None else "歩きの欄なし"
    return f"  {ep.name}  {len(frames):6d} コマ {secs:6.1f} 秒  足{legs:<3}  {walk}"


def has_walk(ep: Path) -> bool:
    cmd = loco_replay.load_loco(ep)
    return cmd is not None and bool(loco_replay.walking_mask(cmd).any())


def main() -> int:
    p = argparse.ArgumentParser(description="遠隔操作の記録の一覧")
    p.add_argument("task", nargs="?", help="課題名（省くと全部）")
    p.add_argument("--has-walk", type=Path, metavar="EPISODE", help="歩きがあるかだけを終了コードで返す")
    a = p.parse_args()
    if a.has_walk is not None:
        try:
            return 0 if has_walk(a.has_walk) else 1
        except (OSError, ValueError) as e:   # 「歩きなし」と取り違えて腕だけで流さないよう、別の終了コードにする
            print(f"[list] ✗ 記録が読めない: {a.has_walk}（{type(e).__name__}: {e}）")
            return 2

    base = DATA / a.task if a.task else DATA
    pattern = "episode_*" if a.task else "*/episode_*"
    # フォルダだけ（check-ep が同じ所に episode_checks.csv を置く）
    eps = sorted((d for d in base.glob(pattern) if d.is_dir()), key=lambda d: d.stat().st_mtime, reverse=True)
    if not eps:
        print(f"[list] 記録が無い: {base}/")
        return 1
    print(f"[list] {base}/（新しい順・{min(len(eps), SHOW_MAX)} / {len(eps)} 本）")
    last_task = None
    for ep in eps[:SHOW_MAX]:
        if ep.parent.name != last_task:
            print(f" {ep.parent.name}/")
            last_task = ep.parent.name
        print(describe(ep))
    return 0


if __name__ == "__main__":
    sys.exit(main())
