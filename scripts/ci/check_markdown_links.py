#!/usr/bin/env python3
"""Markdownの相対リンクが実在するかを検査する。

フォルダのリネームや移動でリンクが切れても、読むまで気づけない。
実際に SLAM/ -> Navigation/ のリネームを行ったため、機械的に検査する。

外部URL（http/https/mailto）は対象外。到達性はネットワーク状態に左右され、
CIを不安定にするため検査しない。
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

# 棚上げ中のフォルダと外部クローンは対象外
EXCLUDED = ("IsaacSim_Env/", "SimEnv3D/", "G1_HuggingFace/")

LINK = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")


def tracked_markdown() -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.md"], capture_output=True, text=True, check=True
    ).stdout.split()
    return [Path(p) for p in out if not p.startswith(EXCLUDED)]


def main() -> int:
    total = 0
    broken: list[str] = []

    for md in tracked_markdown():
        text = md.read_text(encoding="utf-8", errors="replace")
        for match in LINK.finditer(text):
            target = match.group(2).split("#")[0].strip()
            if not target or target.startswith(("http://", "https://", "mailto:")):
                continue
            total += 1
            if not (md.parent / target).exists():
                broken.append(f"{md} -> {target}")

    print(f"[links] 相対リンク {total} 本を検査")
    if broken:
        print(f"[links] NG: {len(broken)} 本が切れている")
        for item in broken:
            print(f"  {item}")
        return 1
    print("[links] OK: 切れているリンクはない")
    return 0


if __name__ == "__main__":
    sys.exit(main())
