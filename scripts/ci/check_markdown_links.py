#!/usr/bin/env python3
"""Markdownの相対リンクが実在するかを検査する。

フォルダのリネームや移動でリンクが切れても、読むまで気づけない。
実際に SLAM/ -> Navigation/ のリネームを行ったため、機械的に検査する。

外部URL（http/https/mailto）は対象外。到達性はネットワーク状態に左右され、
CIを不安定にするため検査しない。

`--only <md> ...` で、そのファイルだけを検査する（PR で変えたものだけを見るため。git 管理下に無いもの・
除外フォルダのものは飛ばす。`--only` だけで 1 つも無ければ何も検査しない）。引数が無ければ git 管理下の全部。
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

# 棚上げ中のフォルダと外部クローンは対象外
EXCLUDED = ("IsaacSim_Env/", "SimEnv3D/", "G1_HuggingFace/")

LINK = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")


def tracked_markdown(only: list[str] | None = None) -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.md"], capture_output=True, text=True, check=True
    ).stdout.split()
    tracked = [p for p in out if not p.startswith(EXCLUDED)]
    if only is not None:
        tracked = [p for p in tracked if p in set(only)]
    return [Path(p) for p in tracked]


def tracked_paths() -> set[str]:
    """git管理下のファイルと、その親ディレクトリの集合。

    ファイルシステムの存在ではなくgit管理下かで判定する。手元にしか無いファイル
    （.gitignore済みのSimEnv3D/など）へのリンクは、cloneした人には切れているため。
    ローカル実行とCIの結果を一致させる意味もある。
    """
    out = subprocess.run(
        ["git", "ls-files"], capture_output=True, text=True, check=True
    ).stdout.split()
    paths: set[str] = set()
    for item in out:
        path = Path(item)
        paths.add(path.as_posix())
        for parent in path.parents:
            if parent != Path("."):
                paths.add(parent.as_posix())
    return paths


def main() -> int:
    total = 0
    broken: list[str] = []
    known = tracked_paths()
    only = None
    if len(sys.argv) > 1:
        if sys.argv[1] != "--only":
            print(f"使い方: {sys.argv[0]} [--only <md> ...]", file=sys.stderr)
            return 2
        only = sys.argv[2:]
        if not only:
            print("[links] 検査する Markdown が無い（PR で Markdown を変えていない）")
            return 0

    for md in tracked_markdown(only):
        text = md.read_text(encoding="utf-8", errors="replace")
        for match in LINK.finditer(text):
            target = match.group(2).split("#")[0].strip()
            if not target or target.startswith(("http://", "https://", "mailto:")):
                continue
            total += 1
            resolved = (md.parent / target).as_posix().rstrip("/")
            # ../ を含むパスを正規化する
            resolved = Path(resolved).resolve().relative_to(Path.cwd()).as_posix()
            if resolved not in known:
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
