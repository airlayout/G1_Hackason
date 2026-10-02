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
# 外部から取り込んだコード（vendor/ の下）の文書も対象外。元のリポジトリにある画像・動画・別のパッケージへの
# リンクなので、このリポジトリでは切れているのが正常（例: Navigation/nav2_stable/vendor/fast_lio_localization_humanoid）
EXCLUDED_PARTS = ("/vendor/",)

LINK = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")


def tracked_markdown() -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.md"], capture_output=True, text=True, check=True
    ).stdout.split()
    return [Path(p) for p in out if not p.startswith(EXCLUDED) and not any(x in f"/{p}" for x in EXCLUDED_PARTS)]


def looks_like_path(target: str) -> bool:
    """リンクの行き先が、ファイルやフォルダのパスらしいか。

    「[m](G1の機体半径)」（単位の [m] のあとに、かっこ書きの説明）のように、リンクではない文も
    [..](..) の形になる。ファイルやフォルダへのリンクは、ふつう「/」か「.」（拡張子）を含むので、
    どちらも含まないものはリンクとみなさない。
    """
    return "/" in target or "." in target


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

    for md in tracked_markdown():
        text = md.read_text(encoding="utf-8", errors="replace")
        for match in LINK.finditer(text):
            target = match.group(2).split("#")[0].strip()
            if not target or target.startswith(("http://", "https://", "mailto:")) or not looks_like_path(target):
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
