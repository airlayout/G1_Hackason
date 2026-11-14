#!/usr/bin/env python3
"""採点環境と公式モデルを _local/ に用意する。ブランチ・作業ファイルは変更しない。"""

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

from yada_paths import CACHE, REPO_ROOT


def prepare(ref, fetch=True, models=True):
    if fetch:
        subprocess.run(["git", "fetch", "origin"], cwd=REPO_ROOT, check=True)
    commit = subprocess.check_output(["git", "rev-parse", "--verify", f"{ref}^{{commit}}"],
                                     cwd=REPO_ROOT, text=True).strip()
    if len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        raise ValueError("採点環境のコミットIDが不正です")
    CACHE.mkdir(parents=True, exist_ok=True)
    snapshot = CACHE / commit
    if not snapshot.exists():
        with tempfile.TemporaryDirectory(dir=CACHE) as staging:
            archive = Path(staging) / "source.tar"
            subprocess.run(["git", "archive", "--format=tar", f"--output={archive}", commit,
                            "Button_Press/Yada", "Button_Press/J1-gen", "Perception/common"],
                           cwd=REPO_ROOT, check=True)
            subprocess.run(["tar", "-xf", str(archive), "-C", staging], check=True)
            archive.unlink()
            Path(staging).rename(snapshot)
    # 初期版のスナップショットにも、RGB-D受信が利用するPerceptionの共通部品をそろえる。
    if not (snapshot / "Perception/common/__init__.py").is_file():
        archive = subprocess.check_output(["git", "archive", commit, "Perception/common"], cwd=REPO_ROOT)
        subprocess.run(["tar", "-xf", "-", "-C", str(snapshot)], input=archive, check=True)
    if models:
        model = snapshot / "_local/button_press/models/g1_description/g1_29dof_rev_1_0.urdf"
        if not model.is_file():
            subprocess.run(["bash", str(snapshot / "Button_Press/J1-gen/sim/fetch_models.sh")],
                           cwd=snapshot, check=True)
    (CACHE / "current.json").write_text(json.dumps({"commit": commit, "ref": ref}, indent=2) + "\n")
    print(f"採点環境: {snapshot / 'Button_Press/Yada'}\n固定コミット: {commit}")
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", default="origin/Dev/ButtonPress_Yada")
    parser.add_argument("--no-fetch", action="store_true", help="取得済みのリモート参照を使う")
    parser.add_argument("--skip-models", action="store_true")
    args = parser.parse_args()
    prepare(args.ref, not args.no_fetch, not args.skip_models)


if __name__ == "__main__":
    main()
