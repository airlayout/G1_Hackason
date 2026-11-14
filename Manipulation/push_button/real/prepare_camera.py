#!/usr/bin/env python3
"""Dev/ButtonPressのカメラ部品と公式モデルを、版を記録して_local/に用意する。"""

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from j1gen_camera import CACHE, REPO_ROOT


def prepare(ref, fetch=True, model_cache=None):
    if fetch:
        subprocess.run(["git", "fetch", "origin"], cwd=REPO_ROOT, check=True)
    commit = subprocess.check_output(["git", "rev-parse", "--verify", f"{ref}^{{commit}}"],
                                     cwd=REPO_ROOT, text=True).strip()
    if len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        raise ValueError("J1-genのコミットIDが不正です")
    CACHE.mkdir(parents=True, exist_ok=True)
    snapshot = CACHE / commit
    if not snapshot.exists():
        with tempfile.TemporaryDirectory(dir=CACHE) as staging:
            archive = Path(staging) / "camera-source.tar"
            subprocess.run(["git", "archive", "--format=tar", f"--output={archive}", commit,
                            "Button_Press/J1-gen", "Perception/common"], cwd=REPO_ROOT, check=True)
            subprocess.run(["tar", "-xf", str(archive), "-C", staging], check=True)
            archive.unlink()
            Path(staging).rename(snapshot)
    feature = snapshot / "Button_Press/J1-gen"
    destination = snapshot / "_local/button_press/models/g1_description"
    if not (destination / "g1_29dof_rev_1_0.urdf").is_file():
        if model_cache:
            import yaml

            source = Path(model_cache).expanduser().resolve()
            cfg = yaml.safe_load((feature / "configs/robot.yaml").read_text())
            provenance = (source / "SOURCE.txt").read_text()
            if (f"commit {cfg['model']['commit']}" not in provenance
                    or not (source / "g1_29dof_rev_1_0.urdf").is_file()):
                raise ValueError("モデルの取得元または版がJ1-genの設定と一致しません")
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(source, destination)
        else:
            subprocess.run(["bash", str(feature / "sim/fetch_models.sh")], cwd=snapshot, check=True)
    (CACHE / "current.json").write_text(json.dumps({"commit": commit, "ref": ref}, indent=2) + "\n")
    print(f"J1-genカメラ部品: {feature}\n固定コミット: {commit}")
    return feature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", default="origin/Dev/ButtonPress")
    parser.add_argument("--no-fetch", action="store_true")
    parser.add_argument("--model-cache", help="取得済みg1_description。SOURCE.txtの版を検査して再利用する")
    args = parser.parse_args()
    prepare(args.ref, not args.no_fetch, args.model_cache)


if __name__ == "__main__":
    main()
