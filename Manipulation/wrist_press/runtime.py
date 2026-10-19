"""Read pinned branch sources without switching, merging, or touching the index."""
from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

from .settings import ROOT

YADA_COMMIT = "634697e1544aa74b8f5d5869d58dbcc5dc9881d6"
J1_COMMIT = "7356db679c63bc713e584c0423475bb3710e9fd3"
CACHE = ROOT / "_local/wrist_press"


def prepare(yada_ref=YADA_COMMIT, j1_ref=J1_COMMIT, model_dir=None):
    def rev(ref):
        return subprocess.check_output(["git", "rev-parse", f"{ref}^{{commit}}"], cwd=ROOT, text=True).strip()
    yada, j1 = rev(yada_ref), rev(j1_ref)
    target = CACHE / "sources" / f"{yada[:12]}-{j1[:12]}"
    if not (target / "sources.json").exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=target.parent) as tmp:
            tmp = Path(tmp)
            for ref, paths in ((yada, ["Button_Press/Yada"]),
                               (j1, ["Button_Press/J1-gen", "Perception/common"])):
                archive = tmp / "source.tar"
                with archive.open("wb") as f:
                    subprocess.run(["git", "archive", ref, *paths], cwd=ROOT, stdout=f, check=True)
                with tarfile.open(archive) as tar:
                    # Only regular files from these exact repository subtrees; no links.
                    for m in tar.getmembers():
                        p = (tmp / m.name).resolve()
                        if not p.is_relative_to(tmp.resolve()) or m.issym() or m.islnk():
                            raise ValueError(f"unsafe archive member: {m.name}")
                        if m.isfile() or m.isdir():
                            tar.extract(m, tmp)
                archive.unlink()
            (tmp / "sources.json").write_text(json.dumps({"yada": yada, "j1": j1}, indent=2))
            shutil.copytree(tmp, target)
    models = target / "_local/button_press/models/g1_description"
    if not models.exists():
        candidates = [Path(model_dir)] if model_dir else list(
            (ROOT / "_local/button_press_yada/evaluation_env").glob("*/_local/button_press/models/g1_description"))
        candidates += list((ROOT / "_local/button_press_camera").glob("*/_local/button_press/models/g1_description"))
        for source in candidates:
            if (source / "g1_29dof_rev_1_0.urdf").is_file():
                shutil.copytree(source, models)
                break
    if not models.exists():
        raise FileNotFoundError(f"G1 models missing. Run bash {target}/Button_Press/J1-gen/sim/fetch_models.sh")
    import yaml
    expected=yaml.safe_load((target/"Button_Press/J1-gen/configs/robot.yaml").read_text())["model"]["commit"]
    source=models/"SOURCE.txt"
    if not source.is_file() or expected not in source.read_text():
        raise ValueError(f"G1 model SOURCE.txt must identify the expected revision {expected}")
    manifest = {"root": str(target), "yada": yada, "j1": j1}
    CACHE.mkdir(parents=True, exist_ok=True)
    (CACHE / "current.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return target


def source_root():
    p = CACHE / "current.json"
    if not p.exists():
        raise FileNotFoundError("Run python -m Manipulation.wrist_press.runtime first")
    return Path(json.loads(p.read_text())["root"])


def j1_module(name):
    package = "wrist_press_j1"
    if package not in sys.modules:
        root = source_root() / "Button_Press/J1-gen/common"
        spec = importlib.util.spec_from_file_location(package, root / "__init__.py", submodule_search_locations=[str(root)])
        module = importlib.util.module_from_spec(spec)
        sys.modules[package] = module
        spec.loader.exec_module(module)
    return importlib.import_module(f"{package}.{name}")


def enable_yada():
    root = source_root() / "Button_Press/Yada"
    existing = sys.modules.get("contest")
    if existing is not None and not Path(existing.__file__).resolve().is_relative_to(root):
        raise RuntimeError("another Yada version is already imported; start a new process")
    sys.path.insert(0, str(root))
    return root


def robot_config():
    import yaml
    root = source_root()
    cfg = yaml.safe_load((root / "Button_Press/J1-gen/configs/robot.yaml").read_text())
    for k in ("urdf_path", "mjcf_path"):
        cfg["model"][k] = str(root / cfg["model"][k])
    return cfg


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yada-ref", default=YADA_COMMIT)
    parser.add_argument("--j1-ref", default=J1_COMMIT)
    parser.add_argument("--model-dir")
    args = parser.parse_args()
    print(prepare(args.yada_ref, args.j1_ref, args.model_dir))
