"""Install existing button weights, fetch official bottle weights, or train buttons."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.request import urlopen

from .runtime import source_root
from .settings import ROOT, resolve_path


def validate(path, expected):
    from ultralytics import YOLO
    model = YOLO(str(path))
    if not set(expected) <= set(model.names.values()):
        raise ValueError(f"wrong YOLO classes: {model.names}")
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "classes": model.names}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--button-source", type=Path, help="existing up/down best.pt")
    ap.add_argument("--button-output", default="_local/button_press/button_yolo/weights/best.pt")
    ap.add_argument("--download-bottle", action="store_true")
    ap.add_argument("--train-buttons", action="store_true", help="render training images and run the existing trainer")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--start-seed",type=int,default=10000,help="keep training seeds separate from evaluation")
    args = ap.parse_args()
    metadata = {}
    output = resolve_path(args.button_output)
    if args.button_source:
        validate(args.button_source, ["up","down"])
        output.parent.mkdir(parents=True, exist_ok=True)
        if args.button_source.resolve() != output.resolve():
            shutil.copyfile(args.button_source, output)
        metadata["button"] = validate(output, ["up","down"])
    if args.download_bottle or args.train_buttons:
        bottle = ROOT / "yolo26n.pt"
        if not bottle.exists():
            # Fixed official release; no silent network activity at robot startup.
            url = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt"
            with urlopen(url, timeout=30) as response:
                data = response.read()
            temporary = bottle.with_suffix(".part")
            temporary.write_bytes(data)
            temporary.replace(bottle)
        metadata["bottle"] = validate(bottle, ["bottle"])
    if args.train_buttons:
        root = source_root()
        feature = root / "Button_Press/J1-gen"
        dataset=ROOT/f"_local/wrist_press/training/buttons-{args.start_seed}-{args.seeds}"
        # Existing trainer uses this path; keep its base weights explicitly local.
        shutil.copyfile(ROOT / "yolo26n.pt", feature / "yolo26n.pt")
        subprocess.run([sys.executable, str(feature / "sim/make_button_dataset.py"),
                        "--seeds", str(args.seeds), "--start",str(args.start_seed),"--workers", "1",
                        "--out",str(dataset)], cwd=ROOT, check=True)
        subprocess.run([sys.executable, str(feature / "sim/train_button_yolo.py"),
                        "--epochs", str(args.epochs), "--device", "cpu", "--out", str(output),
                        "--data",str(dataset/"data.yaml"),"--name",f"up_down_{args.start_seed}_{args.seeds}"], cwd=ROOT, check=True)
        metadata["button"] = validate(output, ["up","down"])
        metadata["button"]["training"] = {"sim_only": True, "seeds": args.seeds,"start_seed":args.start_seed,"epochs": args.epochs}
    directory = ROOT / "_local/wrist_press"
    directory.mkdir(parents=True, exist_ok=True)
    manifest = directory / "weights.json"
    if manifest.exists():
        metadata = {**json.loads(manifest.read_text()), **metadata}
    manifest.write_text(json.dumps(metadata, indent=2)+"\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
