"""commitとSHA-256で固定したロボット・検証用モデルだけを取得する。"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import urllib.request


def main() -> None:
    """--with-demoの指定時だけONNXモデルを取得する。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-demo", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "assets/manifest.json").read_text())
    for item in manifest["files"]:
        if item["group"] == "demo" and not args.with_demo:
            continue
        path = root / "assets/cache" / item["path"]
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]:
            continue
        print(f"[FETCH] {item['path']}", flush=True)
        with urllib.request.urlopen(item["url"], timeout=60) as response:
            content = response.read()
        if hashlib.sha256(content).hexdigest() != item["sha256"]:
            raise ValueError(f"SHA-256 mismatch: {item['path']}")
        temporary = path.with_suffix(path.suffix + ".download")
        temporary.write_bytes(content)
        temporary.replace(path)
    print("[READY] 固定版アセットを確認しました")


if __name__ == "__main__":
    main()
