"""公式OfficeのUSD・テクスチャを再帰取得し、全ファイルのSHA-256を保存する。"""

from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
from pathlib import Path
import re
import time
from urllib.parse import unquote, urljoin
import urllib.error
import urllib.request

from g1_e2e.data.scene import OfficeConfig


def dependencies(layer):
    from pxr import Sdf

    result = set(layer.GetExternalReferences())

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            value = spec.default
            if isinstance(value, Sdf.AssetPath) and value.path:
                result.add(value.path)
            elif spec.typeName == Sdf.ValueTypeNames.AssetArray and value is not None:
                result.update(item.path for item in value if item.path)

    layer.Traverse("/", visit)
    return result


def mdl_dependencies(text):
    """相対MDL moduleと既定textureの参照も保存し、家具だけ灰色になる欠損を防ぐ。"""
    result = set()
    for module in re.findall(r"(?:using|import)\s+((?:\.{1,2}::)+[\w:]+)", text):
        result.add(module.rstrip(":").replace("..::", "../").replace(".::", "./").replace("::", "/") + ".mdl")
    result.update(re.findall(r'"([^"\n]+\.(?:png|jpg|jpeg|tga|tif|exr|hdr|dds))"', text, re.IGNORECASE))
    return result


def fetch(config, root, workers=4):
    from pxr import Sdf

    directory = config.directory(root)
    lock_path = directory / "office-assets.lock.json"
    previous = json.loads(lock_path.read_text()) if lock_path.exists() else {"files": {}}
    files, builtins, pending = {}, set(), {config.url}

    def download(url):
        if not url.startswith(config.asset_root_url):
            raise ValueError(f"指定したアセットroot外の参照です: {url}")
        relative = unquote(url[len(config.asset_root_url) :])
        if ".." in Path(relative).parts:
            raise ValueError(f"cache外の参照です: {relative}")
        path = directory / relative
        expected = (
            config.asset_sha256 if url == config.url else previous["files"].get(relative, {}).get("sha256")
        )
        if path.exists():
            content = path.read_bytes()
        else:
            for attempt in range(5):
                try:
                    with urllib.request.urlopen(url, timeout=45) as response:
                        content = response.read()
                    break
                except urllib.error.HTTPError as error:
                    if error.code not in {429, 500, 502, 503, 504} or attempt == 4:
                        raise
                    time.sleep(2**attempt)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".download")
            temporary.write_bytes(content)
            temporary.replace(path)
        digest = hashlib.sha256(content).hexdigest()
        if expected is not None and digest != expected:
            raise ValueError(f"アセットのSHA-256が固定値と異なります: {path}")
        return relative, path, {"url": url, "sha256": digest, "bytes": len(content)}

    with ThreadPoolExecutor(max_workers=workers) as pool:
        while pending:
            batch, pending = sorted(pending), set()
            for relative, path, record in pool.map(download, batch):
                files[relative] = record
                if path.suffix.lower() == ".mdl":
                    assets = mdl_dependencies(path.read_text(encoding="utf-8-sig"))
                elif path.suffix.lower() in {".usd", ".usda", ".usdc"}:
                    layer = Sdf.Layer.FindOrOpen(str(path))
                    if layer is None:
                        raise ValueError(f"USDを開けません: {path}")
                    assets = dependencies(layer)
                else:
                    continue
                for asset in assets:
                    # NVIDIA標準MDLはIsaac Simの検索パスから解決する。
                    if Path(asset).name == asset and asset.endswith(".mdl"):
                        builtins.add(asset)
                        continue
                    target = urljoin(record["url"], asset)
                    target_relative = unquote(target[len(config.asset_root_url) :])
                    if target_relative not in files:
                        pending.add(target)
            print(f"[FETCH] {len(files)} files; pending {len(pending)}", flush=True)
    result = {
        "source_documentation": config.source_documentation,
        "asset_root_url": config.asset_root_url,
        "entrypoint": config.asset_path,
        "builtin_mdl_modules": sorted(builtins),
        "files": dict(sorted(files.items())),
    }
    lock_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(
        f"[READY] {config.usd_path(root)}; {sum(item['bytes'] for item in files.values()) / 1e6:.1f} MB",
        flush=True,
    )


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=root / "configs/scenes/office.yaml")
    args = parser.parse_args()
    fetch(OfficeConfig.load(args.scene_config), root)


if __name__ == "__main__":
    main()
