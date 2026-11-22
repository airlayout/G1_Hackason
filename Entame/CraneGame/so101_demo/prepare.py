"""Cache the pinned Google Hand Landmarker and its browser runtime."""
from hashlib import sha256
from pathlib import Path
from urllib.request import urlopen

PACKAGE = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.22-rc.20250304/"
ASSETS = {"vision_bundle.mjs": PACKAGE + "vision_bundle.mjs"}
for variant in ("vision_wasm_internal", "vision_wasm_nosimd_internal"):
    for ext in ("js", "wasm"):
        name = f"{variant}.{ext}"
        ASSETS[name] = PACKAGE + "wasm/" + name
ASSETS["hand_landmarker.task"] = ("https://storage.googleapis.com/mediapipe-models/"
                                  "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")

SHA256 = {
    "hand_landmarker.task": "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1",
    "vision_bundle.mjs": "3252b10e0fd26c4095e4b6ee545c1c898120b3a3813388ecd4876abe6f1399a9",
    "vision_wasm_internal.js": "4a97e2520ba506c680ecd6ba6acfb146888afa0e2746d57f205352bc6ebb82eb",
    "vision_wasm_internal.wasm": "f00ec4731faa23b3e714d00e88d4d10e2df5c0a427d3a2b4ae6e3526fdd14ef7",
    "vision_wasm_nosimd_internal.js": "927def7b465c51b86e4b3060f93646aca4e27121f4b8fc0483786e407ea9cf1f",
    "vision_wasm_nosimd_internal.wasm": "3821ea9b1f7fb8c549ef2a064ef5c85750bf375c545a49fd6eea0df44a95f1f4",
}


def main():
    folder = Path(__file__).resolve().parent / "assets"
    folder.mkdir(exist_ok=True)
    for name, url in ASSETS.items():
        dest = folder / name
        if dest.is_file() and sha256(dest.read_bytes()).hexdigest() == SHA256[name]:
            continue
        print(f"取得中 {name}", flush=True)
        with urlopen(url, timeout=60) as response:
            body = response.read()
        if sha256(body).hexdigest() != SHA256[name]:
            raise RuntimeError(f"取得内容が不正: {url}")
        tmp = dest.with_suffix(".part")
        tmp.write_bytes(body)
        tmp.replace(dest)
    print("カメラ用ファイルの準備完了")


if __name__ == "__main__":
    main()
