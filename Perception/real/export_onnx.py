#!/usr/bin/env python3
"""YOLO の .pt を ONNX に書き出す(ラボPCで。ultralytics・onnx が入った G1_HuggingFace/venv)。G1のPC2では onnxruntime で動かす。

    G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py
    G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py --weights yolo26n.pt --out-dir _local/perception/models

end-to-end(YOLO26 のNMSを使わない出力 (1, 300, 6))で書き出す(common/detector/onnx_yolo.py が読む)。書き出したあと、
同じONNXを ultralytics で動かした結果と、common/detector/onnx_yolo.py の結果が一致するかを、ultralytics に付いている
サンプル画像(bus.jpg・zidane.jpg、人が写っている)で確かめる。ONNX(約 9 MB)は git に入れない(_local/ に置く)。
"""
import argparse
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("YOLO_AUTOINSTALL", "False")  # 足りないパッケージを勝手に pip install させない
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
REPO_ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", default=str(REPO_ROOT / "yolo26n.pt"), help="書き出す .pt(無ければ ultralytics がダウンロードする名前)")
    p.add_argument("--out-dir", type=Path, default=REPO_ROOT / "_local" / "perception" / "models")
    p.add_argument("--imgsz", type=int, default=640)
    args = p.parse_args()

    import cv2
    import ultralytics
    from ultralytics import YOLO

    from common.detector.onnx_yolo import OnnxYoloDetector

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pt = args.out_dir / Path(args.weights).name
    if Path(args.weights).exists() and Path(args.weights).resolve() != pt.resolve():
        shutil.copy(args.weights, pt)  # 書き出しは .pt の隣にできるので、_local/ に写してから書き出す
    onnx = Path(YOLO(str(pt) if pt.exists() else args.weights).export(
        format="onnx", imgsz=args.imgsz, simplify=False, opset=17, nms=False))
    print(f"[export] 書き出した: {onnx}（{onnx.stat().st_size / 1e6:.1f} MB）")

    mine = OnnxYoloDetector(str(onnx), classes=["person"], confidence_threshold=0.25)
    # TensorRT のエンジン(PC2で作る)にはクラス名が入らないので、別のファイルにしておく(common/detector/trt_yolo.py が読む)
    names = onnx.with_suffix(".names.json")
    names.write_text(json.dumps(mine.names, ensure_ascii=False))
    print(f"[export] クラス名: {names}")
    ref = YOLO(str(onnx), task="detect")
    assets = Path(ultralytics.__file__).parent / "assets"
    ok = True
    for name in ("bus.jpg", "zidane.jpg"):
        img = cv2.imread(str(assets / name))
        a = sorted((round(d.confidence, 3) for d in mine.detect(img)), reverse=True)
        r = ref.predict(img, classes=[0], conf=0.25, verbose=False, device="cpu")[0]
        b = sorted((round(float(c), 3) for c in r.boxes.conf), reverse=True)
        same = len(a) == len(b) and all(abs(x - y) < 0.01 for x, y in zip(a, b))
        ok &= same
        print(f"[export] {name}: onnx_yolo.py {a} / ultralytics（同じONNX） {b} → {'一致' if same else '❌ 違う'}")
    print(f"[export] {'OK' if ok else '❌ 結果が合わない（onnx_yolo.py の前処理・後処理を見直す）'}。PC2 へは {onnx} を写す")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
