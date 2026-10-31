#!/usr/bin/env python3
"""YOLO の .pt を ONNX に書き出す(ラボPCで。ultralytics・onnx が入った G1_HuggingFace/venv)。G1のPC2では onnxruntime で動かす。

    G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py
    G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py --weights yolo26n.pt --out-dir _local/perception/models

従来型の出力 (1, 4 + クラス数, 8400)(end2end=False。NMS は common/detector/onnx_yolo.py が行う)、opset 17(PC2 の TensorRT 8.5
が読める上限)で書き出す。書き出したあと、common/detector/onnx_yolo.py の結果が .pt(ultralytics、640×640 の正方形に入れる)と
一致するかを、ultralytics に付いているサンプル画像(bus.jpg・zidane.jpg、人が写っている)で確かめる。ONNX(約 10 MB)は git に入れない(_local/ に置く)。

2026-10-31 までは end-to-end の出力 (1, 300, 6)(nms=False)で書き出していたが、.pt と結果がずれていた(bus.jpg の人の確からしさが
.pt 0.85 → ONNX 0.92。ボタンの YOLO では 4 個が 2 個になった)。.pt の推論は従来型の出口を使っているため。当時の確認は同じ ONNX を
ultralytics で動かした結果と比べていたので、このずれに気づかなかった。
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
    model = YOLO(str(pt) if pt.exists() else args.weights)
    onnx = Path(model.export(format="onnx", imgsz=args.imgsz, opset=17, end2end=False))
    print(f"[export] 書き出した: {onnx}（{onnx.stat().st_size / 1e6:.1f} MB）")

    mine = OnnxYoloDetector(str(onnx), classes=["person"], confidence_threshold=0.25)
    # TensorRT のエンジン(PC2で作る)にはクラス名が入らないので、別のファイルにしておく(common/detector/trt_yolo.py が読む)
    names = onnx.with_suffix(".names.json")
    names.write_text(json.dumps(mine.names, ensure_ascii=False))
    print(f"[export] クラス名: {names}")
    assets = Path(ultralytics.__file__).parent / "assets"
    ok = True
    for name in ("bus.jpg", "zidane.jpg"):
        img = cv2.imread(str(assets / name))
        a = sorted((round(d.confidence, 3) for d in mine.detect(img)), reverse=True)
        r = model.predict(img, imgsz=args.imgsz, rect=False, classes=[0], conf=0.25, verbose=False, device="cpu")[0]
        b = sorted((round(float(c), 3) for c in r.boxes.conf), reverse=True)
        same = len(a) == len(b) and all(abs(x - y) < 0.01 for x, y in zip(a, b))
        ok &= same
        print(f"[export] {name}: onnx_yolo.py {a} / ultralytics（.pt、正方形） {b} → {'一致' if same else '❌ 違う'}")
    print(f"[export] {'OK' if ok else '❌ 結果が合わない（onnx_yolo.py の前処理・後処理を見直す）'}。PC2 へは {onnx} を写す")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
