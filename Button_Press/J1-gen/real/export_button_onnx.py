"""ボタンの YOLO（.pt）を ONNX に書き出し、同じ画像で .pt と結果が合うかを確かめる（ノート PC・ラボ PC で。ultralytics・onnx が要る）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/export_button_onnx.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/export_button_onnx.py --weights _local/button_press/yolo_ft/.../best.pt

書き出した .onnx は .pt の隣に置く（git の対象外）。G1 の PC2 では configs/elevator.yaml の yolo.weights に .onnx を書くと、
onnxruntime で動く（common/elevator_buttons.py の YoloButtonDetector。torch・ultralytics は要らない）。
画像は既定で、学習データの検証用（シミュレーション）と、ラボのエレベーターのスマホの写真（_local/button_press/phone_photos/
20261015_lab_elevator）。2 つを比べる:
1. 同じモデルか（合否）: .pt を ultralytics で 640×640 の正方形に入れた結果と、.onnx（onnx_yolo.py）の結果が誤差の範囲で一致するか
2. 今の使い方との差（記録だけ）: YoloButtonDetector を .pt と .onnx で作ったときの候補。ultralytics は .pt を横長のまま
   （640×480 など）入れるが、.onnx は正方形なので、確からしさが少し変わる（2026-10-31: 矢印はすべて同じ、確からしさの差は最大 0.18）
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("YOLO_AUTOINSTALL", "False")  # 足りないパッケージを勝手に pip install させない
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config, resolve_repo_path  # noqa: E402
from common.elevator_buttons import Candidate2D, YoloButtonDetector  # noqa: E402
from common.perception_bridge import perception  # noqa: E402

DEFAULT_IMAGES = [
    REPO_ROOT / "_local" / "button_press" / "button_yolo" / "dataset" / "images" / "val",
    REPO_ROOT / "_local" / "button_press" / "phone_photos" / "20261015_lab_elevator",
]
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}
# 同じモデルとみなす差（同じ正方形の入力なら、.pt は PyTorch、.onnx は onnxruntime で、どちらも FP32。差は計算の順の違いだけのはず）
CONF_TOL = 0.02
IOU_MIN = 0.95
# (枠, クラス名, 確からしさ)
Box = tuple[tuple[float, float, float, float], str | None, float]


def list_images(paths: list[Path]) -> list[Path]:
    out: list[Path] = []
    for p in paths:
        if p.is_dir():
            out += sorted(f for f in p.rglob("*") if f.suffix.lower() in IMAGE_SUFFIXES)
        elif p.suffix.lower() in IMAGE_SUFFIXES:
            out.append(p)
    return out


def iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def compare(ref: list[Box], got: list[Box], strict: bool) -> tuple[bool, str]:
    """確からしさの高い順に、いちばん重なる枠どうしで比べる。strict なら枠の重なりと確からしさの差も合否に入れる。"""
    if len(ref) != len(got):
        return False, f"数が違う（.pt {len(ref)} / .onnx {len(got)}）"
    worst_iou, worst_conf = 1.0, 0.0
    for box, name, conf in sorted(ref, key=lambda b: -b[2]):
        g = max(got, key=lambda b: iou(box, b[0]))
        if g[1] != name:
            return False, f"クラスが違う（.pt {name} / .onnx {g[1]}）"
        worst_iou = min(worst_iou, iou(box, g[0]))
        worst_conf = max(worst_conf, abs(conf - g[2]))
    ok = not strict or (worst_iou >= IOU_MIN and worst_conf <= CONF_TOL)
    return ok, f"{len(ref)} 個、枠の重なり 最小 {worst_iou:.3f}、確からしさの差 最大 {worst_conf:.3f}"


def from_candidates(cands: list[Candidate2D]) -> list[Box]:
    return [(tuple(float(v) for v in c.bbox), c.arrow, c.score) for c in cands]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", help="書き出す .pt（既定は configs/elevator.yaml の yolo.weights）")
    ap.add_argument("--images", type=Path, nargs="+", default=DEFAULT_IMAGES, help="比べる画像（ファイルかフォルダ）")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--no-export", action="store_true", help="書き出さず、.pt の隣にある .onnx と比べるだけ")
    args = ap.parse_args()

    cfg = dict(load_config("elevator.yaml")["yolo"])
    pt = resolve_repo_path(args.weights or cfg["weights"])
    if pt.suffix != ".pt":
        print(f"[export] .pt を指定すること: {pt}")
        return 2
    onnx = pt.with_suffix(".onnx")
    if not args.no_export:
        from ultralytics import YOLO

        # end2end=False（従来型の出力 (1, 4 + クラス数, 8400)。重なりの間引き（NMS）は onnx_yolo.py が行う）で書き出す。
        # YOLO26 の end-to-end の出力（nms=False を付けると選ばれる。Perception/real/export_onnx.py の書き方）は、このボタンの
        # モデルでは .pt と結果が大きく変わった（2026-10-31。確からしさが 1.0 に張り付き、4 個が 2 個になる。.pt の推論は
        # 従来型の出口を使っている）。opset 17 は PC2 の TensorRT 8.5 でも読める上限
        out = Path(YOLO(str(pt)).export(format="onnx", imgsz=args.imgsz, opset=17, end2end=False))
        print(f"[export] 書き出した: {out}（{out.stat().st_size / 1e6:.1f} MB）")
        onnx = out

    from ultralytics import YOLO

    conf = float(cfg.get("confidence", 0.4))
    square_pt = YOLO(str(pt))
    raw_onnx = perception("detector.onnx_yolo").OnnxYoloDetector(str(onnx), classes=None, confidence_threshold=conf)
    det_pt = YoloButtonDetector({**cfg, "weights": str(pt), "device": "cpu"})
    det_onnx = YoloButtonDetector({**cfg, "weights": str(onnx)})
    images = list_images(args.images)
    if not images:
        print(f"[export] 比べる画像が無い: {[str(p) for p in args.images]}")
        return 2
    n_same = n_use = 0
    worst_use = 0.0
    t_pt = t_onnx = 0.0
    for path in images:
        bgr = cv2.imread(str(path))
        if bgr is None:
            print(f"[export] 読めない: {path}")
            continue
        name = path.relative_to(REPO_ROOT) if path.is_relative_to(REPO_ROOT) else path
        # 1. 同じモデルか: どちらも 640×640 の正方形に入れる
        r = square_pt.predict(bgr, imgsz=args.imgsz, rect=False, conf=conf, verbose=False, device="cpu")[0]
        a = [(tuple(float(v) for v in xyxy), r.names[int(c)], float(s))
             for xyxy, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls, r.boxes.conf)]
        b = [(d.bbox, d.class_name, d.confidence) for d in raw_onnx.detect(bgr)]
        same, msg_same = compare(a, b, strict=True)
        # 2. 今の使い方との差: YoloButtonDetector（.pt は横長のまま入る）
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        t = time.perf_counter()
        c_pt = det_pt.detect(rgb)
        t_pt += time.perf_counter() - t
        t = time.perf_counter()
        c_onnx = det_onnx.detect(rgb)
        t_onnx += time.perf_counter() - t
        use, msg_use = compare(from_candidates(c_pt), from_candidates(c_onnx), strict=False)
        if use:
            worst_use = max(worst_use, float(msg_use.rsplit(" ", 1)[-1]))
        n_same += same
        n_use += use
        print(f"[export] {name}\n    1. 同じモデルか {'OK' if same else '❌'}: {msg_same}\n"
              f"    2. 今の使い方との差 {'同じ' if use else '違う'}: {msg_use}")
    n = len(images)
    print(f"[export] 1. 同じモデルか: 一致 {n_same}/{n}（確からしさの差 {CONF_TOL} 以下・枠の重なり {IOU_MIN} 以上）")
    print(f"[export] 2. 今の使い方との差: 個数と矢印が同じ {n_use}/{n}、そのときの確からしさの差 最大 {worst_use:.3f}")
    print(f"[export] 1 枚の時間（候補づくりを含む。CPU）: .pt {t_pt / n * 1000:.0f} ms / .onnx {t_onnx / n * 1000:.0f} ms")
    print(f"[export] {'OK' if n_same == n else '❌ 同じモデルにならない画像がある'}。"
          f"PC2 へは {onnx} を写し、elevator.yaml の yolo.weights をそれにする")
    return 0 if n_same == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
