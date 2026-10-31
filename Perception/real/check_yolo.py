#!/usr/bin/env python3
"""YOLO(人の検出)がこのマシンで動くか・どれくらいの速さかを確かめる。G1のPC2で動かすための確認用(ラボPCでも動く)。

    # G1の頭カメラ(PC2で動いている配信 rgbd_server.py / run_g1_server.py --camera の5555番)をPC2の中から受け取る
    python Perception/real/check_yolo.py --zmq localhost:5555 --model ~/perception/yolo26n.onnx --frames 100
    # 保存した画像で(まず1枚で動くかを見る)
    python Perception/real/check_yolo.py --image bus.jpg --model ~/perception/yolo26n.onnx
    # Webカメラ(OpenCVのデバイス番号)
    python Perception/real/check_yolo.py --webcam 0 --model ~/perception/yolo26n.onnx --frames 100
    # ultralytics(torch)で比べる
    python Perception/real/check_yolo.py --image bus.jpg --backend ultralytics --model yolo26n.pt
    # PC2のGPU(TensorRT)で。システムの Python 3.8 で動かす(エンジンの作り方は common/detector/trt_yolo.py)
    /usr/bin/python3 real/check_yolo.py --webcam 6 --rotate cw --backend tensorrt --model yolo26n.engine --frames 100

--backend onnx(既定)は torch/ultralytics を使わない(onnxruntime・opencv・numpy・pyzmq・PyYAML だけ)。
表示するもの: 1枚にかかる時間(検出だけ・平均と90%)、受け取りを含めた毎秒の枚数、このプロセスのCPU使用率、検出した人の数。
人が写った画像(枠を描いたもの)を --save-max 枚まで保存し、まとめを summary.json に残す(--out-dir、既定は _local/perception/check_yolo/<日時>)。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.detector import build_detector  # noqa: E402

# リポジトリの中(Perception/real/)ならリポジトリ直下、PC2 に写した ~/perception/real/ なら ~/perception を基準にする
_HERE = Path(__file__).resolve()
REPO_ROOT = _HERE.parents[2] if (_HERE.parents[2] / "Perception").is_dir() else _HERE.parents[1]

ROTATIONS = {"cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", nargs="+", help="画像ファイル(複数可)")
    src.add_argument("--zmq", metavar="HOST:PORT", help="ZMQのRGB配信(run_g1_server.py --camera と同じ形式)。PC2の中なら localhost:5555")
    src.add_argument("--webcam", type=int, metavar="INDEX", help="Webカメラ(OpenCVのデバイス番号)")
    p.add_argument("--backend", choices=["onnx", "ultralytics", "tensorrt"], default="onnx")
    p.add_argument("--model", required=True, help="onnx なら .onnx(Perception/real/export_onnx.py で書き出す)、ultralytics なら .pt、"
                                                  "tensorrt なら PC2 で作った .engine(隣に .names.json が要る)")
    p.add_argument("--classes", nargs="*", default=["person"], help="検出するクラス(既定 person。空なら全部)")
    p.add_argument("--conf", type=float, default=0.5, help="確からしさのしきい値")
    p.add_argument("--frames", type=int, default=50, help="--zmq / --webcam で処理する枚数")
    p.add_argument("--camera-name", default="head_camera", help="--zmq の画像の名前")
    p.add_argument("--rotate", choices=["cw", "ccw", "180"],
                   help="検出の前に画像を回す(cw=時計回り90°)。G1 の PC2 に挿した Web カメラは横向きに付いていて cw で正しい向きになる")
    p.add_argument("--threads", type=int, help="onnxruntime のスレッド数(既定はonnxruntimeに任せる。PC2では他の処理と分け合う)")
    p.add_argument("--save-max", type=int, default=20, help="人が写った画像を保存する上限")
    p.add_argument("--out-dir", type=Path)
    return p.parse_args()


def frames(args: argparse.Namespace):
    """(名前, BGR画像) を順に返す。"""
    if args.image:
        for path in args.image:
            img = cv2.imread(path)
            if img is None:
                print(f"[check_yolo] ❌ 画像を読めない: {path}")
                continue
            yield Path(path).name, img
        return
    if args.zmq:
        from common.camera.zmq_camera import ZmqFrameSource

        host, port = args.zmq.rsplit(":", 1)
        source = ZmqFrameSource(server_address=host, port=int(port), camera_name=args.camera_name, timeout_ms=5000)
    else:
        from common.camera.webcam import WebcamSource

        source = WebcamSource(device_index=args.webcam)
    with source:
        for i in range(args.frames):
            img = source.read()
            if img is None:
                print("[check_yolo] ❌ 画像が届かない(配信が動いているか・接続先を確かめる)")
                return
            yield f"{i:05d}", img


def draw(img: np.ndarray, dets: list) -> np.ndarray:
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d.bbox)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, f"{d.class_name} {d.confidence:.2f}", (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1)
    return out


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir or REPO_ROOT / "_local" / "perception" / "check_yolo" / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = {"backend": args.backend, "model": os.path.expanduser(args.model), "classes": args.classes or None,
           "confidence_threshold": args.conf, "device": "cpu"}
    t0 = time.perf_counter()
    detector = build_detector(cfg)
    if args.threads and args.backend == "onnx":
        from common.detector.onnx_yolo import OnnxYoloDetector

        detector = OnnxYoloDetector(cfg["model"], cfg["classes"], args.conf, num_threads=args.threads)
    print(f"[check_yolo] 検出器 {args.backend}（{detector.device}）、読み込み {time.perf_counter() - t0:.2f} 秒、モデル {cfg['model']}")

    times_ms: list[float] = []
    counts: list[int] = []
    saved = 0
    shape = None
    wall0, cpu0 = time.perf_counter(), time.process_time()
    for name, img in frames(args):
        if args.rotate:
            img = cv2.rotate(img, ROTATIONS[args.rotate])
        shape = img.shape
        t = time.perf_counter()
        dets = detector.detect(img)
        times_ms.append((time.perf_counter() - t) * 1000.0)
        counts.append(len(dets))
        if dets and saved < args.save_max:
            cv2.imwrite(str(out_dir / f"{name}_n{len(dets)}.jpg"), draw(img, dets))
            saved += 1
        if args.image or len(times_ms) % 10 == 0:
            print(f"[check_yolo] {name}: {times_ms[-1]:.1f} ms、検出 {len(dets)}"
                  + ("".join(f"  {d.class_name} {d.confidence:.2f}" for d in dets[:5]) if args.image else ""))
    wall, cpu = time.perf_counter() - wall0, time.process_time() - cpu0
    if not times_ms:
        print("[check_yolo] ❌ 1枚も処理できなかった")
        return 1
    # 1枚目はonnxruntimeの準備で遅いことがあるので、2枚目以降で平均を出す(1枚しかなければ1枚目)
    steady = times_ms[1:] or times_ms
    summary = {
        "backend": args.backend, "device": detector.device, "model": cfg["model"], "frames": len(times_ms),
        "image_shape": list(shape) if shape is not None else None,
        "detect_ms_first": round(times_ms[0], 1), "detect_ms_mean": round(float(np.mean(steady)), 1),
        "detect_ms_p90": round(float(np.percentile(steady, 90)), 1),
        "fps_with_receive": round(len(times_ms) / wall, 2) if wall > 0 else None,
        "cpu_percent_of_one_core": round(100.0 * cpu / wall, 0) if wall > 0 else None,
        "frames_with_detection": int(sum(c > 0 for c in counts)), "saved_images": saved, "out_dir": str(out_dir),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"[check_yolo] {len(times_ms)} 枚: 検出 平均 {summary['detect_ms_mean']} ms（90% {summary['detect_ms_p90']} ms、1枚目 "
          f"{summary['detect_ms_first']} ms）、受け取りを含めて {summary['fps_with_receive']} 枚/秒、CPU {summary['cpu_percent_of_one_core']}%"
          f"（1コア=100%）、検出があった枚数 {summary['frames_with_detection']}")
    print(f"[check_yolo] 保存: {out_dir}（summary.json、人が写った画像 {saved} 枚）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
