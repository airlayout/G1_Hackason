#!/usr/bin/env python3
"""Perception / sim 検証用エントリスクリプト（検出結果を画像として保存する版）。

run_sim.py との違いは「検出枠を描いた画像を保存するかどうか」だけ。
数値(console/JSONL/CSV)だけでは検出が正しいか人間が判断できないため、
目視で確認するために画像を残す。

  - run_sim.py        … 数値のみ。軽いので実機のCPUでの本番運用向け
  - run_sim_visual.py … 数値＋画像。開発・検証向け（このファイル）
  - probe_zmq_camera.py … 認識せず、画像取得だけを確認する

common/ の部品(FrameSource / YoloDetector / ResultWriter)はそのまま再利用し、
それらを繋ぐループだけを独自に持つ。common/pipeline.py の run_pipeline() と
ほぼ同じ構造なので、**pipeline.py の作りが変わったらこちらも追随が必要**。

使い方(シム側でZMQカメラ配信が有効な状態にしておくこと):
  cd Perception/sim
  ../../G1_HuggingFace/venv/bin/python run_sim_visual.py --max-frames 30
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.camera import build_frame_source  # noqa: E402
from common.detector import YoloDetector  # noqa: E402
from common.detector.yolo_detector import Detection  # noqa: E402
from common.output import ResultWriter  # noqa: E402
from common.timing import Timer  # noqa: E402

# 描画色(BGR順)。FrameSourceの契約がBGRなので、cv2の既定の解釈と一致する。
BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 255, 0)


def draw_detections(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """検出枠とラベルを描いた画像を新しく作って返す。

    cv2.rectangle / cv2.putText は渡した配列を直接書き換えるため、
    元のフレームを壊さないよう copy() してから描く。
    """
    canvas = frame.copy()
    for d in detections:
        # bbox は float。cv2 は int を要求するので変換する。
        x1, y1, x2, y2 = (int(v) for v in d.bbox)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), BOX_COLOR, thickness=2)

        # ラベルは英数字のみ(cv2.putText は日本語を描けない)。
        label = f"{d.class_name} {d.confidence:.2f}"
        # 枠の上に置くと画像外に出ることがあるため、上端付近では枠の内側に描く。
        text_y = y1 - 6 if y1 - 6 > 10 else y1 + 18
        cv2.putText(
            canvas,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            TEXT_COLOR,
            thickness=1,
            lineType=cv2.LINE_AA,
        )
    return canvas


def should_save(save_mode: str, detections: list[Detection]) -> bool:
    """設定と検出結果から、このフレームを保存するかどうかを決める"""
    if save_mode == "all":
        return True
    if save_mode == "detections_only":
        return len(detections) > 0
    return False  # "none"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Perception / sim: YOLO検出パイプライン(検出結果を画像として保存する版)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "configs" / "config.yaml"),
        help="設定ファイル(YAML)のパス",
    )
    parser.add_argument(
        "--source",
        choices=["zmq", "video", "webcam"],
        help="config.yamlのsource.typeを上書きする",
    )
    parser.add_argument(
        "--video-path",
        help="--source video のときに使う動画ファイルパス(config.yamlのsource.video.pathを上書き)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="処理するフレーム数の上限(動作確認用、指定しなければ無制限)",
    )
    parser.add_argument(
        "--save-images",
        choices=["none", "all", "detections_only"],
        help="画像の保存条件(config.yamlのoutput.save_imagesを上書き)",
    )
    parser.add_argument(
        "--image-dir",
        help="画像の保存先(config.yamlのoutput.image_dirを上書き)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if args.source:
        config["source"]["type"] = args.source
    if args.video_path:
        config["source"].setdefault("video", {})["path"] = args.video_path

    output_cfg = config["output"]
    save_mode = args.save_images or output_cfg.get("save_images", "detections_only")
    image_dir = Path(args.image_dir or output_cfg.get("image_dir", "images"))
    if save_mode != "none":
        image_dir.mkdir(parents=True, exist_ok=True)

    source = build_frame_source(config)

    detector_cfg = config["detector"]
    detector = YoloDetector(
        model_name=detector_cfg.get("model", "yolo26n.pt"),
        classes=detector_cfg.get("classes"),
        confidence_threshold=detector_cfg.get("confidence_threshold", 0.5),
        device=detector_cfg.get("device", "auto"),
    )
    print(f"[visual] detector device = {detector.device}", flush=True)
    print(f"[visual] 画像の保存条件 = {save_mode}", flush=True)
    if save_mode != "none":
        print(f"[visual] 画像の保存先 = {image_dir.resolve()}", flush=True)

    writer = ResultWriter(
        console=output_cfg.get("console", True),
        save_json=output_cfg.get("save_json", False),
        save_csv=output_cfg.get("save_csv", False),
        output_dir=output_cfg.get("output_dir", "outputs"),
    )

    frame_index = 0
    saved_count = 0
    detected_frames = 0
    with source:
        while args.max_frames is None or frame_index < args.max_frames:
            frame = source.read()
            if frame is None:
                print("[visual] フレームを取得できなくなったため終了します", flush=True)
                break

            with Timer() as t:
                detections = detector.detect(frame)

            writer.write(
                frame_index=frame_index,
                timestamp=time.time(),
                detections=detections,
                processing_time_ms=t.elapsed_ms,
            )

            if detections:
                detected_frames += 1

            if should_save(save_mode, detections):
                # JPEGにしているのはPNGより符号化が速いため(大量に保存するので効く)。
                path = image_dir / f"frame_{frame_index:06d}.jpg"
                # FrameSourceの契約はBGR。cv2.imwriteもBGR前提なので変換は不要。
                cv2.imwrite(str(path), draw_detections(frame, detections))
                saved_count += 1

            frame_index += 1

    print("\n=== 結果 ===", flush=True)
    print(f"処理フレーム数     : {frame_index}", flush=True)
    print(f"検出のあったフレーム: {detected_frames}", flush=True)
    print(f"保存した画像       : {saved_count}", flush=True)
    if saved_count:
        print(f"保存先             : {image_dir.resolve()}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
