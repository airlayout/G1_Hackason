#!/usr/bin/env python3
"""Perception / sim 検証用エントリスクリプト（検出結果を画像として保存・表示する版）。

run_sim.py との違いは「検出枠を描いた画像を保存・表示できるかどうか」。
数値(console/JSONL/CSV)だけでは検出が正しいか人間が判断できないため、
目視で確認するために画像を残す、またはその場でウィンドウに表示する。

  - run_sim.py        … 数値のみ。軽いので実機のCPUでの本番運用向け
  - run_sim_visual.py … 数値＋画像の保存・表示。開発・検証向け（このファイル）
  - probe_zmq_camera.py … 認識せず、画像取得だけを確認する

使い分け:
  - 評価(正解が分かっている動画で検出性能を測る) … --source video + --run-name
  - リアルタイム確認(カメラの前で枠が追従するか見る) … --source webcam/zmq + --show

common/ の部品(FrameSource / YoloDetector / ResultWriter)はそのまま再利用し、
それらを繋ぐループだけを独自に持つ。common/pipeline.py の run_pipeline() と
ほぼ同じ構造なので、**pipeline.py の作りが変わったらこちらも追随が必要**。
real/run_real_visual.py と処理は同一なので、**片方を直したらもう片方にも反映すること**。

使い方(シム側でZMQカメラ配信が有効な状態にしておくこと):
  cd Perception/sim
  ../../G1_HuggingFace/venv/bin/python run_sim_visual.py --max-frames 30
  ../../G1_HuggingFace/venv/bin/python run_sim_visual.py --source webcam --show   # qで終了
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
STATUS_COLOR = (0, 255, 255)

WINDOW_NAME = "Perception"
# 表示中にこのキーを押すと終了する(q または Esc)
QUIT_KEYS = (ord("q"), 27)
# 表示するfpsの平滑化係数。1フレームごとの揺れで数字が読めなくなるのを防ぐ。
FPS_SMOOTHING = 0.9


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


def draw_status(canvas: np.ndarray, fps: float, inference_ms: float, num_detections: int) -> None:
    """表示用の画像の左上に、処理速度と検出数を書き込む(canvasを直接書き換える)。

    保存する画像には描かない(評価用の画像の上に文字が重なって検出枠が見づらくなるため)。
    """
    text = f"{fps:.1f} fps  infer {inference_ms:.0f} ms  detections {num_detections}"
    # 背景に関係なく読めるよう、太い黒で縁取ってから色付きの文字を重ねる。
    cv2.putText(canvas, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, STATUS_COLOR, 1, cv2.LINE_AA)


def should_save(save_mode: str, detections: list[Detection]) -> bool:
    """設定と検出結果から、このフレームを保存するかどうかを決める"""
    if save_mode == "all":
        return True
    if save_mode == "detections_only":
        return len(detections) > 0
    return False  # "none"


def open_window() -> str | None:
    """表示用のウィンドウを開く。開けなければその理由を返す。

    opencv-python-headless(表示機能を持たないビルド)が読み込まれている場合や、
    ディスプレイが無い環境では cv2.namedWindow が例外になる。モデルの読み込みや
    カメラへの接続より前に確かめ、原因が分かる形で早めに止めるために使う。
    """
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    except cv2.error as e:
        return str(e).strip()
    return None


def window_closed() -> bool:
    """ウィンドウの×ボタンで閉じられたかどうか"""
    return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Perception / sim: YOLO検出パイプライン(検出結果を画像として保存・表示する版)",
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
        "--run-name",
        help=(
            "この実行の名前。結果(画像・run.csv・run.jsonl)は "
            "config.yamlのoutput.visual_dir/<run-name>/ にまとめて出力される。"
            "省略すると日時(例: 2026-09-17_1830)になるため、実行ごとに別フォルダになる"
        ),
    )
    parser.add_argument(
        "--visual-dir",
        help="実行フォルダを作る親ディレクトリ(config.yamlのoutput.visual_dirを上書き)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="検出枠を描いた映像をウィンドウに表示する(q または Esc で終了)",
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

    # 表示できない環境なら、重いモデルの読み込みより前に止める。
    if args.show:
        reason = open_window()
        if reason is not None:
            print("[visual] エラー: --show を指定しましたが、ウィンドウを開けませんでした。", flush=True)
            print(f"[visual]   OpenCVのメッセージ: {reason}", flush=True)
            print(
                "[visual]   多くの場合、表示機能を持たない opencv-python-headless が読み込まれています。\n"
                "[visual]   確認: python -c \"import cv2; print(cv2.getBuildInformation())\" の GUI 欄が NONE なら該当。\n"
                "[visual]   対処は Perception/sim/README.md の「--show が使えないとき」を参照。",
                flush=True,
            )
            return 1

    output_cfg = config["output"]
    save_mode = args.save_images or output_cfg.get("save_images", "detections_only")

    # 1回の実行 = 1フォルダ。画像も数値も同じ場所にまとめる。
    # 名前を省略すると日時になるので、何もしなくても実行ごとに別フォルダになり、
    # 前回の結果に追記されて混ざることがない(run.csv/run.jsonl は追記方式のため)。
    visual_base = Path(args.visual_dir or output_cfg.get("visual_dir", "visual"))
    run_name = args.run_name or time.strftime("%Y-%m-%d_%H%M")
    run_dir = visual_base / run_name
    image_dir = run_dir / "images"
    run_dir.mkdir(parents=True, exist_ok=True)
    if (run_dir / "run.csv").exists() or (run_dir / "run.jsonl").exists():
        print(
            f"[visual] 警告: {run_dir} には既に結果がある。追記されるため、"
            "条件ごとに比較するなら別の --run-name を指定すること",
            flush=True,
        )
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
    print(f"[visual] この実行の出力先 = {run_dir.resolve()}", flush=True)
    print(f"[visual] 画像の保存条件 = {save_mode}", flush=True)
    if args.show:
        print("[visual] ウィンドウに表示します(q または Esc で終了)", flush=True)

    writer = ResultWriter(
        console=output_cfg.get("console", True),
        save_json=output_cfg.get("save_json", False),
        save_csv=output_cfg.get("save_csv", False),
        output_dir=str(run_dir),
    )

    frame_index = 0
    saved_count = 0
    detected_frames = 0
    # 1フレームあたりの時間の内訳(ミリ秒)。最後に平均を表示する。
    total_read_ms = 0.0
    total_infer_ms = 0.0
    total_post_ms = 0.0
    display_fps = 0.0
    quit_reason = ""

    try:
        with source:
            while args.max_frames is None or frame_index < args.max_frames:
                loop_start = time.perf_counter()

                # 取得: ZMQ・Webカメラでは「次の1枚が届くのを待つ時間」も含む。
                with Timer() as t_read:
                    frame = source.read()
                if frame is None:
                    quit_reason = "フレームを取得できなくなった"
                    break

                with Timer() as t_infer:
                    detections = detector.detect(frame)

                post_start = time.perf_counter()
                writer.write(
                    frame_index=frame_index,
                    timestamp=time.time(),
                    detections=detections,
                    processing_time_ms=t_infer.elapsed_ms,
                )

                if detections:
                    detected_frames += 1

                stop = False
                save_this = should_save(save_mode, detections)
                if save_this or args.show:
                    canvas = draw_detections(frame, detections)

                    if save_this:
                        # JPEGにしているのはPNGより符号化が速いため(大量に保存するので効く)。
                        # FrameSourceの契約はBGR。cv2.imwriteもBGR前提なので変換は不要。
                        cv2.imwrite(str(image_dir / f"frame_{frame_index:06d}.jpg"), canvas)
                        saved_count += 1

                    if args.show:
                        draw_status(canvas, display_fps, t_infer.elapsed_ms, len(detections))
                        cv2.imshow(WINDOW_NAME, canvas)
                        # waitKeyを呼ばないとウィンドウが描画されない(1msだけ待つ)。
                        key = cv2.waitKey(1) & 0xFF
                        if key in QUIT_KEYS:
                            quit_reason = "q または Esc が押された"
                            stop = True
                        elif window_closed():
                            quit_reason = "ウィンドウが閉じられた"
                            stop = True

                post_ms = (time.perf_counter() - post_start) * 1000.0
                loop_ms = (time.perf_counter() - loop_start) * 1000.0

                total_read_ms += t_read.elapsed_ms
                total_infer_ms += t_infer.elapsed_ms
                total_post_ms += post_ms
                if loop_ms > 0:
                    instant_fps = 1000.0 / loop_ms
                    display_fps = (
                        instant_fps
                        if display_fps == 0.0
                        else FPS_SMOOTHING * display_fps + (1 - FPS_SMOOTHING) * instant_fps
                    )

                frame_index += 1
                # 終了操作があっても、このフレームの集計を済ませてから抜ける。
                if stop:
                    break
    finally:
        if args.show:
            cv2.destroyAllWindows()

    if quit_reason:
        print(f"[visual] 終了: {quit_reason}", flush=True)

    print("\n=== 結果 ===", flush=True)
    print(f"処理フレーム数     : {frame_index}", flush=True)
    print(f"検出のあったフレーム: {detected_frames}", flush=True)
    print(f"保存した画像       : {saved_count}", flush=True)
    print(f"出力先             : {run_dir.resolve()}", flush=True)
    if frame_index > 0:
        avg_read = total_read_ms / frame_index
        avg_infer = total_infer_ms / frame_index
        avg_post = total_post_ms / frame_index
        avg_total = avg_read + avg_infer + avg_post
        print("1フレームあたりの平均時間:", flush=True)
        print(f"  取得   : {avg_read:7.1f} ms  (カメラ/動画から1枚取り出す。待ち時間を含む)", flush=True)
        print(f"  推論   : {avg_infer:7.1f} ms", flush=True)
        print(f"  後処理 : {avg_post:7.1f} ms  (結果の書き出し・描画・保存・表示)", flush=True)
        if avg_total > 0:
            print(f"  合計   : {avg_total:7.1f} ms  → 実効 {1000.0 / avg_total:.1f} fps", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
