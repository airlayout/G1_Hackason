import time

from .camera import build_frame_source
from .detector import build_detector
from .output import ResultWriter
from .timing import Timer


def run_pipeline(config: dict, max_frames: int | None = None) -> None:
    """FrameSource -> 検出器(YoloDetector / OnnxYoloDetector) -> ResultWriter を繋いでループ実行する

    sim/・real/どちらの設定(config["source"]["type"])でも共通で使う。
    max_frames: 指定した場合、そのフレーム数で処理を打ち切る(動作確認用)
    """
    source = build_frame_source(config)

    # backend: "ultralytics"(既定) | "onnx"(G1のPC2など、torchが無い環境。common/detector/onnx_yolo.py)
    detector = build_detector(config["detector"])
    print(f"[pipeline] detector device = {detector.device}")

    output_cfg = config["output"]
    writer = ResultWriter(
        console=output_cfg.get("console", True),
        save_json=output_cfg.get("save_json", False),
        save_csv=output_cfg.get("save_csv", False),
        output_dir=output_cfg.get("output_dir", "outputs"),
    )

    frame_index = 0
    with source:
        while max_frames is None or frame_index < max_frames:
            frame = source.read()
            if frame is None:
                print("[pipeline] フレームを取得できなくなったため終了します")
                break

            with Timer() as t:
                detections = detector.detect(frame)

            writer.write(
                frame_index=frame_index,
                timestamp=time.time(),
                detections=detections,
                processing_time_ms=t.elapsed_ms,
            )

            frame_index += 1

    print(f"[pipeline] 終了。処理フレーム数: {frame_index}")
