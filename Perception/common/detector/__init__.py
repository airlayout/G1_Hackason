from .onnx_yolo import OnnxYoloDetector
from .types import Detection

__all__ = ["Detection", "OnnxYoloDetector", "YoloDetector", "build_detector"]


def __getattr__(name: str):
    # YoloDetector(ultralytics)は使うときだけ読み込む。読み込むとtorchも読み込まれるため、
    # G1のPC2でONNXの検出器だけを使うとき(torchが無い環境)でもこのパッケージをimportできるようにする
    if name == "YoloDetector":
        from .yolo_detector import YoloDetector

        return YoloDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def build_detector(detector_cfg: dict):
    """config["detector"] の内容から検出器を組み立てる。

    backend: "ultralytics"(既定。YoloDetector、.ptの重み) | "onnx"(OnnxYoloDetector、.onnxの重み。torch不要)
             | "tensorrt"(TrtYoloDetector、PC2で作った.engine。GPUで動かす。torch・onnxruntime不要)
    """
    backend = detector_cfg.get("backend", "ultralytics")
    if backend == "tensorrt":
        from .trt_yolo import TrtYoloDetector

        return TrtYoloDetector(
            engine_path=detector_cfg.get("model", "yolo26n.engine"),
            classes=detector_cfg.get("classes"),
            confidence_threshold=detector_cfg.get("confidence_threshold", 0.5),
            iou_threshold=detector_cfg.get("iou_threshold", 0.7),
        )
    if backend == "onnx":
        return OnnxYoloDetector(
            model_path=detector_cfg.get("model", "yolo26n.onnx"),
            classes=detector_cfg.get("classes"),
            confidence_threshold=detector_cfg.get("confidence_threshold", 0.5),
            iou_threshold=detector_cfg.get("iou_threshold", 0.7),
        )
    if backend == "ultralytics":
        from .yolo_detector import YoloDetector

        return YoloDetector(
            model_name=detector_cfg.get("model", "yolo26n.pt"),
            classes=detector_cfg.get("classes"),
            confidence_threshold=detector_cfg.get("confidence_threshold", 0.5),
            device=detector_cfg.get("device", "auto"),
        )
    raise ValueError(f"Unknown detector.backend: {backend!r}")
