from dataclasses import dataclass

import numpy as np
import torch
from ultralytics import YOLO


@dataclass
class Detection:
    """1つの検出結果を表す。将来ROS2/SDK送信に渡す際もこの単位で扱う想定。"""

    class_name: str
    confidence: float
    # ピクセル座標系の (x1, y1, x2, y2)
    bbox: tuple[float, float, float, float]


class YoloDetector:
    """ultralytics YOLO を使った物体検出器。

    GPU(CUDA)が使える場合は自動でGPU推論、使えない場合はCPUにフォールバックする。
    """

    def __init__(
        self,
        model_name: str = "yolo26n.pt",
        classes: list[str] | None = None,
        confidence_threshold: float = 0.5,
        device: str = "auto",
    ) -> None:
        self._confidence_threshold = confidence_threshold
        self._device = self._resolve_device(device)

        self._model = YOLO(model_name)
        self._model.to(self._device)

        # クラス名フィルタ(例: ["person"]) -> ultralyticsのクラスIDリストに変換
        self._target_class_ids: list[int] | None = None
        if classes:
            name_to_id = {name: idx for idx, name in self._model.names.items()}
            missing = [c for c in classes if c not in name_to_id]
            if missing:
                raise ValueError(
                    f"モデルに存在しないクラス名が指定されました: {missing} "
                    f"(利用可能なクラスの一部: {list(name_to_id)[:10]}...)"
                )
            self._target_class_ids = [name_to_id[c] for c in classes]

    @staticmethod
    def _resolve_device(device: str) -> str:
        if device == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device='cuda' が指定されましたが、このマシンではCUDAが利用できません")
        return device

    @property
    def device(self) -> str:
        return self._device

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """1フレームに対して推論を行い、Detectionのリストを返す"""
        results = self._model.predict(
            frame,
            classes=self._target_class_ids,
            conf=self._confidence_threshold,
            device=self._device,
            verbose=False,
        )

        detections: list[Detection] = []
        for result in results:
            names = result.names
            for box in result.boxes:
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                detections.append(
                    Detection(
                        class_name=names[class_id],
                        confidence=confidence,
                        bbox=(x1, y1, x2, y2),
                    )
                )
        return detections
