from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Detection:
    """1つの検出結果を表す。将来ROS2/SDK送信に渡す際もこの単位で扱う想定。

    torch/ultralyticsに依存しないファイルに置く(G1のPC2でONNXの検出器だけを動かすときに、
    torchを読み込まずに済ませるため)。
    """

    class_name: str
    confidence: float
    # ピクセル座標系の (x1, y1, x2, y2)
    bbox: tuple[float, float, float, float]
