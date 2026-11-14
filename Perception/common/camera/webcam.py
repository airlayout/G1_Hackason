from __future__ import annotations

import cv2
import numpy as np

from .base import FrameSource


def parse_device(value: str) -> int | str:
    """コマンドラインの --webcam を、番号(int)か、デバイスのパス(str)にする。

    /dev/videoN の番号は再起動や挿し直しで入れ替わる(2026-10-31、G1のPC2で再起動後に別のカメラが video6 になった)。
    同じカメラを確実に開くには /dev/v4l/by-id/usb-<メーカー>_<名前>_<シリアル>-video-index0 のようなパスを使う。
    """
    return int(value) if value.isdigit() else value


class WebcamSource(FrameSource):
    """OpenCVのVideoCapture経由でWebカメラから画像を取得する。device_index は番号か、デバイスのパス(/dev/v4l/by-id/...)。"""

    def __init__(self, device_index: int | str = 0) -> None:
        self._device_index = device_index
        self._cap: cv2.VideoCapture | None = None

    def open(self) -> None:
        if isinstance(self._device_index, str):
            self._cap = cv2.VideoCapture(self._device_index, cv2.CAP_V4L2)  # パスで開けるのはV4L2(Linux)だけ
        else:
            self._cap = cv2.VideoCapture(self._device_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Webカメラ(device_index={self._device_index})を開けませんでした")

    def read(self) -> np.ndarray | None:
        if self._cap is None:
            raise RuntimeError("open() を呼び出す前に read() が呼ばれました")
        ok, frame = self._cap.read()
        if not ok:
            return None
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
