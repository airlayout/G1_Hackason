import cv2
import numpy as np

from .base import FrameSource


class WebcamSource(FrameSource):
    """OpenCVのVideoCapture経由でローカルWebカメラから画像を取得する(手元での動作確認用)"""

    def __init__(self, device_index: int = 0) -> None:
        self._device_index = device_index
        self._cap: cv2.VideoCapture | None = None

    def open(self) -> None:
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
