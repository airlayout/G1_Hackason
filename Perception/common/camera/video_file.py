import cv2
import numpy as np

from .base import FrameSource


class VideoFileSource(FrameSource):
    """ローカルの動画ファイルから画像を取得する(ZMQストリームに接続できない場合のフォールバック用)"""

    def __init__(self, path: str, loop: bool = False) -> None:
        self._path = path
        self._loop = loop
        self._cap: cv2.VideoCapture | None = None

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._path)
        if not self._cap.isOpened():
            raise RuntimeError(f"動画ファイルを開けませんでした: {self._path}")

    def read(self) -> np.ndarray | None:
        if self._cap is None:
            raise RuntimeError("open() を呼び出す前に read() が呼ばれました")

        ok, frame = self._cap.read()
        if not ok:
            if self._loop:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._cap.read()
                if not ok:
                    return None
            else:
                return None
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
