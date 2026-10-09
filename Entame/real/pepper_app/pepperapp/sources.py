"""Frame sources: a video or image file, a USB camera (V4L2 on Linux), or Pepper's top camera."""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .naoqi_robot import _default_session, naoqi_url

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
# ALVideoDevice constants
TOP_CAMERA, RESOLUTIONS, RGB_COLORSPACE = 0, {"QVGA": 1, "VGA": 2}, 11


class SourceError(RuntimeError):
    pass


def fit_width(frame: np.ndarray, max_width: int) -> np.ndarray:
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    return cv2.resize(frame, (max_width, round(height * max_width / width)), interpolation=cv2.INTER_AREA)


class FileSource:
    """Plays a video at its own frame rate (looping), or repeats a still image.

    Frames wider than max_width are shrunk to it (Pepper's VGA width by default).
    """

    def __init__(self, path: str | Path, fps_for_images: float = 10.0, max_width: int = 640):
        self.path = Path(path).expanduser()
        if not self.path.is_file():
            raise SourceError(f"ファイルがありません: {self.path}")
        self.name = f"ファイル {self.path.name}"
        self._still = None
        self._capture = None
        self.max_width = max_width
        if self.path.suffix.lower() in IMAGE_SUFFIXES:
            still = cv2.imread(str(self.path))
            if still is None:
                raise SourceError(f"画像を読めません: {self.path}")
            self._still = fit_width(still, max_width)
            self._period = 1.0 / fps_for_images
        else:
            self._capture = cv2.VideoCapture(str(self.path))
            if not self._capture.isOpened():
                raise SourceError(f"動画を開けません: {self.path}")
            fps = self._capture.get(cv2.CAP_PROP_FPS) or 30.0
            self._period = 1.0 / fps
        self._next = time.monotonic()

    def read(self) -> np.ndarray:
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next = max(self._next + self._period, time.monotonic())
        if self._still is not None:
            return self._still.copy()
        ok, frame = self._capture.read()
        if not ok:
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
        if not ok or frame is None:
            raise SourceError("動画のフレームを読めません")
        return fit_width(frame, self.max_width)

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()


class WebcamSource:
    def __init__(self, index: int = 0, width: int = 640, height: int = 480):
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
        self._capture = cv2.VideoCapture(int(index), backend)
        if not self._capture.isOpened():
            raise SourceError(f"USB カメラ {index} を開けません（接続と他のアプリの使用を確認）")
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.name = f"USB カメラ {index}"

    def read(self) -> np.ndarray:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise SourceError("USB カメラから読めません")
        return frame

    def close(self) -> None:
        self._capture.release()


def decode_image(image) -> np.ndarray:
    """ALVideoDevice.getImageRemote result -> BGR array. Rejects malformed payloads."""
    if not isinstance(image, (list, tuple)) or len(image) < 7:
        raise SourceError("ALVideoDevice の画像の形式が違います")
    width, height, layers, colorspace = (int(v) for v in image[:4])
    if layers != 3 or colorspace != RGB_COLORSPACE or width <= 0 or height <= 0:
        raise SourceError(f"RGB 3 層の画像を期待したが {width}x{height}x{layers} / {colorspace}")
    raw = image[6]
    if hasattr(raw, "data") and callable(raw.data):
        raw = raw.data()
    # qi delivers NAOqi's binary (Raw) as bytearray.
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise SourceError(f"画像データの型が想定外です: {type(raw).__name__}")
    payload = bytes(raw)
    if len(payload) != width * height * 3:
        raise SourceError(f"画像のバイト数 {len(payload)} が {width * height * 3} と合いません")
    rgb = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3)
    return np.ascontiguousarray(rgb[:, :, ::-1])


class PepperCameraSource:
    """Pepper top camera through ALVideoDevice on the NAOqi API (port 9559)."""

    def __init__(self, ip: str, port: int = 9559, resolution: str = "VGA", fps: int = 10,
                 session_factory=_default_session):
        if resolution not in RESOLUTIONS:
            raise ValueError(f"resolution must be one of {list(RESOLUTIONS)}")
        if not 1 <= int(fps) <= 30:
            raise ValueError("fps must be 1..30")
        self.session = session_factory()
        self.session.connect(naoqi_url(ip, port))
        try:
            self.video = self.session.service("ALVideoDevice")
            self.handle = self.video.subscribeCamera("pepper-app", TOP_CAMERA,
                                                     RESOLUTIONS[resolution], RGB_COLORSPACE, int(fps))
        except Exception:
            self.session.close()
            raise
        self.name = f"Pepper {ip} {resolution} {fps}fps"

    def read(self) -> np.ndarray:
        image = self.video.getImageRemote(self.handle)
        if not image:
            raise SourceError("Pepper のカメラから画像が来ません")
        return decode_image(image)

    def close(self) -> None:
        try:
            self.video.unsubscribe(self.handle)
        finally:
            self.session.close()
