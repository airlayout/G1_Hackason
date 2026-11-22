import numpy as np
import pytest
from adapters.camera_webcam import WebcamCamera, CameraError


def test_bgr_to_pil_rgb():
    frame = np.array([[[0, 10, 255]]], dtype=np.uint8)
    assert WebcamCamera.to_image(frame).getpixel((0, 0)) == (255, 10, 0)


def test_unavailable_camera_releases_every_attempt(monkeypatch):
    from adapters import camera_webcam
    released = []
    class ClosedCapture:
        def __init__(self, index, backend):
            self.backend = backend
        def isOpened(self):
            return False
        def release(self):
            released.append(self.backend)
    monkeypatch.setattr(camera_webcam.cv2, "VideoCapture", ClosedCapture)
    with pytest.raises(CameraError):
        WebcamCamera().open()
    assert len(released) == 2


def test_failed_read_still_closes_camera(monkeypatch):
    from adapters import camera_webcam
    released = []
    class Capture:
        def __init__(self, *args): pass
        def isOpened(self): return True
        def set(self, *args): return True
        def read(self): return False, None
        def release(self): released.append(True)
    monkeypatch.setattr(camera_webcam.cv2, "VideoCapture", Capture)
    camera = WebcamCamera()
    with pytest.raises(CameraError):
        with camera:
            camera.read()
    assert camera.capture is None and released == [True]
