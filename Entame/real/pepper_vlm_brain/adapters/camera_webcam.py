"""OpenCV camera ownership stays outside the Brain."""
import cv2
from PIL import Image


class CameraError(RuntimeError):
    pass


class WebcamCamera:
    def __init__(self, index=0, backend="auto", width=640, height=480):
        self.index, self.backend = index, backend
        self.width, self.height = width, height
        self.capture = None

    def open(self):
        if self.capture is not None:
            return self
        choices = {"auto": [cv2.CAP_DSHOW, cv2.CAP_MSMF],
                   "dshow": [cv2.CAP_DSHOW], "msmf": [cv2.CAP_MSMF]}
        if self.backend not in choices:
            raise ValueError("backend must be auto, dshow, or msmf")
        for backend in choices[self.backend]:
            capture = cv2.VideoCapture(self.index, backend)
            if capture.isOpened():
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.capture = capture
                return self
            capture.release()
        raise CameraError(f"Webcam {self.index} cannot be opened. Check connection, "
                          "Windows camera permissions and competing camera apps.")

    def read(self):
        if self.capture is None:
            raise CameraError("Camera is not open")
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise CameraError("Webcam frame read failed")
        return frame

    @staticmethod
    def to_image(frame):
        return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.close()
