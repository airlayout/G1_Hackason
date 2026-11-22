"""Read-only Pepper top-camera adapter with qi isolated behind this boundary."""

from dataclasses import dataclass
from time import perf_counter, time
import uuid

import numpy as np


TOP_CAMERA = 0
QVGA = 1
RGB_COLORSPACE = 11


class PepperCameraError(RuntimeError):
    pass


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray
    bgr: np.ndarray
    width: int
    height: int
    layers: int
    colorspace: int
    pepper_timestamp_s: float
    host_receive_timestamp: float
    host_receive_monotonic: float
    request_started_monotonic: float
    acquisition_latency_s: float
    payload_bytes: int


def _payload_bytes(payload):
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, bytearray):
        return bytes(payload)
    if hasattr(payload, "data"):
        data = payload.data()
        return data if isinstance(data, bytes) else bytes(data)
    try:
        return bytes(payload)
    except Exception as exc:
        raise PepperCameraError(f"Unsupported image payload: {type(payload).__name__}") from exc


def decode_remote_image(image, *, request_started=None, received=None, received_wall=None):
    """Decode ALVideoDevice getImageRemote result and reject malformed metadata/data."""
    if not isinstance(image, (list, tuple)) or len(image) < 7:
        raise PepperCameraError("ALVideoDevice image must contain at least 7 fields")
    width, height, layers, colorspace = image[:4]
    if any(type(value) is not int for value in (width, height, layers, colorspace)):
        raise PepperCameraError("Image dimensions, layers and colorspace must be integers")
    if width <= 0 or height <= 0 or layers != 3 or colorspace != RGB_COLORSPACE:
        raise PepperCameraError(
            f"Expected positive RGB image with 3 layers/colorspace 11; got {width}x{height}x{layers}, {colorspace}"
        )
    seconds, microseconds = image[4], image[5]
    if not isinstance(seconds, (int, float)) or not isinstance(microseconds, (int, float)):
        raise PepperCameraError("Pepper timestamp fields must be numeric")
    payload = _payload_bytes(image[6])
    expected = width * height * layers
    if len(payload) != expected:
        raise PepperCameraError(f"Payload is {len(payload)} bytes; expected {expected}")
    rgb = np.frombuffer(payload, dtype=np.uint8).reshape((height, width, layers))
    # OpenCV and the existing YOLO path consume BGR. Copy both arrays so no qi
    # transport buffer can be retained after unsubscribe.
    rgb = np.ascontiguousarray(rgb).copy()
    bgr = np.ascontiguousarray(rgb[:, :, ::-1]).copy()
    rgb.setflags(write=False)
    bgr.setflags(write=False)
    received = perf_counter() if received is None else received
    request_started = received if request_started is None else request_started
    received_wall = time() if received_wall is None else received_wall
    return CameraFrame(
        rgb=rgb,
        bgr=bgr,
        width=width,
        height=height,
        layers=layers,
        colorspace=colorspace,
        pepper_timestamp_s=float(seconds) + float(microseconds) / 1_000_000,
        host_receive_timestamp=float(received_wall),
        host_receive_monotonic=float(received),
        request_started_monotonic=float(request_started),
        acquisition_latency_s=float(received) - float(request_started),
        payload_bytes=len(payload),
    )


class PepperCameraSource:
    """Credential-less qi camera source; no motion-capable service is requested."""

    def __init__(self, ip, *, port=9559, fps=5, subscriber_prefix="robot-vlm-brain-probe",
                 session_factory=None):
        if not 1 <= int(fps) <= 30:
            raise ValueError("fps must be in 1..30")
        self.ip = str(ip)
        self.port = int(port)
        self.fps = int(fps)
        self.subscriber_prefix = str(subscriber_prefix)
        self._session_factory = session_factory
        self.session = None
        self.video = None
        self.subscriber = None

    def _new_session(self):
        if self._session_factory is not None:
            return self._session_factory()
        import qi  # Pepper-only dependency: never imported by the Brain modules.
        return qi.Session()

    def open(self):
        if self.subscriber is not None:
            raise PepperCameraError("Camera is already open")
        self.session = self._new_session()
        self.session.connect(f"tcp://{self.ip}:{self.port}")
        self.video = self.session.service("ALVideoDevice")
        requested = f"{self.subscriber_prefix}-{uuid.uuid4().hex[:8]}"
        try:
            self.subscriber = self.video.subscribeCamera(
                requested, TOP_CAMERA, QVGA, RGB_COLORSPACE, self.fps
            )
        except Exception:
            self.video = None
            self.session = None
            raise
        return self

    def read(self):
        if self.subscriber is None or self.video is None:
            raise PepperCameraError("Camera is not open")
        started = perf_counter()
        image = self.video.getImageRemote(self.subscriber)
        received = perf_counter()
        if not image:
            raise PepperCameraError("getImageRemote returned no image")
        return decode_remote_image(image, request_started=started, received=received)

    def close(self):
        # Keep the qi Session alive until unsubscribe has completed. Dropping
        # the last Session reference first disconnects the transport while the
        # proxy still exists and turns a successful capture into a close error.
        session, subscriber, video = self.session, self.subscriber, self.video
        self.subscriber = None
        self.video = None
        try:
            if subscriber is not None and video is not None:
                return video.unsubscribe(subscriber)
            return None
        finally:
            self.session = None
            del session

    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.close()
