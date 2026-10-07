import ast
from pathlib import Path

import numpy as np
import pytest

from adapters.pepper.camera import PepperCameraError, PepperCameraSource, decode_remote_image


def remote_image(payload=None, width=2, height=1, layers=3, colorspace=11):
    if payload is None:
        payload = bytes([1, 2, 3, 4, 5, 6])
    return [width, height, layers, colorspace, 10, 250_000, payload]


def test_payload_decode_and_rgb_to_bgr_boundary():
    frame = decode_remote_image(remote_image(), request_started=1.0, received=1.25, received_wall=20.0)
    assert frame.rgb.tolist() == [[[1, 2, 3], [4, 5, 6]]]
    assert frame.bgr.tolist() == [[[3, 2, 1], [6, 5, 4]]]
    assert frame.payload_bytes == 6 and frame.pepper_timestamp_s == 10.25
    assert frame.acquisition_latency_s == pytest.approx(.25)
    assert not frame.rgb.flags.writeable and not frame.bgr.flags.writeable


@pytest.mark.parametrize("image", [
    None,
    [1, 2, 3],
    remote_image(b"short"),
    remote_image(colorspace=9),
    remote_image(layers=2),
])
def test_malformed_payload_rejected(image):
    with pytest.raises(PepperCameraError):
        decode_remote_image(image)


class FakeVideo:
    def __init__(self, image=None, error=None):
        self.image = remote_image() if image is None else image
        self.error = error
        self.unsubscribed = []

    def subscribeCamera(self, name, camera, resolution, colorspace, fps):
        assert (camera, resolution, colorspace, fps) == (0, 1, 11, 5)
        return name + "_0"

    def getImageRemote(self, subscriber):
        if self.error:
            raise self.error
        return self.image

    def unsubscribe(self, subscriber):
        self.unsubscribed.append(subscriber)
        return True


class FakeSession:
    def __init__(self, video):
        self.video = video
        self.connected = None

    def connect(self, url):
        self.connected = url

    def service(self, name):
        assert name == "ALVideoDevice"
        return self.video


def test_unsubscribe_on_read_exception():
    video = FakeVideo(error=RuntimeError("read failed"))
    source = PepperCameraSource("192.0.2.1", session_factory=lambda: FakeSession(video))
    with pytest.raises(RuntimeError):
        with source:
            source.read()
    assert len(video.unsubscribed) == 1


def test_session_is_retained_until_unsubscribe():
    events = []
    class LifetimeSession(FakeSession):
        def __del__(self):
            events.append("session_deleted")
    class LifetimeVideo(FakeVideo):
        def unsubscribe(self, subscriber):
            assert "session_deleted" not in events
            events.append("unsubscribed")
            return True
    video = LifetimeVideo()
    source = PepperCameraSource("192.0.2.1", session_factory=lambda: LifetimeSession(video))
    source.open()
    assert source.close()
    assert events[0] == "unsubscribed"


def test_pepper_dependency_isolation_and_no_motion_calls():
    root = Path(__file__).parents[1]
    brain_text = "\n".join(path.read_text() for path in (root / "brain").glob("*.py"))
    assert "import qi" not in brain_text and "from qi" not in brain_text
    source = (root / "adapters" / "pepper" / "camera.py").read_text()
    tree = ast.parse(source)
    service_names = [
        node.args[0].value for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == "service" and node.args and isinstance(node.args[0], ast.Constant)
    ]
    assert service_names == ["ALVideoDevice"]
    for forbidden in ("ALMotion", "ALTextToSpeech", "ALLeds", "ALNavigation"):
        assert forbidden not in source


def test_pepper_producer_has_latest_slot_pacing_not_a_fifo():
    source = (Path(__file__).parents[1] / "hybrid" / "pepper.py").read_text()
    tree = ast.parse(source)
    assert "1 / self.fps" in source
    assert "Queue(" not in source and "deque(" not in source
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
               and node.func.attr == "publish" for node in ast.walk(tree))
