"""NaoqiRobot / PepperCameraSource through the real qi library against a local qi service.

The services below only record calls; they stand in for Pepper's ALMotion etc. so that the
argument types, method names and connection handling go through the actual qi transport.
"""
import time

import numpy as np
import pytest

qi = pytest.importorskip("qi")

from pepperapp.naoqi_robot import NaoqiRobot, naoqi_url, read_pepper_info  # noqa: E402
from pepperapp.robot import ActionExecutor, LookConfig  # noqa: E402
from pepperapp.rules import Command  # noqa: E402
from pepperapp.observations import Target  # noqa: E402
from pepperapp.sources import PepperCameraSource, SourceError, decode_image  # noqa: E402

LOCOMOTION = ("moveTo", "move", "moveToward", "setWalkTargetVelocity", "navigateTo")


class Recorder:
    def __init__(self, calls):
        self.calls = calls


class Motion(Recorder):
    head = [0.1, -0.2]

    def getAngles(self, names, use_sensors):
        self.calls.append(("getAngles", list(names), use_sensors))
        return list(self.head)

    def angleInterpolationWithSpeed(self, names, angles, speed):
        self.calls.append(("angleInterpolationWithSpeed", list(names),
                           [round(a, 4) for a in angles], speed))

    def setAngles(self, names, angles, speed):
        self.calls.append(("setAngles", list(names), [round(a, 4) for a in angles], speed))

    def robotIsWakeUp(self):
        return True

    # Present so that a call would be recorded instead of failing silently.
    def moveTo(self, x, y, theta):
        self.calls.append(("moveTo", x, y, theta))

    def move(self, x, y, theta):
        self.calls.append(("move", x, y, theta))

    def moveToward(self, x, y, theta):
        self.calls.append(("moveToward", x, y, theta))


class AnimationPlayer(Recorder):
    def run(self, name):
        self.calls.append(("run", name))


class BasicAwareness(Recorder):
    def pauseAwareness(self):
        self.calls.append(("pauseAwareness",))

    def resumeAwareness(self):
        self.calls.append(("resumeAwareness",))


class System(Recorder):
    def robotName(self):
        return "pepper-test"

    def systemVersion(self):
        return "2.5.10.7"


class BehaviorManager(Recorder):
    def getInstalledBehaviors(self):
        return ["animations/Stand/Gestures/Hey_1", "animations/Stand/Gestures/Hey_3",
                "animations/Stand/Emotions/Positive/Happy_1"]


class VideoDevice(Recorder):
    def subscribeCamera(self, name, camera, resolution, colorspace, fps):
        self.calls.append(("subscribeCamera", camera, resolution, colorspace, fps))
        return name + "_0"

    # "r" = Raw: how NAOqi sends the image bytes (a plain Python bytes would travel as a string).
    @qi.bind("(iiiiiir)", ["s"])
    def getImageRemote(self, handle):
        rgb = np.zeros((2, 3, 3), dtype=np.uint8)
        rgb[..., 0] = 200  # red
        return [3, 2, 3, 11, 1, 0, rgb.tobytes()]

    def unsubscribe(self, handle):
        self.calls.append(("unsubscribe", handle))
        return True


@pytest.fixture
def pepper():
    calls = []
    server = qi.Session()
    server.listenStandalone("tcp://127.0.0.1:0")
    for name, cls in (("ALMotion", Motion), ("ALAnimationPlayer", AnimationPlayer),
                      ("ALBasicAwareness", BasicAwareness), ("ALSystem", System),
                      ("ALBehaviorManager", BehaviorManager), ("ALVideoDevice", VideoDevice)):
        server.registerService(name, cls(calls))
    port = int(server.endpoints()[0].rsplit(":", 1)[1])
    yield "127.0.0.1", port, calls
    server.close()


def test_naoqi_url_validation():
    assert naoqi_url("192.168.0.50") == "tcp://192.168.0.50:9559"
    for bad in ("", "1.2.3.4/x", "a b"):
        with pytest.raises(ValueError):
            naoqi_url(bad)


def test_head_and_animation_calls(pepper):
    ip, port, calls = pepper
    robot = NaoqiRobot(ip, port)
    assert robot.head_angles() == pytest.approx((0.1, -0.2))
    robot.set_head(0.3, -0.1, 0.15)
    robot.play_animation("animations/Stand/Gestures/Hey_1")
    robot.close()
    assert ("angleInterpolationWithSpeed", ["HeadYaw", "HeadPitch"], [0.3, -0.1], 0.15) in calls
    assert ("run", "animations/Stand/Gestures/Hey_1") in calls
    assert calls[0] == ("pauseAwareness",) and calls[-1] == ("resumeAwareness",)


def test_awareness_is_left_alone_when_not_asked(pepper):
    ip, port, calls = pepper
    NaoqiRobot(ip, port, pause_awareness=False).close()
    assert not [c for c in calls if "Awareness" in c[0]]


@pytest.mark.parametrize("yaw, pitch, speed", [(2.5, 0.0, 0.1), (0.0, 0.9, 0.1), (0.0, 0.0, 1.0)])
def test_out_of_range_head_commands_are_never_sent(pepper, yaw, pitch, speed):
    ip, port, calls = pepper
    robot = NaoqiRobot(ip, port)
    with pytest.raises(ValueError):
        robot.set_head(yaw, pitch, speed)
    robot.close()
    assert not [c for c in calls if c[0] in ("setAngles", "angleInterpolationWithSpeed")]


def test_executor_on_pepper_never_moves_the_base(pepper):
    ip, port, calls = pepper
    robot, results = NaoqiRobot(ip, port), []
    executor = ActionExecutor(robot, LookConfig(), on_result=results.append)
    for command in (Command("look", "person_appeared", 0, 0.0, Target(0.8, 0.3)),
                    Command("look_front", "person_left", 1, 0.0),
                    Command("wave", "wave", 2, 0.0)):
        assert executor.submit(command)
        while executor.busy:
            time.sleep(0.01)
    executor.shutdown()
    robot.close()
    assert [r.ok for r in results] == [True, True, True]
    assert not [c for c in calls if c[0] in LOCOMOTION]


@pytest.mark.parametrize("name", ["animations/Stand/Waiting/Walk_1", "animations/Stand/Gestures/Hey",
                                  "../Hey_1", "animations/Stand/Gestures/Hey_1; rm"])
def test_only_hey_animations_can_be_played(pepper, name):
    ip, port, calls = pepper
    robot = NaoqiRobot(ip, port, pause_awareness=False)
    with pytest.raises(ValueError, match="Hey_"):
        robot.play_animation(name)
    robot.close()
    assert not [c for c in calls if c[0] == "run"]


def test_pepper_info(pepper):
    ip, port, _ = pepper
    info = read_pepper_info(ip, port)
    assert (info.robot_name, info.naoqi_version, info.awake) == ("pepper-test", "2.5.10.7", True)
    assert info.wave_animations == ("animations/Stand/Gestures/Hey_1",
                                    "animations/Stand/Gestures/Hey_3")


def test_pepper_camera_returns_bgr(pepper):
    ip, port, calls = pepper
    source = PepperCameraSource(ip, port, "VGA", 10)
    frame = source.read()
    source.close()
    assert frame.shape == (2, 3, 3)
    assert tuple(frame[0, 0]) == (0, 0, 200)  # BGR
    assert ("subscribeCamera", 0, 2, 11, 10) in calls
    assert any(c[0] == "unsubscribe" for c in calls)


def test_decode_rejects_bad_images():
    with pytest.raises(SourceError):
        decode_image([3, 2, 3, 11, 0, 0, b"short"])
    with pytest.raises(SourceError):
        decode_image([3, 2, 1, 0, 0, 0, b"\0" * 6])
    with pytest.raises(SourceError):
        decode_image(None)
    with pytest.raises(SourceError, match="型が想定外"):
        decode_image([1, 1, 3, 11, 0, 0, "abc"])
