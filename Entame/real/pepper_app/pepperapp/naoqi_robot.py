"""Pepper over the NAOqi API (qi, tcp port 9559): head angles and gesture animations only.

Services used: ALMotion (getAngles / angleInterpolationWithSpeed on HeadYaw, HeadPitch),
ALAnimationPlayer (run, Gestures/Hey_* only),
ALBasicAwareness (pause / resume so Pepper does not turn its head on its own),
ALSystem / ALBehaviorManager (read-only diagnostics). Nothing here moves the base.
"""
import logging
from dataclasses import dataclass

from .robot import HEAD_PITCH_RANGE, HEAD_YAW_LIMIT, check_wave_animation

log = logging.getLogger(__name__)
HEAD_JOINTS = ["HeadYaw", "HeadPitch"]


def _default_session():
    import qi  # only needed when talking to a robot
    return qi.Session()


def naoqi_url(ip: str, port: int = 9559) -> str:
    ip = str(ip).strip()
    if not ip or any(ch in ip for ch in "/ \t"):
        raise ValueError("Pepper の IP を入れてください")
    if not 1 <= int(port) <= 65535:
        raise ValueError("port must be 1..65535")
    return f"tcp://{ip}:{int(port)}"


@dataclass(frozen=True)
class PepperInfo:
    robot_name: str
    naoqi_version: str
    awake: bool
    wave_animations: tuple[str, ...]


class NaoqiRobot:
    name = "Pepper"

    def __init__(self, ip: str, port: int = 9559, *, pause_awareness: bool = True,
                 session_factory=_default_session):
        self.session = session_factory()
        self.session.connect(naoqi_url(ip, port))
        self._awareness = None
        try:
            self.motion = self.session.service("ALMotion")
            self.animation = self.session.service("ALAnimationPlayer")
            if pause_awareness:
                awareness = self.session.service("ALBasicAwareness")
                awareness.pauseAwareness()
                self._awareness = awareness
                log.info("ALBasicAwareness paused")
        except Exception:
            self.session.close()
            raise

    def head_angles(self) -> tuple[float, float]:
        yaw, pitch = self.motion.getAngles(HEAD_JOINTS, True)
        return float(yaw), float(pitch)

    def set_head(self, yaw: float, pitch: float, speed: float) -> None:
        # Second guard behind LookConfig: never send angles outside the joint range.
        if abs(yaw) > HEAD_YAW_LIMIT or not HEAD_PITCH_RANGE[0] <= pitch <= HEAD_PITCH_RANGE[1]:
            raise ValueError(f"head angles out of range: yaw={yaw}, pitch={pitch}")
        if not 0.0 < speed <= 0.3:
            raise ValueError(f"speed out of range: {speed}")
        # Blocking: returns when the head has arrived.
        self.motion.angleInterpolationWithSpeed(HEAD_JOINTS, [float(yaw), float(pitch)], float(speed))

    def play_animation(self, name: str) -> None:
        self.animation.run(check_wave_animation(name))

    def close(self) -> None:
        try:
            if self._awareness is not None:
                self._awareness.resumeAwareness()
                log.info("ALBasicAwareness resumed")
        finally:
            self.session.close()


def read_pepper_info(ip: str, port: int = 9559, session_factory=_default_session) -> PepperInfo:
    """Read-only connection check for the 接続確認 tab."""
    session = session_factory()
    session.connect(naoqi_url(ip, port))
    try:
        system = session.service("ALSystem")
        behaviors = session.service("ALBehaviorManager").getInstalledBehaviors()
        waves = tuple(sorted(b for b in behaviors if "Gestures/Hey" in b))
        return PepperInfo(robot_name=str(system.robotName()),
                          naoqi_version=str(system.systemVersion()),
                          awake=bool(session.service("ALMotion").robotIsWakeUp()),
                          wave_animations=waves)
    finally:
        session.close()
