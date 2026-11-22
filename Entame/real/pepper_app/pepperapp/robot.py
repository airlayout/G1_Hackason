"""Head angles for LOOK, the robot interface, the dry-run robot and the one-at-a-time executor."""
import logging
import math
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Protocol

from .observations import Target
from .rules import Command

log = logging.getLogger(__name__)

# Pepper head joint limits (rad). We stay well inside them.
HEAD_YAW_LIMIT = 2.0857
HEAD_PITCH_RANGE = (-0.7068, 0.6371)
# Only the hand-wave greetings. Other animations can move the whole body, which is out of scope.
WAVE_ANIMATION = re.compile(r"animations/Stand/Gestures/Hey_\d+")


def check_wave_animation(name: str) -> str:
    if not WAVE_ANIMATION.fullmatch(name):
        raise ValueError(f"手を振るアニメーションは animations/Stand/Gestures/Hey_<番号> だけ: {name!r}")
    return name


@dataclass(frozen=True)
class LookConfig:
    hfov_rad: float = math.radians(57.2)  # Pepper top camera
    vfov_rad: float = math.radians(44.3)
    yaw_limit: float = 1.0
    pitch_min: float = -0.4
    pitch_max: float = 0.3
    front_pitch: float = -0.15
    deadband_rad: float = 0.05
    speed: float = 0.15  # fraction of max joint speed
    wave_animation: str = "animations/Stand/Gestures/Hey_1"

    def __post_init__(self):
        if not 0.0 < self.yaw_limit <= HEAD_YAW_LIMIT:
            raise ValueError("yaw_limit must be within the head joint limit")
        if not HEAD_PITCH_RANGE[0] <= self.pitch_min < self.pitch_max <= HEAD_PITCH_RANGE[1]:
            raise ValueError("pitch range must be within the head joint limit")
        if not 0.01 <= self.speed <= 0.3:
            raise ValueError("speed must be 0.01..0.3")
        check_wave_animation(self.wave_animation)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def look_angles(target: Target, current: tuple[float, float],
                config: LookConfig) -> tuple[float, float] | None:
    """Absolute head (yaw, pitch) that centers `target`; None inside the deadband.

    The camera is on the head, so the offset in the image is added to the current angles.
    Yaw is positive to Pepper's left (image left); pitch is positive looking down.
    """
    d_yaw = -(target.x - 0.5) * config.hfov_rad
    d_pitch = (target.y - 0.5) * config.vfov_rad
    if abs(d_yaw) < config.deadband_rad and abs(d_pitch) < config.deadband_rad:
        return None
    yaw = _clamp(current[0] + d_yaw, -config.yaw_limit, config.yaw_limit)
    pitch = _clamp(current[1] + d_pitch, config.pitch_min, config.pitch_max)
    return yaw, pitch


class Robot(Protocol):
    def head_angles(self) -> tuple[float, float]: ...
    def set_head(self, yaw: float, pitch: float, speed: float) -> None:
        """Blocks until the head arrives, so the next LOOK reads settled angles."""
    def play_animation(self, name: str) -> None: ...
    def close(self) -> None: ...


class DryRunRobot:
    """Sends nothing. Keeps the head angles it was told so LOOK can be followed on screen."""

    name = "試運転"

    def __init__(self, wave_duration_s: float = 2.0):
        self.wave_duration_s = wave_duration_s
        self._head = (0.0, 0.0)

    def head_angles(self) -> tuple[float, float]:
        return self._head

    def set_head(self, yaw: float, pitch: float, speed: float) -> None:
        self._head = (yaw, pitch)

    def play_animation(self, name: str) -> None:
        time.sleep(self.wave_duration_s)

    def close(self) -> None:
        pass


@dataclass(frozen=True)
class ActionResult:
    command: Command
    ok: bool
    detail: str
    finished_at: float


class ActionExecutor:
    """Runs one command at a time on a worker thread. Busy commands are refused, not queued."""

    def __init__(self, robot: Robot, config: LookConfig, on_result=None):
        self.robot = robot
        self.config = config
        self._on_result = on_result or (lambda result: None)
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pepper-action")
        self._busy = threading.Event()
        self._submit_lock = threading.Lock()  # the loop and the UI thread both submit

    @property
    def busy(self) -> bool:
        return self._busy.is_set()

    def submit(self, command: Command) -> bool:
        with self._submit_lock:
            if self._busy.is_set():
                return False
            self._busy.set()
            try:
                self._pool.submit(self._run, command)
            except RuntimeError:  # already shut down
                self._busy.clear()
                return False
        return True

    def _run(self, command: Command) -> None:
        try:
            detail = self._execute(command)
            result = ActionResult(command, True, detail, time.monotonic())
        except Exception as exc:  # reported to the UI log, never swallowed
            log.exception("action %s failed", command.action)
            result = ActionResult(command, False, f"{type(exc).__name__}: {exc}", time.monotonic())
        finally:
            self._busy.clear()
        self._on_result(result)

    def _execute(self, command: Command) -> str:
        c = self.config
        if command.action == "wave":
            self.robot.play_animation(c.wave_animation)
            return c.wave_animation
        if command.action == "look_front":
            self.robot.set_head(0.0, c.front_pitch, c.speed)
            return f"yaw=0.00 pitch={c.front_pitch:+.2f}"
        if command.action == "look":
            if command.target is None:
                raise ValueError("look には対象が要る")
            angles = look_angles(command.target, self.robot.head_angles(), c)
            if angles is None:
                return "既に正面に捉えている"
            self.robot.set_head(angles[0], angles[1], c.speed)
            return f"yaw={angles[0]:+.2f} pitch={angles[1]:+.2f}"
        raise ValueError(f"未対応の操作: {command.action}")

    def shutdown(self) -> None:
        self._pool.shutdown(wait=True)
