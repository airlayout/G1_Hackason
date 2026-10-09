"""Builds the loop from UI settings and owns it. One instance per Streamlit server (st.cache_resource)."""
import atexit
import logging
import threading
from dataclasses import dataclass, field
from pathlib import Path

from .detector import DetectorConfig, YoloDetector
from .naoqi_robot import NaoqiRobot
from .pipeline import RecognitionLoop, Snapshot
from .robot import DryRunRobot, LookConfig
from .rules import Rule, load_rules, save_rules
from .sources import FileSource, PepperCameraSource, WebcamSource

log = logging.getLogger(__name__)
APP_DIR = Path(__file__).resolve().parent.parent
DEFAULT_RULES = APP_DIR / "rules" / "default.yaml"
USER_RULES = APP_DIR / "rules" / "rules.yaml"
SOURCE_KINDS = ("file", "webcam", "pepper")
ROBOT_MODES = ("dry_run", "pepper")


@dataclass(frozen=True)
class Settings:
    source_kind: str = "file"
    file_path: str = ""
    webcam_index: int = 0
    pepper_ip: str = ""
    pepper_port: int = 9559
    camera_resolution: str = "VGA"
    camera_fps: int = 10
    robot_mode: str = "dry_run"
    pause_awareness: bool = True
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    look: LookConfig = field(default_factory=LookConfig)

    def __post_init__(self):
        if self.source_kind not in SOURCE_KINDS:
            raise ValueError(f"source_kind must be one of {SOURCE_KINDS}")
        if self.robot_mode not in ROBOT_MODES:
            raise ValueError(f"robot_mode must be one of {ROBOT_MODES}")


def build_source(s: Settings):
    if s.source_kind == "file":
        return FileSource(s.file_path)
    if s.source_kind == "webcam":
        return WebcamSource(s.webcam_index)
    return PepperCameraSource(s.pepper_ip, s.pepper_port, s.camera_resolution, s.camera_fps)


def build_robot(s: Settings):
    if s.robot_mode == "dry_run":
        return DryRunRobot()
    return NaoqiRobot(s.pepper_ip, s.pepper_port, pause_awareness=s.pause_awareness)


class AppController:
    def __init__(self, *, rules_path: Path = USER_RULES, default_rules_path: Path = DEFAULT_RULES,
                 source_factory=build_source, detector_factory=YoloDetector,
                 robot_factory=build_robot):
        self.rules_path, self.default_rules_path = Path(rules_path), Path(default_rules_path)
        self._source_factory = source_factory
        self._detector_factory = detector_factory
        self._robot_factory = robot_factory
        # Held for the whole of start / stop: two tabs pressing 開始 must not leave an orphan loop.
        self._lifecycle = threading.RLock()
        self._loop: RecognitionLoop | None = None
        self.settings: Settings | None = None
        atexit.register(self.stop)  # resume Pepper's awareness and unsubscribe the camera on exit

    def rules(self) -> tuple[Rule, ...]:
        path = self.rules_path if self.rules_path.is_file() else self.default_rules_path
        return load_rules(path)

    def save_rules(self, rules: tuple[Rule, ...]) -> None:
        save_rules(self.rules_path, rules)
        with self._lifecycle:
            if self._loop is not None:
                self._loop.set_rules(rules)

    def reset_rules(self) -> tuple[Rule, ...]:
        rules = load_rules(self.default_rules_path)
        self.save_rules(rules)
        return rules

    def start(self, settings: Settings) -> None:
        """Stop any running loop, then build source, detector and robot. Nothing is left open on failure."""
        with self._lifecycle:
            self._start_locked(settings)

    def _start_locked(self, settings: Settings) -> None:
        self.stop()
        opened = []
        try:
            detector = self._detector_factory(settings.detector)
            source = self._source_factory(settings)
            opened.append(source)
            robot = self._robot_factory(settings)
            opened.append(robot)
            loop = RecognitionLoop(source, detector, robot, self.rules(), look_config=settings.look,
                                   reacting=settings.robot_mode == "dry_run")
        except Exception:
            for item in reversed(opened):
                try:
                    item.close()
                except Exception:
                    log.exception("close after failed start")
            raise
        loop.start()
        self._loop, self.settings = loop, settings

    def stop(self) -> None:
        with self._lifecycle:
            loop, self._loop = self._loop, None
            if loop is not None:
                loop.stop()

    @property
    def running(self) -> bool:
        return self._loop is not None

    def snapshot(self) -> Snapshot:
        loop = self._loop
        return loop.snapshot() if loop is not None else Snapshot()

    def set_reacting(self, reacting: bool) -> None:
        loop = self._loop
        if loop is not None:
            loop.set_reacting(reacting)

    def run_action(self, action: str) -> bool:
        loop = self._loop
        return loop.run_action(action) if loop is not None else False
