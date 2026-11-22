"""Background loop: source -> detector -> gestures -> events -> rules -> executor, plus a UI snapshot."""
import logging
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from datetime import datetime

from .annotate import annotate, to_jpeg
from .events import EventConfig, EventDetector
from .gestures import GestureConfig, GestureTracker
from .robot import ActionExecutor, ActionResult, LookConfig, Robot
from .rules import Command, Rule, RuleEngine

log = logging.getLogger(__name__)
LOG_SIZE = 500
HEAD_POLL_S = 0.5
RETRY_S = 1.0


@dataclass(frozen=True)
class LogEntry:
    time: str
    kind: str   # event / command / result / error
    text: str


@dataclass(frozen=True)
class Snapshot:
    running: bool = False
    reacting: bool = False
    source_name: str = ""
    robot_name: str = ""
    frame_jpeg: bytes | None = None
    fps: float = 0.0
    inference_ms: float = 0.0
    persons: int = 0
    objects: dict[str, int] = field(default_factory=dict)
    gestures: tuple[str, ...] = ()
    head: tuple[float, float] = (0.0, 0.0)
    robot_busy: bool = False
    error: str | None = None
    log: tuple[LogEntry, ...] = ()


class RecognitionLoop:
    def __init__(self, source, detector, robot: Robot, rules: tuple[Rule, ...], *,
                 look_config: LookConfig, reacting: bool = False, min_interval_s: float = 1.0,
                 gesture_config: GestureConfig | None = None, event_config: EventConfig | None = None):
        self._source, self._detector, self._robot = source, detector, robot
        self._gestures = GestureTracker(gesture_config)
        self._event_config = event_config
        self._events = EventDetector(event_config)
        self._reset_events = False
        self._min_interval_s = min_interval_s
        self._engine = RuleEngine(rules, min_interval_s=min_interval_s)
        self._executor = ActionExecutor(robot, look_config, on_result=self._on_result)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._log: deque[LogEntry] = deque(maxlen=LOG_SIZE)
        self._snapshot = Snapshot(source_name=source.name, robot_name=robot.name, reacting=reacting)
        self._reacting = reacting
        self._head_polled = 0.0

    # --- control (called from the Streamlit thread) ---
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="recognition", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
            if self._thread.is_alive():
                log.warning("recognition thread did not stop within 10 s (source read blocked?)")
        self._executor.shutdown()
        for close in (self._source.close, self._robot.close):
            try:
                close()
            except Exception:
                log.exception("close failed")

    def set_rules(self, rules: tuple[Rule, ...]) -> None:
        with self._lock:
            self._engine = RuleEngine(rules, min_interval_s=self._min_interval_s,
                                      previous=self._engine)
        self._append("command", f"割り当てを反映（{len(rules)} 件）")

    def set_reacting(self, reacting: bool) -> None:
        """Turning reactions on counts what is in view now as newly seen.

        Events fire when something appears; without this, a chair already in view when 反応を始める
        is pressed would never trigger its rule.
        """
        with self._lock:
            self._reacting = reacting
            self._reset_events = self._reset_events or reacting
        self._append("command", "反応を開始" if reacting else "反応を停止")

    def run_action(self, action: str) -> bool:
        """Manual test from the UI (look_front / wave). Refused while another action runs."""
        accepted = self._executor.submit(Command(action, "manual", -1, time.monotonic()))
        self._append("command", f"手動: {action}" + ("" if accepted else "（動作中のため見送り）"))
        return accepted

    def snapshot(self) -> Snapshot:
        with self._lock:
            return replace(self._snapshot, reacting=self._reacting,
                           robot_busy=self._executor.busy, log=tuple(self._log))

    # --- loop ---
    def _run(self) -> None:
        frames: deque[float] = deque(maxlen=30)
        last_error = None
        while not self._stop.is_set():
            try:
                self._step(frames)
                last_error = None
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                if message != last_error:
                    log.exception("recognition step failed")
                    self._append("error", message)
                last_error = message
                with self._lock:
                    self._snapshot = replace(self._snapshot, error=message)
                self._stop.wait(RETRY_S)
        with self._lock:
            self._snapshot = replace(self._snapshot, running=False)

    def _step(self, frames: deque[float]) -> None:
        frame = self._source.read()
        t = time.monotonic()
        detections = self._detector(frame)
        gestures = self._gestures.update(detections.persons, t)
        with self._lock:
            reset, self._reset_events = self._reset_events, False
        if reset:
            self._events = EventDetector(self._event_config)
        events = self._events.update(detections, gestures, t)
        for event in events:
            if event.kind != "person_visible":
                who = f" #{event.track_id}" if event.track_id is not None else ""
                self._append("event", event.kind + who)
        with self._lock:
            engine, reacting = self._engine, self._reacting
        if reacting and not self._stop.is_set():
            command = engine.decide(events, t, self._executor.busy)
            if command is not None:
                accepted = not self._stop.is_set() and self._executor.submit(command)
                self._append("command", f"{command.trigger} → {command.action}"
                             + ("" if accepted else "（見送り）"))
        frames.append(t)
        self._publish(frame, detections, gestures, frames, t)

    def _publish(self, frame, detections, gestures, frames, t) -> None:
        span = frames[-1] - frames[0] if len(frames) > 1 else 0.0
        fps = (len(frames) - 1) / span if span > 0 else 0.0
        head = self._snapshot.head
        if t - self._head_polled >= HEAD_POLL_S and not self._executor.busy:
            self._head_polled = t
            try:
                head = self._robot.head_angles()
            except Exception as exc:  # keep the last value; a blip must not drop the frame
                log.warning("head_angles failed: %s", exc)
        jpeg = to_jpeg(annotate(frame, detections, gestures))
        labels = tuple(f"#{tid}: {', '.join(sorted(g))}" for tid, g in sorted(gestures.items()) if g)
        with self._lock:
            self._snapshot = Snapshot(
                running=True, source_name=self._source.name, robot_name=self._robot.name,
                frame_jpeg=jpeg, fps=fps, inference_ms=detections.inference_ms,
                persons=len(detections.persons),
                objects=dict(Counter(o.label for o in detections.objects)),
                gestures=labels, head=head, error=None)

    def _on_result(self, result: ActionResult) -> None:
        status = "OK" if result.ok else "失敗"
        self._append("result" if result.ok else "error",
                     f"{result.command.action} {status}: {result.detail}")

    def _append(self, kind: str, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._lock:
            self._log.append(LogEntry(stamp, kind, text))
