"""Gestures from pose keypoints over time: hand_raised, both_hands_up, wave.

Image y grows downward, so "above" means a smaller y. Distances are divided by the
shoulder width so the thresholds do not depend on how far the person is.
"""
from collections import deque
from dataclasses import dataclass

from .observations import (L_ELBOW, L_SHOULDER, L_WRIST, NOSE, R_ELBOW, R_SHOULDER, R_WRIST,
                           Person)

GESTURES = ("hand_raised", "both_hands_up", "wave")
SIDES = ((L_SHOULDER, L_ELBOW, L_WRIST), (R_SHOULDER, R_ELBOW, R_WRIST))


@dataclass(frozen=True)
class GestureConfig:
    keypoint_min_conf: float = 0.5
    hold_s: float = 0.8            # hand_raised / both_hands_up must last this long
    still_tolerance: float = 0.25  # a raised hand moving more than this (x shoulder width) is not "raised"
    wave_window_s: float = 1.5
    wave_min_reversals: int = 2    # right-left-right = 2 reversals
    wave_min_swing: float = 0.15   # x shoulder width
    wave_end_hold_s: float = 0.3   # one frame with a lost wrist does not end the wave
    forget_s: float = 2.0


@dataclass(frozen=True)
class PoseSample:
    t: float
    raised: tuple[bool, bool]          # wrist above shoulder (left, right)
    above_head: tuple[bool, bool]      # wrist above nose (left, right)
    wrist_x: tuple[float | None, float | None]  # wrist x relative to elbow, only while wrist is above elbow


def count_reversals(values: list[float], threshold: float) -> int:
    """Direction changes whose swing is at least `threshold` (hysteresis against jitter)."""
    if len(values) < 2:
        return 0
    direction, extreme, reversals = 0, values[0], 0
    for value in values[1:]:
        if direction == 0:
            if abs(value - extreme) >= threshold:
                direction = 1 if value > extreme else -1
                extreme = value
        elif (value - extreme) * direction > 0:
            extreme = value
        elif abs(value - extreme) >= threshold:
            reversals += 1
            direction, extreme = -direction, value
    return reversals


def _shoulder_width(person: Person, min_conf: float) -> float:
    kp, conf = person.keypoints, person.keypoint_conf
    if conf[L_SHOULDER] >= min_conf and conf[R_SHOULDER] >= min_conf:
        width = abs(float(kp[L_SHOULDER][0] - kp[R_SHOULDER][0]))
        if width > 1.0:
            return width
    x1, _, x2, _ = person.box
    return max(1.0, 0.35 * (x2 - x1))


def pose_sample(person: Person, t: float, min_conf: float) -> PoseSample:
    kp, conf = person.keypoints, person.keypoint_conf
    width = _shoulder_width(person, min_conf)
    raised, above_head, wrist_x = [], [], []
    for shoulder, elbow, wrist in SIDES:
        wrist_seen = conf[wrist] >= min_conf
        shoulder_seen = conf[shoulder] >= min_conf
        is_raised = wrist_seen and shoulder_seen and kp[wrist][1] < kp[shoulder][1] - 0.1 * width
        head_y = kp[NOSE][1] if conf[NOSE] >= min_conf else kp[shoulder][1] - 0.6 * width
        raised.append(bool(is_raised))
        above_head.append(bool(wrist_seen and shoulder_seen and kp[wrist][1] < head_y))
        over_elbow = wrist_seen and conf[elbow] >= min_conf and kp[wrist][1] < kp[elbow][1]
        wrist_x.append(float(kp[wrist][0] - kp[elbow][0]) / width if over_elbow else None)
    return PoseSample(t, tuple(raised), tuple(above_head), tuple(wrist_x))


class GestureTracker:
    """Keeps a short pose history per track ID and reports the gestures active now."""

    def __init__(self, config: GestureConfig | None = None):
        self.config = config or GestureConfig()
        self._history: dict[int, deque[PoseSample]] = {}
        self._last_seen: dict[int, float] = {}

    def update(self, persons: tuple[Person, ...], t: float) -> dict[int, frozenset[str]]:
        c = self.config
        keep_s = max(c.wave_window_s, c.hold_s) + 0.5
        active = {}
        for person in persons:
            if person.track_id is None:
                continue
            history = self._history.setdefault(person.track_id, deque())
            history.append(pose_sample(person, t, c.keypoint_min_conf))
            while history and t - history[0].t > keep_s:
                history.popleft()
            self._last_seen[person.track_id] = t
            active[person.track_id] = self._gestures(list(history), t)
        for track_id in [k for k, seen in self._last_seen.items() if t - seen > c.forget_s]:
            self._history.pop(track_id, None)
            self._last_seen.pop(track_id, None)
        return active

    def _gestures(self, history: list[PoseSample], t: float) -> frozenset[str]:
        found = set()
        if self._is_waving(history, t):
            found.add("wave")
        elif self._held(history, t, lambda s: all(s.above_head)):
            found.add("both_hands_up")
        elif any(self._held(history, t, lambda s, i=i: s.raised[i]) and self._still(history, t, i)
                 for i in range(2)):
            found.add("hand_raised")
        return frozenset(found)

    def _held(self, history: list[PoseSample], t: float, condition) -> bool:
        """True when the latest unbroken run of `condition` started at least hold_s ago."""
        start = None
        for sample in reversed(history):
            if not condition(sample):
                break
            start = sample.t
        return start is not None and t - start >= self.config.hold_s

    def _still(self, history: list[PoseSample], t: float, side: int) -> bool:
        xs = [s.wrist_x[side] for s in history
              if t - s.t <= self.config.hold_s and s.wrist_x[side] is not None]
        return not xs or max(xs) - min(xs) < self.config.still_tolerance

    def _is_waving(self, history: list[PoseSample], t: float) -> bool:
        """Swings in the window, and the hand was up within wave_end_hold_s (lowering it ends the wave)."""
        c = self.config
        for side in range(2):
            last_up = next((s.t for s in reversed(history) if s.wrist_x[side] is not None), None)
            if last_up is None or t - last_up > c.wave_end_hold_s:
                continue
            xs = [s.wrist_x[side] for s in history
                  if t - s.t <= c.wave_window_s and s.wrist_x[side] is not None]
            if count_reversals(xs, c.wave_min_swing) >= c.wave_min_reversals:
                return True
        return False
