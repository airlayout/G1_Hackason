"""Turn per-frame detections and gestures into events (rising edges with hold times).

Trigger keys (used by rules):
  person_appeared / person_left / person_near  presence of anyone, debounced
  person_visible                               every frame while someone is present (for following)
  hand_raised / both_hands_up / wave           per person, when the gesture starts
  seen:<label>                                 an object label (e.g. seen:chair) becomes present
"""
from dataclasses import dataclass

from .observations import Detections, Person, Target, box_center, person_target

PERSON_TRIGGERS = ("person_appeared", "person_visible", "person_left", "person_near")
GESTURE_TRIGGERS = ("hand_raised", "both_hands_up", "wave")
SEEN_PREFIX = "seen:"


@dataclass(frozen=True)
class Event:
    kind: str
    t: float
    target: Target | None = None
    track_id: int | None = None


@dataclass(frozen=True)
class EventConfig:
    appear_hold_s: float = 0.5
    leave_hold_s: float = 1.5
    seen_hold_s: float = 0.5
    unseen_hold_s: float = 2.0
    near_height_ratio: float = 0.6


class Debounced:
    """Presence that must stay changed for a hold time before it flips."""

    def __init__(self, rise_hold_s: float, fall_hold_s: float):
        self.rise_hold_s, self.fall_hold_s = rise_hold_s, fall_hold_s
        self.state = False
        self._changed_since: float | None = None

    def update(self, present: bool, t: float) -> str | None:
        if present == self.state:
            self._changed_since = None
            return None
        if self._changed_since is None:
            self._changed_since = t
        hold = self.rise_hold_s if present else self.fall_hold_s
        if t - self._changed_since < hold:
            return None
        self.state, self._changed_since = present, None
        return "rise" if present else "fall"


class EventDetector:
    def __init__(self, config: EventConfig | None = None):
        self.config = config or EventConfig()
        self._people = Debounced(self.config.appear_hold_s, self.config.leave_hold_s)
        self._labels: dict[str, Debounced] = {}
        self._near = False
        self._gestures: dict[int, frozenset[str]] = {}

    def update(self, detections: Detections, gestures: dict[int, frozenset[str]],
               t: float) -> list[Event]:
        events = self._presence_events(detections, t)
        events += self._gesture_events(detections, gestures, t)
        events += self._seen_events(detections, t)
        return events

    def _presence_events(self, d: Detections, t: float) -> list[Event]:
        main = max(d.persons, key=lambda p: p.area, default=None)
        target = person_target(main, d.width, d.height) if main else None
        edge = self._people.update(main is not None, t)
        events = []
        if edge == "rise":
            events.append(Event("person_appeared", t, target, main.track_id))
        elif edge == "fall":
            events.append(Event("person_left", t))
            self._near = False
        if self._people.state and main is not None:
            events.append(Event("person_visible", t, target, main.track_id))
            events += self._near_event(main, d, target, t)
        return events

    def _near_event(self, person: Person, d: Detections, target: Target, t: float) -> list[Event]:
        ratio = (person.box[3] - person.box[1]) / d.height
        threshold = self.config.near_height_ratio
        if not self._near and ratio >= threshold:
            self._near = True
            return [Event("person_near", t, target, person.track_id)]
        if self._near and ratio < threshold - 0.1:
            self._near = False
        return []

    def _gesture_events(self, d: Detections, gestures: dict[int, frozenset[str]],
                        t: float) -> list[Event]:
        by_id = {p.track_id: p for p in d.persons if p.track_id is not None}
        events = []
        for track_id, active in gestures.items():
            started = active - self._gestures.get(track_id, frozenset())
            person = by_id.get(track_id)
            target = person_target(person, d.width, d.height) if person else None
            events += [Event(kind, t, target, track_id) for kind in sorted(started)]
        self._gestures = dict(gestures)
        return events

    def _seen_events(self, d: Detections, t: float) -> list[Event]:
        c = self.config
        labels = {o.label for o in d.objects} | set(self._labels)
        events = []
        for label in sorted(labels):
            found = [o for o in d.objects if o.label == label]
            presence = self._labels.setdefault(label, Debounced(c.seen_hold_s, c.unseen_hold_s))
            if presence.update(bool(found), t) == "rise":
                main = max(found, key=lambda o: o.area)
                events.append(Event(SEEN_PREFIX + label, t, box_center(main.box, d.width, d.height)))
        return events
