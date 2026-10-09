"""Rules map a trigger (event kind) to one Pepper action, with a per-rule cooldown.

Only LOOK and WAVE exist. Locomotion (move / turn) is deliberately not an action,
in dry-run or on the robot (2026-10-09 decision).
"""
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml

from .events import GESTURE_TRIGGERS, PERSON_TRIGGERS, SEEN_PREFIX, Event
from .observations import Target

ACTIONS = ("look", "look_front", "wave")
TRIGGERS = PERSON_TRIGGERS + GESTURE_TRIGGERS
CONTINUOUS_TRIGGERS = ("person_visible",)  # re-sent every frame, so never queued


@dataclass(frozen=True)
class Rule:
    trigger: str
    action: str
    cooldown_s: float = 5.0
    enabled: bool = True
    note: str = ""

    def __post_init__(self):
        if self.action not in ACTIONS:
            raise ValueError(f"操作 {self.action!r} は使えません（使えるのは {', '.join(ACTIONS)}。"
                             "回る・進むは使わない方針）")
        if self.trigger not in TRIGGERS and not (
                self.trigger.startswith(SEEN_PREFIX) and len(self.trigger) > len(SEEN_PREFIX)):
            raise ValueError(f"きっかけ {self.trigger!r} は使えません")
        if not 0.0 <= float(self.cooldown_s) <= 3600.0:
            raise ValueError("待ち時間は 0〜3600 秒")


@dataclass(frozen=True)
class Command:
    action: str
    trigger: str
    rule_index: int
    t: float
    target: Target | None = None


def rules_from_records(records: list[dict]) -> tuple[Rule, ...]:
    rules = []
    for i, record in enumerate(records, start=1):
        try:
            rules.append(Rule(trigger=str(record["trigger"]), action=str(record["action"]),
                              cooldown_s=float(record.get("cooldown_s", 5.0)),
                              enabled=bool(record.get("enabled", True)),
                              note=str(record.get("note") or "")))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{i} 行目: {exc}") from exc
    return tuple(rules)


def load_rules(path: Path) -> tuple[Rule, ...]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    records = data.get("rules")
    if not isinstance(records, list):
        raise ValueError(f"{path}: 'rules' の一覧がありません")
    return rules_from_records(records)


def save_rules(path: Path, rules: tuple[Rule, ...]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump({"rules": [asdict(r) for r in rules]}, allow_unicode=True, sort_keys=False)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=path.stem, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(text)
    os.replace(temporary, path)


class RuleEngine:
    """At most one command per call. Earlier rules win; edge events wait up to event_ttl_s.

    LOOK ignores events older than look_max_age_s: the head may have moved since the frame.
    `previous` carries cooldowns of unchanged rules over, so saving rules does not re-fire them.
    """

    def __init__(self, rules: tuple[Rule, ...], *, min_interval_s: float = 1.0,
                 event_ttl_s: float = 2.0, look_max_age_s: float = 0.5,
                 previous: "RuleEngine | None" = None):
        self.rules = tuple(rules)
        self.min_interval_s = min_interval_s
        self.event_ttl_s = event_ttl_s
        self.look_max_age_s = look_max_age_s
        self._last_fired: dict[int, float] = {}
        self._last_command: float | None = None
        self._pending: list[Event] = []
        if previous is not None:
            fired = {rule: previous._last_fired[i] for i, rule in enumerate(previous.rules)
                     if i in previous._last_fired}
            self._last_fired = {i: fired[rule] for i, rule in enumerate(self.rules) if rule in fired}
            self._last_command = previous._last_command
            self._pending = list(previous._pending)

    def decide(self, events: list[Event], t: float, robot_busy: bool) -> Command | None:
        continuous = [e for e in events if e.kind in CONTINUOUS_TRIGGERS]
        self._pending = [e for e in self._pending if t - e.t <= self.event_ttl_s]
        self._pending += [e for e in events if e.kind not in CONTINUOUS_TRIGGERS]
        if robot_busy:
            return None
        if self._last_command is not None and t - self._last_command < self.min_interval_s:
            return None
        candidates = self._pending + continuous
        for index, rule in enumerate(self.rules):
            if not rule.enabled or t - self._last_fired.get(index, float("-inf")) < rule.cooldown_s:
                continue
            event = next((e for e in reversed(candidates) if e.kind == rule.trigger), None)
            if event is None:
                continue
            if rule.action == "look" and (event.target is None or t - event.t > self.look_max_age_s):
                continue
            self._last_fired[index] = t
            self._last_command = t
            if event in self._pending:
                self._pending.remove(event)
            return Command(rule.action, rule.trigger, index, t, event.target)
        return None
