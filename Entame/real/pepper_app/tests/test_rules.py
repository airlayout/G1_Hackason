import pytest

from pepperapp.controller import DEFAULT_RULES
from pepperapp.events import Event
from pepperapp.observations import Target
from pepperapp.rules import (ACTIONS, Rule, RuleEngine, load_rules, rules_from_records,
                             save_rules)

TARGET = Target(0.7, 0.4)


@pytest.mark.parametrize("action", ["move", "turn", "MOVE", "goto", ""])
def test_locomotion_and_unknown_actions_are_rejected(action):
    with pytest.raises(ValueError, match="回る・進むは使わない方針"):
        Rule("wave", action)


def test_only_look_and_wave_actions_exist():
    assert ACTIONS == ("look", "look_front", "wave")


@pytest.mark.parametrize("trigger", ["dance", "seen:", ""])
def test_unknown_trigger_is_rejected(trigger):
    with pytest.raises(ValueError, match="きっかけ"):
        Rule(trigger, "look")


def test_seen_trigger_and_cooldown_range():
    assert Rule("seen:chair", "look").trigger == "seen:chair"
    with pytest.raises(ValueError, match="待ち時間"):
        Rule("wave", "wave", cooldown_s=-1)


def test_save_leaves_no_temporary_files(tmp_path):
    save_rules(tmp_path / "rules.yaml", (Rule("wave", "wave"),))
    save_rules(tmp_path / "rules.yaml", (Rule("wave", "look"),))
    assert [p.name for p in tmp_path.iterdir()] == ["rules.yaml"]


def test_yaml_round_trip(tmp_path):
    rules = (Rule("wave", "wave", 10.0, True, "振り返す"), Rule("seen:chair", "look", 30.0, False))
    path = tmp_path / "rules.yaml"
    save_rules(path, rules)
    assert load_rules(path) == rules


def test_default_rules_load_and_use_only_allowed_actions():
    rules = load_rules(DEFAULT_RULES)
    assert rules and {r.action for r in rules} <= {"look", "look_front", "wave"}


def test_record_errors_name_the_row():
    with pytest.raises(ValueError, match="2 行目"):
        rules_from_records([{"trigger": "wave", "action": "wave"},
                            {"trigger": "wave", "action": "move"}])
    with pytest.raises(ValueError, match="1 行目"):
        rules_from_records([{"action": "wave"}])


def test_load_rejects_file_without_rules(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("hello: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="rules"):
        load_rules(path)


def test_first_matching_rule_wins():
    engine = RuleEngine((Rule("wave", "wave"), Rule("wave", "look")), min_interval_s=0)
    command = engine.decide([Event("wave", 0.0, TARGET, 1)], 0.0, robot_busy=False)
    assert (command.action, command.rule_index, command.target) == ("wave", 0, TARGET)


def test_cooldown_and_min_interval():
    engine = RuleEngine((Rule("person_visible", "look", cooldown_s=2.0),), min_interval_s=1.0)
    def visible(t):
        return [Event("person_visible", t, TARGET)]
    assert engine.decide(visible(0.0), 0.0, False) is not None
    assert engine.decide(visible(1.5), 1.5, False) is None      # cooldown
    assert engine.decide(visible(2.0), 2.0, False) is not None


def test_look_ignores_stale_targets_but_wave_does_not():
    engine = RuleEngine((Rule("person_appeared", "look"), Rule("person_appeared", "wave")),
                        min_interval_s=0, look_max_age_s=0.5)
    engine.decide([Event("person_appeared", 0.0, TARGET)], 0.0, robot_busy=True)
    assert engine.decide([], 1.0, robot_busy=False).action == "wave"


def test_saving_rules_keeps_cooldowns_of_unchanged_rules():
    wave = Rule("wave", "wave", cooldown_s=10.0)
    old = RuleEngine((wave,), min_interval_s=0)
    assert old.decide([Event("wave", 0.0, TARGET)], 0.0, False) is not None
    new = RuleEngine((Rule("seen:chair", "look"), wave), min_interval_s=0, previous=old)
    assert new.decide([Event("wave", 1.0, TARGET)], 1.0, False) is None   # still cooling down
    assert new.decide([Event("wave", 11.0, TARGET)], 11.0, False).rule_index == 1


def test_edge_event_waits_while_busy_then_expires():
    engine = RuleEngine((Rule("wave", "wave"),), min_interval_s=0, event_ttl_s=2.0)
    assert engine.decide([Event("wave", 0.0, TARGET)], 0.0, robot_busy=True) is None
    assert engine.decide([], 1.0, robot_busy=False).action == "wave"
    engine2 = RuleEngine((Rule("wave", "wave"),), min_interval_s=0, event_ttl_s=2.0)
    engine2.decide([Event("wave", 0.0, TARGET)], 0.0, robot_busy=True)
    assert engine2.decide([], 2.5, robot_busy=False) is None


def test_continuous_events_are_not_queued():
    engine = RuleEngine((Rule("person_visible", "look"),), min_interval_s=0)
    engine.decide([Event("person_visible", 0.0, TARGET)], 0.0, robot_busy=True)
    assert engine.decide([], 0.1, robot_busy=False) is None


def test_look_needs_a_target_and_disabled_rules_are_skipped():
    engine = RuleEngine((Rule("person_left", "look"), Rule("person_left", "look_front", enabled=False),
                         Rule("person_left", "wave")), min_interval_s=0)
    command = engine.decide([Event("person_left", 0.0)], 0.0, False)
    assert command.action == "wave"


def test_fired_edge_event_is_consumed():
    engine = RuleEngine((Rule("wave", "wave", cooldown_s=0),), min_interval_s=0)
    assert engine.decide([Event("wave", 0.0, TARGET)], 0.0, False) is not None
    assert engine.decide([], 0.1, False) is None
