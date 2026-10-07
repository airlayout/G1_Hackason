import json
from evaluate_scenario import evaluate
from brain.memory import Memory
from brain.schemas import Decision, Observation


def fixture_report():
    memory = Memory()
    frames = []
    for i, (action, direction) in enumerate([("LOOK", "RIGHT"), ("SEARCH", "RIGHT"), ("LOOK", "LEFT")]):
        payload = {"action": action, "direction": direction, "target": "PERSON",
                   "distance": "UNKNOWN", "confidence": 0.9}
        before = memory.context()
        obs = {"person_visible": i != 1, "person_count": int(i != 1),
               "person_direction": direction if i != 1 else "UNKNOWN", "person_distance": "UNKNOWN",
               "blocking_obstacle": "NONE", "person_transition": ["UNKNOWN", "DISAPPEARED", "APPEARED"][i]}
        memory.update(Decision.model_validate(payload), observation=Observation.model_validate(obs))
        frames.append({"timestamp": i, "decision": payload, "observation": obs, "raw_response": json.dumps({"observation": obs, "decision": payload}),
                       "fallback": False, "memory_before": before, "memory_after": memory.context()})
    return {"status": "pass", "combined_output": True, "timestamps": [0, 1, 2], "frames": frames}


SCENARIO = {"name": "generic direction relation", "timestamps": [0, 1, 2], "checks": [
    {"timestamp": 1, "observation": "disappeared", "allowed_actions": ["SEARCH"],
     "direction_from_memory": "last_seen_person_direction", "target": "PERSON", "person_visible_after": False},
    {"timestamp": 2, "observation": "reappeared", "allowed_actions": ["LOOK", "FOUND"],
     "direction": "LEFT", "target": "PERSON", "person_visible_after": True, "require_previous_absence": True}]}


def test_scenario_checks_dynamic_previous_direction():
    assert evaluate(fixture_report(), SCENARIO)["pass"]


def test_report_action_override_without_raw_change_is_rejected():
    report = fixture_report()
    report["frames"][1]["decision"]["action"] = "LOOK"
    assert not evaluate(report, SCENARIO)["pass"]
    assert evaluate(report, SCENARIO)["integrity_errors"]


def test_search_new_direction_instead_of_previous_is_rejected():
    report = fixture_report()
    report["frames"][1]["decision"]["direction"] = "LEFT"
    assert not evaluate(report, SCENARIO)["pass"]


def test_broken_memory_continuity_is_rejected():
    report = fixture_report()
    report["frames"][2]["memory_before"]["last_seen_person_direction"] = "LEFT"
    assert not evaluate(report, SCENARIO)["pass"]


def test_missing_timestamp_is_rejected():
    report = fixture_report()
    report["frames"].pop()
    assert not evaluate(report, SCENARIO)["pass"]


def test_reappearance_requires_previous_absence():
    report = fixture_report()
    report["frames"][2]["memory_before"]["person_visible_last_frame"] = True
    result = evaluate(report, SCENARIO)
    assert not result["pass"]
    assert any("previous detected absence" in e for e in result["checks"][1]["errors"])
