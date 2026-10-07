import json
import pytest
from pydantic import ValidationError
from brain.decision import parse_decision, Brain
from brain.schemas import Decision, Action

GOOD = {"action": "LOOK", "direction": "RIGHT", "target": "PERSON",
        "distance": "MID", "confidence": 0.91}


@pytest.mark.parametrize("wrapper", ["{}", "```json\n{}\n```", "Here is my decision:\n{}\nDone."])
def test_valid_json_wrappers(wrapper):
    result = parse_decision(wrapper.format(json.dumps(GOOD)))
    assert not result.fallback
    assert result.decision.action == Action.LOOK
    assert result.decision.direction == "RIGHT"


@pytest.mark.parametrize("key,value", [
    ("action", "MOVE"), ("direction", "UP"), ("target", "DOG"), ("distance", "2m"),
    ("confidence", -0.1), ("confidence", 1.1), ("confidence", "0.9"),
    ("confidence", True), ("confidence", float("nan")), ("confidence", float("inf")),
    ("speed", 0.2), ("scene_summary", "x" * 241)])
def test_invalid_field_falls_back(key, value):
    result = parse_decision(json.dumps({**GOOD, key: value}))
    assert result.fallback and result.error
    assert result.decision.action == Action.WAIT
    assert result.decision.target == "NONE"


@pytest.mark.parametrize("key", list(GOOD))
def test_missing_required_field(key):
    payload = GOOD.copy()
    del payload[key]
    assert parse_decision(json.dumps(payload)).fallback


@pytest.mark.parametrize("raw", ["", "not JSON", "{broken}", "{}", "null", None,
    '[{"action":"WAIT"}]', json.dumps([GOOD]), json.dumps(GOOD) + json.dumps(GOOD),
    '{"action":"WAIT", "action":"LOOK"}', '{"decision":' + json.dumps(GOOD) + '}',
    '{"action":"LOOK",', "x" * 16001])
def test_unparseable_or_ambiguous(raw):
    result = parse_decision(raw)
    assert result.fallback and result.decision.action == Action.WAIT


def test_summary_with_braces_and_unicode():
    result = parse_decision(json.dumps({**GOOD, "scene_summary": '人 {right} "visible"'}))
    assert not result.fallback
    assert "scene_summary" not in result.decision.structured_fields()


def test_schema_direct_validation_and_immutable():
    decision = Decision.model_validate(GOOD)
    with pytest.raises(ValidationError):
        decision.confidence = 0.5


def test_inference_failure_is_wait():
    class FailingEngine:
        def generate(self, image, memory_context=None):
            raise RuntimeError("simulated GPU failure")
    result = Brain(FailingEngine()).decide(object())
    assert result.fallback and result.decision.action == Action.WAIT
    assert "simulated GPU failure" in result.error


def test_real_parser_in_brain():
    class FakeEngine:
        def generate(self, image, memory_context=None):
            return json.dumps(GOOD), {"gpu_used": False}
    result = Brain(FakeEngine()).decide(object())
    assert not result.fallback
    assert result.metrics == {"gpu_used": False}
