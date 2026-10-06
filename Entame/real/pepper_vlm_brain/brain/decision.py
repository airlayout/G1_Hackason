"""Parsing and error handling, without CUDA or robot dependencies."""
from dataclasses import dataclass, field
import json
from typing import Protocol, Any
from pydantic import ValidationError
from .schemas import Decision, wait_decision, Observation, PerceptionDecision


@dataclass
class DecisionResult:
    decision: Decision = field(default_factory=wait_decision)
    raw_response: str = ""
    fallback: bool = False
    error: str | None = None
    metrics: dict = field(default_factory=dict)
    observation: Observation | None = None

    def to_dict(self):
        return {"decision": self.decision.model_dump(mode="json", exclude_none=True),
                "raw_response": self.raw_response, "fallback": self.fallback,
                "error": self.error, "metrics": self.metrics,
                **({"observation": self.observation.model_dump(mode="json", exclude_none=True)}
                   if self.observation is not None else {})}


def unique_object(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"Duplicate JSON key: {key}")
        obj[key] = value
    return obj


def parse_decision(raw: str, combined=False):
    result = DecisionResult(raw_response=raw if isinstance(raw, str) else "")
    try:
        if not isinstance(raw, str) or not raw.strip() or len(raw) > 16000:
            raise ValueError("Empty, non-text or oversized model response")
        # JSONDecoder respects strings/escaped braces. Accept one outer object,
        # including a markdown fence or surrounding explanation, never multiple.
        decoder = json.JSONDecoder(object_pairs_hook=unique_object)
        objects = []
        cursor = 0
        while (start := raw.find("{", cursor)) >= 0:
            obj, consumed = decoder.raw_decode(raw[start:])
            objects.append(obj)
            cursor = start + consumed
        if len(objects) != 1:
            raise ValueError("Expected exactly one JSON object")
        # A top-level array is not an Action object.
        prefix, suffix = raw[:raw.find("{")].strip(), raw[cursor:].strip()
        if prefix.endswith("[") or suffix.startswith("]"):
            raise ValueError("Action arrays are not supported")
        if combined:
            payload = PerceptionDecision.model_validate(objects[0])
            result.observation, result.decision = payload.observation, payload.decision
        else:
            result.decision = Decision.model_validate(objects[0])
    except (ValueError, TypeError, ValidationError, RecursionError) as exc:
        result.fallback = True
        result.error = f"JSON validation failed: {exc}"
    return result


class VisualEngine(Protocol):
    def generate(self, image: Any, memory_context: dict | None = None, previous_image: Any = None) -> tuple[str, dict]: ...


class Brain:
    def __init__(self, engine: VisualEngine, combined=False):
        self.engine = engine
        self.combined = combined

    def decide(self, image, memory_context=None, previous_image=None):
        try:
            if self.combined:
                raw, metrics = self.engine.generate(image, memory_context, previous_image=previous_image)
            else:
                raw, metrics = self.engine.generate(image, memory_context)
            result = parse_decision(raw, combined=self.combined)
            if (self.combined and not result.fallback and previous_image is None and
                    result.observation.person_transition.value != "UNKNOWN"):
                return DecisionResult(raw_response=raw, fallback=True,
                                      error="No previous image: transition must be UNKNOWN", metrics=metrics)
            result.metrics = metrics
            return result
        except Exception as exc:
            return DecisionResult(fallback=True, error=f"Inference failed: {type(exc).__name__}: {exc}",
                                  metrics=dict(getattr(self.engine, "last_metrics", {})))
