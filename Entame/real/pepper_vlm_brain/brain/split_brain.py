"""Source-independent Perception -> deterministic State -> text Planner."""
from dataclasses import dataclass, field
import json
from time import perf_counter
from pydantic import ValidationError
from .decision import unique_object
from .split_schemas import CurrentObservation, PlannerDecision, wait_planner_decision
from .state import State


def reject_nonfinite_constant(value):
    raise ValueError(f'Non-JSON numeric constant: {value}')


def bare_json(raw):
    try:
        return isinstance(json.loads(raw, object_pairs_hook=unique_object, parse_constant=reject_nonfinite_constant), dict)
    except (ValueError, TypeError):
        return False


def parse_flat(raw, schema):
    if not isinstance(raw, str) or not raw.strip() or len(raw) > 16000:
        raise ValueError('Empty, non-text or oversized response')
    text = raw.strip()
    if text.startswith('```json\n') and text.endswith('```'):
        text = text[len('```json\n'):-3].strip()
    data = json.loads(text, object_pairs_hook=unique_object, parse_constant=reject_nonfinite_constant)
    if not isinstance(data, dict):
        raise ValueError('Exactly one flat JSON object required')
    return schema.model_validate(data)


@dataclass
class PerceptionResult:
    observation: CurrentObservation | None = None
    raw_response: str = ''
    fallback: bool = False
    error: str | None = None
    metrics: dict = field(default_factory=dict)

    def to_dict(self):
        return {'observation': self.observation.model_dump(mode='json') if self.observation else None,
                'raw_response': self.raw_response, 'bare_json': bare_json(self.raw_response), 'fallback': self.fallback,
                'error': self.error, 'metrics': self.metrics}


@dataclass
class PlannerResult:
    decision: PlannerDecision = field(default_factory=wait_planner_decision)
    raw_response: str = ''
    fallback: bool = False
    error: str | None = None
    metrics: dict = field(default_factory=dict)

    def to_dict(self):
        return {'decision': self.decision.model_dump(mode='json', exclude_none=True),
                'raw_response': self.raw_response, 'bare_json': bare_json(self.raw_response), 'fallback': self.fallback,
                'error': self.error, 'metrics': self.metrics}


class Perception:
    def __init__(self, engine): self.engine = engine

    def observe(self, image):
        result = PerceptionResult()
        try:
            result.raw_response, result.metrics = self.engine.perceive(image)
            result.observation = parse_flat(result.raw_response, CurrentObservation)
        except Exception as exc:
            result.fallback = True
            result.error = f'{type(exc).__name__}: {exc}'
        return result


class Planner:
    def __init__(self, engine): self.engine = engine

    def decide(self, observation: CurrentObservation, snapshot):
        result = PlannerResult()
        try:
            # Frozen models contain no mutable nested containers and no State write API.
            result.raw_response, result.metrics = self.engine.plan(observation, snapshot)
            result.decision = parse_flat(result.raw_response, PlannerDecision)
        except Exception as exc:
            result.fallback = True
            result.error = f'{type(exc).__name__}: {exc}'
        return result


class SplitBrain:
    def __init__(self, engine):
        self.perception, self.planner, self.state = Perception(engine), Planner(engine), State()

    def decide(self, image):
        start = perf_counter()
        before = self.state.snapshot
        pstart = perf_counter()
        perception = self.perception.observe(image)
        perception_s = perf_counter() - pstart
        state_start = perf_counter()
        snapshot = self.state.update(perception.observation) if not perception.fallback else before
        state_s = perf_counter() - state_start
        planner_start = perf_counter()
        planner = (self.planner.decide(perception.observation, snapshot) if not perception.fallback else
                   PlannerResult(fallback=True, error='Planner skipped: invalid Perception'))
        planner_s = perf_counter() - planner_start if not perception.fallback else 0.0
        return {'perception': perception.to_dict(), 'state_before': before.model_dump(mode='json'),
                'state_after': snapshot.model_dump(mode='json'), 'planner': planner.to_dict(),
                'timing': {'perception_wall_s': perception_s, 'state_update_s': state_s,
                           'planner_wall_s': planner_s, 'total_brain_s': perf_counter() - start}}
