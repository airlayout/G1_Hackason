"""Serialized shared-model scheduling contracts and separate execution binding."""
from dataclasses import dataclass
import math
from brain.state import State
from brain.split_schemas import CurrentObservation, PlannerDecision


@dataclass(frozen=True, order=True)
class Stamp:
    timestamp: float
    generation: int

    def __post_init__(self):
        if not math.isfinite(self.timestamp) or self.generation < 0:
            raise ValueError('Finite acquisition timestamp and nonnegative generation required')


class SharedState:
    def __init__(self):
        self.state = State()
        self.geometry = self.semantic = None
        self.geometry_stamp = self.semantic_stamp = None

    def update_fast(self, observation, stamp):
        if not isinstance(observation, CurrentObservation) or not isinstance(stamp, Stamp):
            raise TypeError('Validated fast Observation and Stamp required')
        if self.geometry_stamp is not None and stamp <= self.geometry_stamp:
            return False
        self.geometry, self.geometry_stamp = observation, stamp
        self.state.update(observation)
        return True

    def update_semantic(self, observation, stamp):
        if not isinstance(observation, CurrentObservation) or not isinstance(stamp, Stamp):
            raise TypeError('Validated semantic Observation and Stamp required')
        if self.semantic_stamp is not None and stamp <= self.semantic_stamp:
            return False
        # Retain separately for diagnostics. Never write geometry or transition history.
        self.semantic, self.semantic_stamp = observation, stamp
        return True

    def combined(self):
        if self.geometry is None:
            raise ValueError('No authoritative fast geometry')
        fields = self.geometry.model_dump(mode='json')
        # Distance/obstacles are usable only for the same acquired frame and generation.
        if (self.semantic_stamp == self.geometry_stamp and self.semantic is not None
                and self.semantic.person_visible == self.geometry.person_visible
                and self.semantic.person_direction == self.geometry.person_direction):
            fields['person_distance'] = self.semantic.person_distance.value
            fields['blocking_obstacle'] = self.semantic.blocking_obstacle.value
        return CurrentObservation.model_validate(fields)


def resolve(decision, shared):
    """LOOK/FOUND PERSON bind live direction. SEARCH remains the model's memory direction."""
    if not isinstance(decision, PlannerDecision):
        raise TypeError('Resolver accepts validated PlannerDecision only')
    geometry = shared.geometry
    if decision.target.value == 'PERSON' and decision.action.value in {'LOOK', 'FOUND'}:
        if geometry is None or not geometry.person_visible or geometry.person_direction.value == 'UNKNOWN':
            return None  # Suppress stale tracking command; do not fabricate a replacement action.
        fields = decision.model_dump(mode='json', exclude_none=True)
        fields['direction'] = geometry.person_direction.value
        return PlannerDecision.model_validate(fields)
    if decision.action.value == 'APPROACH':
        if geometry is None or not geometry.person_visible or geometry.person_direction.value != 'CENTER':
            return None
    return decision


def execute_resolved(robot, decision, shared):
    resolved = resolve(decision, shared)
    if resolved is not None:
        robot.execute(resolved)
    return resolved
