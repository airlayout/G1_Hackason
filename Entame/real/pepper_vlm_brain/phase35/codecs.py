from latency.codecs import observation, PLANNER_CLASS
from brain.split_brain import parse_flat
from brain.split_schemas import PlannerDecision, CurrentObservation


def decision(raw, format, current):
    if format == 'json':
        return parse_flat(raw, PlannerDecision)
    if format != 'planner_class' or not isinstance(raw, str) or raw not in PLANNER_CLASS:
        raise ValueError('Exactly one verified Planner class required')
    if not isinstance(current, CurrentObservation):
        raise TypeError('Validated immutable Observation required for distance serialization')
    action, direction, target, distance = PLANNER_CLASS[raw]
    if action in {'LOOK', 'FOUND', 'APPROACH'}:
        distance = current.person_distance.value
    return PlannerDecision(action=action, direction=direction, target=target, distance=distance)
