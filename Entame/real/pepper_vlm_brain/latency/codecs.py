"""Strict generic serializations; no scene, timestamp or expected-action rules."""
from brain.split_brain import parse_flat
from brain.split_schemas import CurrentObservation, PlannerDecision

DIRECTION = dict(L='LEFT', C='CENTER', R='RIGHT', U='UNKNOWN')
DISTANCE = dict(N='NEAR', M='MID', F='FAR', U='UNKNOWN')
OBSTACLE = {**DIRECTION, '0':'NONE'}
ACTION = dict(L='LOOK', A='APPROACH', S='SEARCH', X='AVOID_LEFT', Y='AVOID_RIGHT',
              F='FOUND', R='SURPRISE', W='WAIT')
TARGET = dict(P='PERSON', B='BANANA', T='PLUSHIE', O='OBSTACLE', **{'0':'NONE'})
CLASS = dict(A=None, B='LEFT', C='CENTER', D='RIGHT')
PLANNER_CLASS = {
    'A':('LOOK','LEFT','PERSON','UNKNOWN'), 'B':('LOOK','CENTER','PERSON','UNKNOWN'),
    'C':('LOOK','RIGHT','PERSON','UNKNOWN'), 'D':('APPROACH','CENTER','PERSON','MID'),
    'E':('SEARCH','LEFT','PERSON','UNKNOWN'), 'F':('SEARCH','CENTER','PERSON','UNKNOWN'),
    'G':('SEARCH','RIGHT','PERSON','UNKNOWN'), 'H':('AVOID_LEFT','LEFT','OBSTACLE','UNKNOWN'),
    'I':('AVOID_RIGHT','RIGHT','OBSTACLE','UNKNOWN'), 'J':('FOUND','LEFT','PERSON','UNKNOWN'),
    'K':('FOUND','CENTER','PERSON','UNKNOWN'), 'L':('FOUND','RIGHT','PERSON','UNKNOWN'),
    'M':('WAIT','UNKNOWN','NONE','UNKNOWN'), 'N':('SURPRISE','UNKNOWN','NONE','UNKNOWN')}


def fields(raw):
    if not isinstance(raw,str) or len(raw)>64: raise ValueError('Invalid compact response')
    parts=raw.strip().split('|')
    if len(parts)!=4 or any(len(p)!=1 for p in parts): raise ValueError('Exactly four single-character codes required')
    return parts


def observation(raw, format):
    if format=='json': return parse_flat(raw,CurrentObservation)
    if format=='class':
        if not isinstance(raw,str) or len(raw)>64: raise ValueError('Invalid class response')
        code=raw.strip()
        if code not in CLASS: raise ValueError('Unknown finite scene code')
        direction=CLASS[code]
        return CurrentObservation(person_visible=direction is not None,person_count=int(direction is not None),
                                  person_direction=direction or 'UNKNOWN',person_distance='UNKNOWN',blocking_obstacle='UNKNOWN')
    if format!='compact': raise ValueError('Unknown serialization')
    visible,side,distance,blocking=fields(raw)
    if visible not in {'0','1'}: raise ValueError('Unknown presence code')
    try:
        return CurrentObservation(person_visible=visible=='1',person_count=int(visible),
                                  person_direction=DIRECTION[side],person_distance=DISTANCE[distance],
                                  blocking_obstacle=OBSTACLE[blocking])
    except KeyError as exc: raise ValueError('Unknown Observation code') from exc


def decision(raw, format):
    if format=='json': return parse_flat(raw,PlannerDecision)
    if format=='planner_class':
        if not isinstance(raw,str) or raw.strip() not in PLANNER_CLASS: raise ValueError('Unknown Planner class')
        return PlannerDecision(**dict(zip(['action','direction','target','distance'],PLANNER_CLASS[raw.strip()])))
    if format not in {'compact','class'}: raise ValueError('Unknown serialization')
    action,side,target,distance=fields(raw)
    try:
        return PlannerDecision(action=ACTION[action],direction=DIRECTION[side],target=TARGET[target],distance=DISTANCE[distance])
    except KeyError as exc: raise ValueError('Unknown Decision code') from exc
