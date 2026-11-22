"""Deterministic, image-independent transitions. Planner has no state write API."""
from pydantic import BaseModel, ConfigDict, Field
from .schemas import Direction, PersonTransition
from .split_schemas import CurrentObservation


class StateSnapshot(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True)
    last_seen_person_direction: Direction = Direction.UNKNOWN
    person_visible_last_frame: bool | None = None
    frames_since_person_seen: int = Field(default=0, ge=0)
    transition: PersonTransition = PersonTransition.UNKNOWN


def transition(previous: bool | None, current: bool):
    if previous is None:
        return PersonTransition.UNKNOWN
    return {(False, False): PersonTransition.STILL_ABSENT,
            (False, True): PersonTransition.APPEARED,
            (True, True): PersonTransition.STILL_VISIBLE,
            (True, False): PersonTransition.DISAPPEARED}[(previous, current)]


class State:
    def __init__(self):
        self._snapshot = StateSnapshot()

    @property
    def snapshot(self):
        return self._snapshot

    def update(self, observation: CurrentObservation):
        if not isinstance(observation, CurrentObservation):
            raise TypeError('State accepts validated CurrentObservation only')
        before = self._snapshot
        self._snapshot = StateSnapshot(
            last_seen_person_direction=(observation.person_direction if observation.person_visible
                                        else before.last_seen_person_direction),
            person_visible_last_frame=observation.person_visible,
            frames_since_person_seen=0 if observation.person_visible else before.frames_since_person_seen + 1,
            transition=transition(before.person_visible_last_frame, observation.person_visible))
        return self._snapshot
