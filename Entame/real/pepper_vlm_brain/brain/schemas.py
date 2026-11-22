from enum import StrEnum
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class Action(StrEnum):
    LOOK = "LOOK"
    APPROACH = "APPROACH"
    SEARCH = "SEARCH"
    AVOID_LEFT = "AVOID_LEFT"
    AVOID_RIGHT = "AVOID_RIGHT"
    FOUND = "FOUND"
    SURPRISE = "SURPRISE"
    WAIT = "WAIT"


class Direction(StrEnum):
    LEFT = "LEFT"
    CENTER = "CENTER"
    RIGHT = "RIGHT"
    UNKNOWN = "UNKNOWN"


class Target(StrEnum):
    PERSON = "PERSON"
    BANANA = "BANANA"
    PLUSHIE = "PLUSHIE"
    OBSTACLE = "OBSTACLE"
    NONE = "NONE"


class Distance(StrEnum):
    NEAR = "NEAR"
    MID = "MID"
    FAR = "FAR"
    UNKNOWN = "UNKNOWN"


class Decision(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    action: Action
    direction: Direction
    target: Target
    distance: Distance
    confidence: float = Field(ge=0, le=1, allow_inf_nan=False)
    scene_summary: str | None = Field(default=None, max_length=240, strict=True)
    observation: str | None = Field(default=None, max_length=240, strict=True)
    salient_fact: str | None = Field(default=None, max_length=120, strict=True)
    salient_fact_ja: str | None = Field(default=None, max_length=120, strict=True)
    speech: str | None = Field(default=None, min_length=1, max_length=80, strict=True)

    @field_validator("confidence", mode="before")
    @classmethod
    def numeric_confidence(cls, value):
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError("confidence must be a JSON number")
        return value

    def structured_fields(self):
        return self.model_dump(mode="json", exclude={"scene_summary", "observation",
                                                       "salient_fact", "salient_fact_ja",
                                                       "speech"})


def wait_decision():
    return Decision(action=Action.WAIT, direction=Direction.UNKNOWN,
                    target=Target.NONE, distance=Distance.UNKNOWN, confidence=0.0)


class BlockingObstacle(StrEnum):
    LEFT = "LEFT"
    CENTER = "CENTER"
    RIGHT = "RIGHT"
    NONE = "NONE"
    UNKNOWN = "UNKNOWN"


class PersonTransition(StrEnum):
    APPEARED = "APPEARED"
    STILL_VISIBLE = "STILL_VISIBLE"
    DISAPPEARED = "DISAPPEARED"
    STILL_ABSENT = "STILL_ABSENT"
    UNKNOWN = "UNKNOWN"


class Observation(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    person_visible: bool = Field(strict=True)
    person_count: int = Field(ge=0, strict=True)
    person_direction: Direction
    person_distance: Distance
    blocking_obstacle: BlockingObstacle
    person_transition: PersonTransition
    scene_summary: str | None = Field(default=None, max_length=240, strict=True)

    @model_validator(mode="after")
    def consistent_presence(self):
        if self.person_visible != (self.person_count > 0):
            raise ValueError("person_visible must agree with person_count")
        if not self.person_visible and (self.person_direction != Direction.UNKNOWN or
                                        self.person_distance != Distance.UNKNOWN):
            raise ValueError("Absent person must have UNKNOWN direction and distance")
        if self.person_transition in {PersonTransition.APPEARED, PersonTransition.STILL_VISIBLE} and not self.person_visible:
            raise ValueError("Visible transition requires current visible person")
        if self.person_transition in {PersonTransition.DISAPPEARED, PersonTransition.STILL_ABSENT} and self.person_visible:
            raise ValueError("Absent transition requires current absent person")
        return self


class PerceptionDecision(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    observation: Observation
    decision: Decision
