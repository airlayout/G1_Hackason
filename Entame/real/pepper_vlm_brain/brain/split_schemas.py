"""Phase 3.2 contracts. Legacy combined schemas remain in schemas.py."""
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from .schemas import Action, Direction, Distance, Target, BlockingObstacle, PersonTransition


class CurrentObservation(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True)
    person_visible: bool = Field(strict=True)
    person_count: int = Field(ge=0, strict=True)
    person_direction: Direction
    person_distance: Distance
    blocking_obstacle: BlockingObstacle

    @model_validator(mode='after')
    def consistent_presence(self):
        if self.person_visible != (self.person_count > 0):
            raise ValueError('person_visible must agree with person_count')
        if not self.person_visible and (self.person_direction != Direction.UNKNOWN or self.person_distance != Distance.UNKNOWN):
            raise ValueError('Absent person requires count=0 and UNKNOWN direction/distance')
        return self


class PlannerDecision(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True)
    action: Action
    direction: Direction
    target: Target
    distance: Distance
    confidence: float | None = Field(default=None, ge=0, le=1, allow_inf_nan=False)

    @field_validator('confidence', mode='before')
    @classmethod
    def numeric_confidence(cls, value):
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
            raise ValueError('confidence must be an optional JSON number')
        return value

    def structured_fields(self):
        return self.model_dump(mode='json')


def wait_planner_decision():
    return PlannerDecision(action='WAIT', direction='UNKNOWN', target='NONE', distance='UNKNOWN')
