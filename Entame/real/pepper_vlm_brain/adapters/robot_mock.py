from typing import Protocol
from brain.schemas import Decision
from brain.split_schemas import PlannerDecision


class RobotAdapter(Protocol):
    def execute(self, decision: Decision | PlannerDecision) -> None: ...


class MockRobot:
    def execute(self, decision: Decision | PlannerDecision) -> None:
        if not isinstance(decision, (Decision, PlannerDecision)):
            raise TypeError('Robot requires a validated Decision')
        fields = decision.structured_fields()
        confidence = fields.get("confidence")
        confidence_text = "n/a" if confidence is None else f"{confidence:.2f}"
        self.last_output = (f"[MOCK ROBOT] {fields['action']} {fields['direction']} "
              f"target={fields['target']} distance={fields['distance']} "
              f"confidence={confidence_text}")
        print(self.last_output)
