"""Record validated Observation only; Decision records last_action, never perception."""
from dataclasses import dataclass
from .schemas import Action, Direction, Decision, Observation


@dataclass
class Memory:
    last_action: Action = Action.WAIT
    last_seen_person_direction: Direction = Direction.UNKNOWN
    person_visible_last_frame: bool = False
    frames_since_person_seen: int = 0

    def context(self):
        return {"memory_available": True, "last_action": self.last_action.value,
                "last_seen_person_direction": self.last_seen_person_direction.value,
                "person_visible_last_frame": self.person_visible_last_frame,
                "frames_since_person_seen": self.frames_since_person_seen}

    def update(self, decision: Decision, observation: Observation | None = None, perception_valid=True):
        self.last_action = decision.action
        if not perception_valid or observation is None:
            # A parser/GPU failure supplies no evidence of appearance/disappearance.
            return
        visible = observation.person_visible
        self.person_visible_last_frame = visible
        if visible:
            self.frames_since_person_seen = 0
            if observation.person_direction != Direction.UNKNOWN:
                self.last_seen_person_direction = observation.person_direction
        else:
            self.frames_since_person_seen += 1
