from dataclasses import replace
import json
from brain.split_prompt import planner_user
from latency.engine import LatencyQwen


class FastQwen(LatencyQwen):
    """Same weights/Linear backend; B's Perception uses the exact uninstrumented reference."""
    def _profile_generate(self, messages, stage, prompt):
        original = self.profile
        if stage == 'perception' and original.name.startswith('B_'):
            self.profile = replace(original, instrument=False)
        try:
            return super()._profile_generate(messages, stage, prompt)
        finally:
            self.profile = original

    def plan(self, observation, snapshot):
        raw, metrics = super().plan(observation, snapshot)
        metrics['planner_input'] = json.loads(planner_user(observation, snapshot))
        metrics['chosen_code'] = raw if self.profile.planner_class else None
        return raw, metrics
