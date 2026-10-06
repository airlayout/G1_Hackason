from dataclasses import dataclass
from latency.profiles import Profile
from latency.codecs import PLANNER_CLASS
from brain.split_prompt import PLANNER_SYSTEM

# Generic classes encode action/direction/target. Visible-person distance is a
# serialization binding to the supplied Observation, never an action correction.
CHOICES = {c: fields[:3] for c, fields in PLANNER_CLASS.items() if c != 'N'}
FINITE_PLANNER = (PLANNER_SYSTEM.split('Return ONLY')[0] +
    'If last direction is UNKNOWN do not SEARCH. Missed frames >=4 do not authorize '
    'SEARCH unless transition is DISAPPEARED. Avoidance target OBSTACLE; WAIT target NONE. '
    'Choose one action/direction/target code from this complete catalog: ' +
    '; '.join(f"{c}={' '.join(fields)}" for c, fields in CHOICES.items()) +
    '. N=SURPRISE UNKNOWN NONE (not required by this policy). '
    'Visible-person distance is copied from current_observation during serialization; '
    'SEARCH/AVOID/WAIT distance is UNKNOWN. Return exactly one code, no other tokens.')

EXAMPLE_PLANNER = (
    'Classify the next robot action using only current_observation and memory. '
    'Output ONE code character. First select the action using the following policy, '
    'then serialize it with the dictionary. '
    'Visible person LEFT: LOOK LEFT; RIGHT: LOOK RIGHT. '
    'Visible CENTER at MID/FAR: APPROACH CENTER; at NEAR/UNKNOWN: LOOK CENTER. '
    'APPEARED also permits FOUND toward the visible person. '
    'Absent person with known last direction and (DISAPPEARED or missed frames 1..3): '
    'SEARCH that last direction. Otherwise blocking LEFT: AVOID_RIGHT; blocking RIGHT: '
    'AVOID_LEFT; otherwise WAIT. '
    'Dictionary: A=LOOK LEFT PERSON; B=LOOK CENTER PERSON; C=LOOK RIGHT PERSON; '
    'D=APPROACH CENTER PERSON; E=SEARCH LEFT PERSON; F=SEARCH CENTER PERSON; '
    'G=SEARCH RIGHT PERSON; H=AVOID_LEFT LEFT OBSTACLE; I=AVOID_RIGHT RIGHT OBSTACLE; '
    'J=FOUND LEFT PERSON; K=FOUND CENTER PERSON; L=FOUND RIGHT PERSON; '
    'M=WAIT UNKNOWN NONE; N=SURPRISE UNKNOWN NONE (not required by this policy). '
    'Distance is copied from Observation for LOOK/FOUND/APPROACH, UNKNOWN otherwise. '
    'Generic serialization examples: visible RIGHT at FAR -> C; absent with last CENTER '
    'and missed frames 2 -> F; absent with missed frames 5 and obstacle NONE -> M. '
    'Return only the dictionary code for the supplied input, no explanation.')


@dataclass(frozen=True)
class FastProfile(Profile):
    planner_variant: str = 'catalog'

    def prompt(self, stage):
        if stage == 'planner' and self.planner_class:
            return (EXAMPLE_PLANNER if self.planner_variant=='examples' else FINITE_PLANNER), None
        return super().prompt(stage)


PROFILES = {
    'A': FastProfile('A_geometry_linear', format='class', constrained=True,
                     planner_json=True, linear_patch=True),
    'B': FastProfile('B_full_observation_finite_planner', planner_class=True,
                     linear_patch=True),
    'B2': FastProfile('B_examples_full_observation_finite_planner', planner_class=True,
                     linear_patch=True, planner_variant='examples'),
    'C': FastProfile('C_fast_e2e', format='class', constrained=True,
                     planner_class=True, linear_patch=True),
    'C2': FastProfile('C_examples_fast_e2e', format='class', constrained=True,
                     planner_class=True, linear_patch=True, planner_variant='examples'),
}
