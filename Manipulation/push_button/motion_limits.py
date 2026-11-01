"""Right-arm command limits and timing shared with the standard-library SDK bridge."""
from dataclasses import asdict, dataclass
import math


CONTROL_PERIOD_S = .02
WEIGHT_RAMP_STEPS = 101
ENABLE_DURATION_S = WEIGHT_RAMP_STEPS * CONTROL_PERIOD_S
RELEASE_DURATION_S = ENABLE_DURATION_S
HOME_CONFIRMATION_S = .3 + 3 * CONTROL_PERIOD_S
MOTION_OVERHEAD_S = ENABLE_DURATION_S + HOME_CONFIRMATION_S + RELEASE_DURATION_S
PLANNED_LIMIT_FRACTION = .8
MOTION_TOLERANCE_S = 3.


@dataclass(frozen=True)
class JointMotionLimits:
    joint_speed_rad_s: float = .8
    joint_acceleration_rad_s2: float = 2.

    def __post_init__(self):
        for value, maximum in ((self.joint_speed_rad_s, 1.2),
                               (self.joint_acceleration_rad_s2, 4.)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError('Joint command limits must be positive and at most 1.2rad/s and 4rad/s²')

    def to_dict(self):
        return asdict(self)
