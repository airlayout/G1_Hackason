"""Shared robot-state and publication policy for verified real-arm trials."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class ExecutionPolicy:
    state_timeout_s: float
    leg_speed_rad_s: float
    gyro_rad_s: float
    record_deviations: bool
    synchronous_stream: bool

    def to_dict(self):
        return asdict(self)


STRICT_POLICY = ExecutionPolicy(.1, .15, .10, False, False)
COLLECTION_POLICY = ExecutionPolicy(.4, .5, .3, True, True)


def execution_policy(record_deviations=False):
    return COLLECTION_POLICY if record_deviations else STRICT_POLICY
