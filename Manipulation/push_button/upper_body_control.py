"""Shared rt/arm_sdk upper-body ownership and holding gains (29-DoF G1)."""
import math

WAIST_INDICES = (12, 13, 14)
UPPER_BODY_INDICES = tuple(range(12, 29))
UPPER_BODY_JOINTS = (
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
)
# The official G1 arm7 example uses 60 / 1.5 for waist holding. Preserve the
# existing lower arm gains, and model the held left arm with those same gains.
UPPER_BODY_KP = (60., 60., 60.) + (40., 40., 40., 40., 15., 15., 15.) * 2
UPPER_BODY_KD = (1.5,) * 17
WAIST_TORQUE_LIMIT_NM = 10.


def validate_waist_tau(values):
    values = tuple(float(value) for value in values)
    if (len(values) != 3 or not all(math.isfinite(value) for value in values)
            or max(abs(value) for value in values) > WAIST_TORQUE_LIMIT_NM):
        raise ValueError('腰の補助トルクは有限な3値・各10Nm以内が必要です')
    return values


def control_contract():
    return dict(schema='g1-upper-body-hold-v1', joint_indices=list(UPPER_BODY_INDICES),
                joint_names=list(UPPER_BODY_JOINTS), kp=list(UPPER_BODY_KP), kd=list(UPPER_BODY_KD),
                waist_feedforward_limit_nm=WAIST_TORQUE_LIMIT_NM)
