from resident_worker import classify_runtime_exception
from robot_transport import RecoverableMotionEnvelopeExceeded


def test_tilt_exceed_is_recoverable_only_in_hackathon_runtime():
    error = RecoverableMotionEnvelopeExceeded(
        tilt_peak=.04, tilt_limit=.30, gyro_peak=1.48, gyro_limit=.80
    )
    assert classify_runtime_exception(error, False) == "HARD_FAULT"
    assert classify_runtime_exception(error, True) == "RECOVERABLE_ABORT"


def test_hard_fault_categories_never_become_recoverable():
    errors = [
        RuntimeError("Arm control ownership blocked: external writer"),
        RuntimeError("Joint limit violation"),
        RuntimeError("Runtime hard safety: new penetration"),
        RuntimeError("Arm motor error or high temperature"),
        RuntimeError("DDS write failed; robot receipt is not guaranteed"),
        RuntimeError("Non-finite robot state"),
        RuntimeError("Unverified hardware variant"),
    ]
    assert all(
        classify_runtime_exception(error, True) == "HARD_FAULT"
        for error in errors
    )
