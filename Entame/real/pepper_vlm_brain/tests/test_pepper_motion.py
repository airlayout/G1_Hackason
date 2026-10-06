import pytest

from adapters.pepper.motion import (
    DryRunTransport, MotionValidationError, PepperMotionAdapter, PepperMotionError,
)


def adapter(lines=None):
    transport = DryRunTransport(output=(lines if lines is not None else []).append)
    return PepperMotionAdapter(transport), transport


def test_required_dry_run_sequence():
    lines = []
    motion, transport = adapter(lines)
    motion.execute({"action": "MOVE", "forward_m": 0.3, "speed_mps": 0.2})
    motion.execute({"action": "TURN", "direction": "left", "degrees": 30})
    motion.execute({"action": "LOOK", "x": 1, "y": 1, "z": 1.2})
    motion.execute({"action": "STOP"})
    assert lines == [
        "DRY-RUN: MOVE forward=0.3m sideways=0m speed=0.2",
        "DRY-RUN: TURN left 30deg",
        "DRY-RUN: LOOK x=1 y=1 z=1.2 policy=head_only",
        "DRY-RUN: STOP",
    ]
    assert [path for path, _ in transport.commands] == ["/move", "/turn", "/look", "/stop"]


@pytest.mark.parametrize("command", [
    {"action": "MOVE", "forward_m": 0.0},
    {"action": "MOVE", "forward_m": 0.6},
    {"action": "MOVE", "forward_m": 0.2, "speed_mps": 0.5},
    {"action": "TURN", "direction": "left", "degrees": 10},
    {"action": "TURN", "direction": "around", "degrees": 30},
    {"action": "LOOK", "x": 0, "y": 0, "z": 0},
    {"action": "STOP", "degrees": 30},
])
def test_rejects_unsafe_or_malformed_commands(command):
    motion, _ = adapter()
    with pytest.raises(MotionValidationError):
        motion.execute(command)


def test_directional_look_is_head_only():
    motion, transport = adapter()
    motion.execute({"action": "LOOK", "direction": "right"})
    assert transport.commands[-1] == ("/look", {
        "action": "LOOK", "x": 1.0, "y": -1.0, "z": 1.2,
        "movement_policy": "head_only",
    })


def test_running_movement_is_blocked_but_stop_is_allowed():
    class Busy(DryRunTransport):
        def status(self):
            return {"ready": True, "movement_running": True}

    transport = Busy(output=lambda _: None)
    motion = PepperMotionAdapter(transport)
    with pytest.raises(PepperMotionError, match="already running"):
        motion.turn(15)
    motion.stop()
