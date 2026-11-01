"""物理機体を動かさず、破損した指令・通信断・停止の境界を検証する。"""

import numpy as np
import pytest

from g1_e2e.vla.config import load_config
from g1_e2e.vla.control import ArmControl


@pytest.fixture
def control():
    control = ArmControl(load_config()["arm"])
    control.update_state(np.zeros(29), np.zeros(29), now=10)
    control.camera_at = 10
    return control


def start(control):
    identifier = control.observe(now=10)
    session = control.start(now=10, max_task_s=30)
    return session, identifier


def test_start_holds_observed_pose_and_does_not_jump_to_zero(control):
    control.q[15:29] = 0.3
    start(control)
    target, weight = control.tick(now=10.02, dt=0.02)
    np.testing.assert_allclose(target, 0.3)
    assert weight == pytest.approx(0.02)


def test_velocity_is_bounded_even_when_control_loop_is_delayed(control):
    session, identifier = start(control)
    control.accept(session, identifier, np.full((8, 14), 0.4), 30, now=10)
    q, _ = control.tick(now=10.02, dt=100)
    assert np.max(np.abs(q)) <= 0.4 * 0.04


@pytest.mark.parametrize(
    "positions",
    [
        np.zeros((3, 13)),
        np.full((8, 14), np.nan),
        np.full((8, 14), np.inf),
        np.full((8, 14), 3),
        np.full((8, 14), 0.9),
        np.zeros((41, 14)),
    ],
)
def test_invalid_chunk_never_changes_target(control, positions):
    session, identifier = start(control)
    with pytest.raises(ValueError):
        control.accept(session, identifier, positions, 30, now=10)
    np.testing.assert_array_equal(control.target, 0)


@pytest.mark.parametrize("failure", ["state", "camera", "command", "deadline", "tracking"])
def test_failure_revokes_session_and_returns_weight(control, failure):
    session, identifier = start(control)
    control.accept(session, identifier, np.zeros((8, 14)), 30, now=10)
    control.tick(now=10.02, dt=0.02)
    now = 10.04
    if failure == "state":
        control.state_at = 9
    elif failure == "camera":
        control.camera_at = 8
    elif failure == "command":
        control.command_at = 6
    elif failure == "deadline":
        control.deadline = 10.03
    else:
        control.target[0] = 0.6
    _target, weight = control.tick(now=now, dt=0.02)
    assert control.session is None
    assert weight == 0
    assert control.reason
    with pytest.raises(ValueError, match="停止済み"):
        control.accept(session, identifier, np.zeros((8, 14)), 30, now=now)


def test_old_predictions_cannot_restart_after_stop(control):
    session, identifier = start(control)
    control.stop()
    control.start(now=10.01, max_task_s=30)
    with pytest.raises(ValueError, match="停止済み"):
        control.accept(session, identifier, np.zeros((8, 14)), 30, now=10.02)


def test_prediction_timeout_uses_g1_monotonic_clock(control):
    session, identifier = start(control)
    control.update_state(np.zeros(29), np.zeros(29), now=14)
    control.camera_at = 14
    with pytest.raises(ValueError, match="古すぎ"):
        control.accept(session, identifier, np.zeros((8, 14)), 30, now=14)


def test_observation_replay_is_rejected_even_after_newer_chunk(control):
    session, first = start(control)
    second = control.observe(now=10.01)
    control.accept(session, first, np.zeros((8, 14)), 30, now=10.02)
    control.accept(session, second, np.zeros((8, 14)), 30, now=10.03)
    with pytest.raises(ValueError):
        control.accept(session, first, np.zeros((8, 14)), 30, now=10.04)


def test_waiting_for_stop_to_finish_before_next_start(control):
    start(control)
    control.tick(now=10.1, dt=0.1)
    control.stop()
    with pytest.raises(ValueError, match="停止処理"):
        control.start(now=10.1, max_task_s=30)


def test_stale_camera_cannot_start(control):
    control.camera_at = 0
    with pytest.raises(ValueError, match="カメラ"):
        control.start(now=10, max_task_s=30)
