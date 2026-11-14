"""指令経路の逆再生と、経路検査が飛び越しを許さないことを検査する。"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from return_path import ApproachPath, validate_return_path


def test_replay_keeps_correction_waypoint_and_returns_home_without_mutable_alias():
    home = np.zeros(7)
    path = ApproachPath(home)
    raised = np.array([0.4, -0.3, 0, 0.2, 0, 0, 0])
    corrected = raised.copy()
    corrected[1] += 0.1
    path.append(raised, 2.0)
    path.append(corrected, 0.5)
    raised[:] = home[:] = 1  # 呼出元の配列が変わっても記録を変えない。
    path.freeze()
    targets = np.array(path.reversed_targets())
    np.testing.assert_allclose(targets[0], corrected)
    np.testing.assert_allclose(targets[-1], np.zeros(7))
    assert any(np.allclose(target, [0.4, -0.3, 0, 0.2, 0, 0, 0]) for target in targets)
    assert np.max(np.abs(np.diff(targets, axis=0)))*50 <= 0.8+1e-12
    assert (len(targets)-1)/50 >= 2.5
    with pytest.raises(ValueError, match="押下開始後"):
        path.append(corrected, 0.02)


def test_replay_slows_fast_recording_instead_of_skipping_samples():
    path = ApproachPath(np.zeros(7))
    path.append(np.full(7, 0.1), 0.02)
    path.freeze()
    targets = np.array(path.reversed_targets())
    assert (len(targets)-1)/50 > 0.02
    assert np.max(np.abs(np.diff(targets, axis=0)))*50 <= 0.8+1e-12


def test_missing_or_corrupt_history_cannot_be_replayed():
    path = ApproachPath(np.zeros(7))
    with pytest.raises(ValueError):
        path.freeze()
    with pytest.raises(ValueError):
        path.reversed_targets()
    path.append(np.full(7, 0.1), 0.2)
    path.freeze()
    path.samples[-1] = (0.0, path.samples[-1][1])
    with pytest.raises(ValueError, match="時刻"):
        path.reversed_targets()


def test_collision_validation_checks_adjacent_segments_and_restores_state_on_failure():
    kin = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(29)), qadr=np.arange(22, 29))
    starts = []
    kin.update = lambda q: setattr(kin.data, "qpos", q.copy())

    def validate(goal, face, normal):
        starts.append(kin.data.qpos[22:].copy())
        if len(starts) == 3:
            raise ValueError("collision")

    kin.validate_path = validate
    goals = [np.full(7, value) for value in (0.1, 0.2, 0.3)]
    with pytest.raises(ValueError, match="collision"):
        validate_return_path(kin, goals, None, None)
    np.testing.assert_allclose(starts, [np.zeros(7), goals[0], goals[1]])
    np.testing.assert_allclose(kin.data.qpos, np.zeros(29))
