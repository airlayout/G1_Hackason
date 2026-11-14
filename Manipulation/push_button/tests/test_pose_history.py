"""遅延した画像には撮影時の関節角・IMU姿勢を対応させる。"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_history import PoseHistory


def test_interpolates_exposure_pose_and_wraps_imu_angles():
    history = PoseHistory()
    history.append((np.zeros(29), np.deg2rad([0, 0, 179]), 1.))
    history.append((np.ones(29), np.deg2rad([2, 4, -179]), 1.1))
    q, rpy = history.at(1.05, None)
    assert q == pytest.approx(np.full(29, .5))
    assert np.rad2deg(rpy) == pytest.approx([1, 2, 180])


def test_history_owns_samples_and_ignores_duplicate_states():
    history = PoseHistory()
    q, rpy = np.zeros(29), np.zeros(3)
    history.append((q, rpy, 1.))
    q[:] = 1
    history.append((q, rpy, 1.))
    observed, _ = history.at(.9, None)
    observed[:] = 2
    observed, _ = history.at(1.2, None)
    assert np.all(observed == 0)
    assert len(history.samples) == 1


def test_empty_history_uses_supplied_state():
    q, rpy = PoseHistory().at(1., (np.zeros(29), np.zeros(3), 1.))
    assert q.shape == (29,)
    assert rpy.shape == (3,)
