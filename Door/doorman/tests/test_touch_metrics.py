"""touch_metrics.py の分け方の試験（10-19 夜）。numpy だけで回る: python -m pytest Door/doorman/tests"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from touch_metrics import IMPACT_MAX_N, LEVER_STOP_DEG, load, split_contact  # noqa: E402


def test_impact_at_first_step_is_counted_as_hit() -> None:
    # 1 ステップ目に 40 N で当たり、その後は 12 N で動かす（元の方策の典型）
    f = np.array([40.0, 12.0, 12.0, 12.0, 12.0, 12.0])
    lev = np.array([2.0, 10.0, 20.0, 30.0, 40.0, 44.0])
    out = split_contact(f, lev)
    assert out["first_peak_n"] == 40.0
    assert out["over15_cause"] == "当たり"
    assert out["moving_n"] == 12.0


def test_bottoming_is_separated_from_moving() -> None:
    # 当たりはやさしい（8 N）が、レバーが止まりに着いてから 60 N で押し付ける
    f = np.array([8.0, 9.0, 10.0, 60.0, 30.0, 25.0])
    lev = np.array([2.0, 20.0, 40.0, LEVER_STOP_DEG + 1, 45.0, 45.0])
    out = split_contact(f, lev)
    assert out["first_peak_n"] == 9.0
    assert out["over15_cause"] == "底突き"
    assert out["bottom_n"] == 60.0
    assert out["moving_n"] == 9.0
    assert out["reached_stop"]


def test_push_while_moving_is_push() -> None:
    f = np.array([5.0, 8.0, 20.0, 10.0, 10.0])
    lev = np.array([1.5, 5.0, 10.0, 15.0, 20.0])
    out = split_contact(f, lev)
    assert out["over15_cause"] == "押し"
    assert not out["reached_stop"]
    assert np.isnan(out["bottom_n"])


def test_gentle_touch_has_no_cause() -> None:
    f = np.full(6, IMPACT_MAX_N - 1)
    lev = np.linspace(2, 30, 6)
    assert split_contact(f, lev)["over15_cause"] == "なし"


def _write_record(d: str, peak_rows: int) -> None:
    n, e = 6, 2
    base = {
        "stage": np.zeros((n, e)), "episode_length": np.tile(np.arange(n)[:, None], (1, e)),
        "root_pos": np.zeros((n, e, 3)), "door_dof_pos": np.zeros((n, e, 2)),
        "hand_lever_force": np.zeros((n, e, 3)), "palm_used": np.zeros((n, e, 3)), "grasp_target": np.ones((n, e, 3)),
    }
    base["hand_lever_force_peak"] = np.arange(peak_rows * e, dtype=float).reshape(peak_rows, e) + 1
    base["hand_lever_force_mean"] = base["hand_lever_force_peak"] / 2
    np.savez(os.path.join(d, "states_0000.npz"), **base)


def test_missing_first_row_of_peak_is_padded(tmp_path) -> None:
    # 10-19 18:00 の試運転の記録: 物理を回す前の最初の行だけ peak・mean が欠けている
    _write_record(str(tmp_path), peak_rows=5)
    r = load(str(tmp_path))
    assert r["hand_lever_force_peak"].shape == r["stage"].shape
    assert np.all(r["hand_lever_force_peak"][0] == 0)
    assert r["hand_lever_force_peak"][1, 0] == 1.0


def test_aligned_record_is_unchanged(tmp_path) -> None:
    _write_record(str(tmp_path), peak_rows=6)
    r = load(str(tmp_path))
    assert r["hand_lever_force_peak"][0, 0] == 1.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
