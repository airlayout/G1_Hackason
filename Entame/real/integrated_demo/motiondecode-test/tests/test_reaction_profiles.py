from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from reaction_profiles import build_named_paths, load_trajectory_profile
from run_real_reaction import experimental_preflight


ROOT = Path(__file__).resolve().parents[1]


def snapshot() -> dict:
    return json.loads(
        (ROOT / "output/fast_game_reactions_20260916/fresh_q0.json").read_text()
    )


def test_selected_fast_profiles_start_at_fresh_q0_and_exclude_legs() -> None:
    expected = {"surprise": (1.0, 1.5), "found": (1.0, 4.0), "joy": (1.5, 2.0)}
    q0 = np.asarray(snapshot()["all_q"])
    for name, (speed, duration) in expected.items():
        paths, _, durations, metadata = build_named_paths(snapshot(), name, 0.5, 0.25)
        assert metadata["playback_speed"] == speed
        assert durations["clip"] == duration
        assert np.array_equal(paths["entry"][0], q0[12:])
        assert np.array_equal(paths["clip"][0], q0[12:])
        assert np.array_equal(paths["return"][-1], q0[12:])
        assert metadata["fresh_q0_aligned"] is True


def test_only_real_validated_profiles_are_allowlisted() -> None:
    for name in ("surprise", "found"):
        assert load_trajectory_profile(name)["real_g1_validated"] is True
    assert load_trajectory_profile("joy")["real_g1_validated"] is False


def test_suspended_profile_is_not_validated_and_only_softens_four_checks() -> None:
    profile = load_trajectory_profile("found_100_suspended_experimental")
    assert profile["real_g1_validated"] is False
    assert profile["reaction_engine_available"] is False
    report = {"passed": False, "checks": {
        "joint_limits": True, "collision": True, "finite": True,
        "timestamp_continuity": True, "first_frame_jump": True,
        "last_frame_jump": True, "velocity": False, "acceleration": False,
        "max_delta_le_0_8": False, "clip_range": False,
    }}
    policy = experimental_preflight(report)
    assert policy["hard_safety_passed"] is True
    report["checks"]["collision"] = False
    assert experimental_preflight(report)["hard_safety_passed"] is False
    surprise = load_trajectory_profile("surprise_100_hackathon_suspended")
    assert surprise["real_g1_validated"] is False
    assert surprise["reaction_engine_available"] is False
    assert surprise["manual_gate_token"] == "RUN_SURPRISE_100_1X"
    assert surprise["fast_reaction"] is True
    joy = load_trajectory_profile("joy_100_hackathon_suspended")
    assert joy["validation_mode"] == "hackathon_suspended"
    assert joy["arms_amplitude"] == joy["waist_amplitude"] == 1.0
    assert joy["real_g1_validated"] is False


def test_fast_surprise_changes_only_smooth_outer_transitions() -> None:
    regular, _, regular_durations, _ = build_named_paths(
        snapshot(), "surprise_100_hackathon_suspended", 1.0, 1.0)
    fast, _, fast_durations, metadata = build_named_paths(
        snapshot(), "surprise_100_hackathon_suspended", 1.0, 1.0,
        fast_reaction=True)
    np.testing.assert_array_equal(fast["clip"], regular["clip"])
    assert fast_durations == {"entry": .1, "clip": 1.0, "return": .5}
    assert regular_durations["return"] > fast_durations["return"]
    assert np.array_equal(fast["entry"][0], np.asarray(snapshot()["all_q"])[12:])
    assert np.array_equal(fast["return"][-1], np.asarray(snapshot()["all_q"])[12:])
    assert metadata["fast_reaction"] is True
