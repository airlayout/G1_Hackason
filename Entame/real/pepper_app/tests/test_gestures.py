from conftest import make_person, pose

from pepperapp.gestures import GestureConfig, GestureTracker, count_reversals

L_ELBOW_RAISED = (300, 140)


def _run(tracker, frames, step=0.1):
    """frames: list of keypoint dicts; returns the active gestures after each frame."""
    return [tracker.update((make_person(points),), i * step).get(1, frozenset())
            for i, points in enumerate(frames)]


def test_count_reversals_counts_swings_and_ignores_jitter():
    assert count_reversals([0.0, 0.3, 0.0, 0.3], 0.15) == 2
    assert count_reversals([0.0, 0.05, 0.0, 0.05, 0.0], 0.15) == 0
    assert count_reversals([0.0, 0.1, 0.2, 0.3], 0.15) == 0
    assert count_reversals([0.5], 0.15) == 0


def test_waving_hand_is_wave_not_hand_raised():
    # left wrist above the raised elbow, swinging +-20 px (shoulder width 60 px -> 0.67)
    frames = [pose(**{"7": L_ELBOW_RAISED, "9": (300 + (20 if i % 4 < 2 else -20), 90)})
              for i in range(16)]
    active = _run(GestureTracker(), frames)
    assert "wave" in active[-1]
    assert not any("hand_raised" in a for a in active)


def test_wave_ends_as_soon_as_the_hand_goes_down():
    waving = [pose(**{"7": L_ELBOW_RAISED, "9": (300 + (20 if i % 4 < 2 else -20), 90)})
              for i in range(16)]
    active = _run(GestureTracker(), waving + [pose()] * 4 + waving[:8])
    assert "wave" in active[15]
    assert active[19] == frozenset()          # hand down for 0.4 s: not waving any more
    assert "wave" in active[-1]               # waving again -> a new wave can start


def test_one_lost_wrist_frame_does_not_end_the_wave():
    waving = [pose(**{"7": L_ELBOW_RAISED, "9": (300 + (20 if i % 4 < 2 else -20), 90)})
              for i in range(16)]
    lost = pose(**{"7": L_ELBOW_RAISED})
    lost.pop(9)                               # wrist not detected in this frame
    active = _run(GestureTracker(), waving + [lost] + waving[:2])
    assert all("wave" in a for a in active[15:])


def test_still_raised_hand_is_hand_raised_after_hold():
    frames = [pose(**{"7": L_ELBOW_RAISED, "9": (300, 90)}) for _ in range(12)]
    active = _run(GestureTracker(GestureConfig(hold_s=0.8)), frames)
    assert active[5] == frozenset()          # 0.5 s: not yet
    assert active[9] == {"hand_raised"}      # 0.9 s
    assert "wave" not in active[-1]


def test_both_hands_above_head():
    frames = [pose(**{"7": (300, 140), "8": (200, 140), "9": (300, 60), "10": (200, 60)})
              for _ in range(12)]
    active = _run(GestureTracker(), frames)
    assert active[-1] == {"both_hands_up"}


def test_hands_down_is_nothing():
    active = _run(GestureTracker(), [pose() for _ in range(20)])
    assert all(a == frozenset() for a in active)


def test_hidden_keypoints_give_nothing():
    tracker = GestureTracker()
    for i in range(20):
        hidden = make_person({0: (250, 100)}, conf=0.9)
        assert tracker.update((hidden,), i * 0.1).get(1, frozenset()) == frozenset()


def test_lowering_the_hand_resets_the_hold():
    raised = pose(**{"7": L_ELBOW_RAISED, "9": (300, 90)})
    frames = [raised] * 6 + [pose()] + [raised] * 6
    active = _run(GestureTracker(GestureConfig(hold_s=0.8)), frames)
    assert all("hand_raised" not in a for a in active)


def test_people_without_track_id_are_ignored_and_old_tracks_forgotten():
    tracker = GestureTracker(GestureConfig(forget_s=1.0))
    assert tracker.update((make_person(pose(), track_id=None),), 0.0) == {}
    tracker.update((make_person(pose(), track_id=7),), 0.0)
    tracker.update((), 2.0)
    assert 7 not in tracker._history
