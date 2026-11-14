"""ボトルの終点では停止を完了し、揺れで押し直す動作を繰り返さない。"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from alignment import ContinuousNormalController, SmoothNormalSettings
from bottle import validate_push_distance


def controller(latch=True):
    return ContinuousNormalController(
        [.3, -.25, .83], [1., 0., 0.], .015, .05, [.3337, -.25, .83],
        SmoothNormalSettings(completion_tolerance_m=.0015, completion_hold_s=.04),
        travel_validator=validate_push_distance, latch_completion=latch)


def test_terminal_entry_brakes_without_pushing_again_on_small_pose_noise():
    servo = controller()
    servo.velocity = .0012
    assert servo.next_goal([.3337, -.25, .83], .02) is not None
    # The second sample is 2.5mm short, still comfortably within bottle acceptance.
    assert servo.next_goal([.3325, -.25, .83], .02) is None
    assert servo.velocity == pytest.approx(0.)


def test_latched_completion_keeps_overtravel_and_command_difference_guards():
    servo = controller()
    servo.next_goal([.3337, -.25, .83], .02)
    with pytest.raises(ValueError, match='押込量'):
        servo.next_goal([.3361, -.25, .83], .02)
    with pytest.raises(ValueError, match='1cm'):
        controller().next_goal([.32, -.25, .83], .02)


def test_default_button_completion_still_requires_continuous_in_tolerance_samples():
    servo = controller(latch=False)
    servo.next_goal([.3337, -.25, .83], .02)
    assert servo.next_goal([.3325, -.25, .83], .02) is not None
    assert servo.settled_s == 0.
