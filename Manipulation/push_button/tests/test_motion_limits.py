"""Motion budgets, command headroom and consistent limits without robot hardware."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]

from alignment import JointCommandFilter
from motion_limits import JointMotionLimits, MOTION_OVERHEAD_S, ENABLE_DURATION_S, RELEASE_DURATION_S
from robust_options import add_arguments, parse
from robust_planner import SearchSettings, retime_reference, retime_button_arm, spline_peaks


@pytest.mark.parametrize('speed,acceleration', [(0, 4), (1.3, 4), (1.2, 4.1),
                                              (float('nan'), 4), (True, 4), (1.2, -1)])
def test_invalid_limits_reject_before_any_motion(speed, acceleration):
    with pytest.raises(ValueError, match='Joint command limits'):
        JointMotionLimits(speed, acceleration)


def test_robust_defaults_and_json_cli_precedence(tmp_path):
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    _, defaults = parse(parser, [])
    assert defaults.motion_duration_s == 30
    assert defaults.scenarios == 3 and defaults.position_span_m == .003
    assert defaults.bottle_feedback == 'off'
    assert defaults.motion_limits == JointMotionLimits(1.2, 4)
    assert JointMotionLimits() == JointMotionLimits(.8, 2)
    path = tmp_path/'trial.json'
    path.write_text(json.dumps(dict(motion_duration_s=40, joint_speed_rad_s=.9,
                                   joint_acceleration_rad_s2=3)))
    args, settings = parse(parser, ['--trial-config', str(path), '--motion-duration-s', '30'])
    assert args.motion_duration_s == settings.motion_duration_s == 30
    assert settings.motion_limits == JointMotionLimits(.9, 3)
    with pytest.raises(ValueError, match='Motion duration'):
        SearchSettings(motion_duration_s=MOTION_OVERHEAD_S)


def test_spline_extrema_include_velocity_peak_between_ticks():
    from scipy.interpolate import CubicHermiteSpline
    spline = CubicHermiteSpline([0, 1], [[0.]*7, [1.]*7], np.zeros((2, 7)))
    speed, acceleration = spline_peaks(spline)
    assert speed == pytest.approx(1.5)
    assert acceleration == pytest.approx(6.)


def test_button_arm_speed_retimes_free_phases_and_respects_limits():
    labels = ['approach', 'approach', 'close', 'contact', 'press', 'retract', 'return', 'home']
    times = np.arange(len(labels), dtype=float) * 3.
    points = np.array([[x] * 7 for x in (0., .2, .3, .32, .34, .3, .2, 0.)])
    settings = SearchSettings(arm_speed_multiplier=5.)
    adjusted, spline, samples, metrics = retime_button_arm(times, points, labels, settings, {})
    assert 1. < metrics['arm_speed_multiplier_effective'] <= 5.
    np.testing.assert_allclose(np.diff(adjusted)[1:5], np.diff(times)[1:5])
    np.testing.assert_allclose(spline(adjusted), points, atol=1e-12)
    assert metrics['speed_limit_fraction'] <= .8 + 1e-9
    assert metrics['acceleration_limit_fraction'] <= .8 + 1e-9
    assert samples[-1] == pytest.approx(adjusted[-1])


def test_button_arm_speed_is_validated_and_passed_through_cli():
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    _, settings = parse(parser, ['--arm-speed-multiplier', '5'])
    assert settings.arm_speed_multiplier == 5.
    for invalid in (0, -1, 11, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='Invalid robust search settings'):
            SearchSettings(arm_speed_multiplier=invalid)


def test_retiming_preserves_geometry_and_reserves_twenty_percent():
    from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
    times = np.array([0., 10., 20., 30.])
    points = np.array([[0.]*7, [.5]*7, [-.2]*7, [0.]*7])
    settings = SearchSettings()
    new_times, spline, samples, metrics = retime_reference(times, points, settings)
    assert metrics['expected_motion_s'] == pytest.approx(30.)
    assert len(samples)*.02 == pytest.approx(metrics['trajectory_s'])
    np.testing.assert_allclose(np.diff(samples), .02, atol=1e-12)
    assert metrics['speed_limit_fraction'] <= .8+1e-12
    assert metrics['acceleration_limit_fraction'] <= .8+1e-12
    derivatives = PchipInterpolator(times, points, axis=0).derivative()(times)
    derivatives[0] = derivatives[-1] = 0
    original = CubicHermiteSpline(times, points, derivatives)
    fractions = np.linspace(0, 1, 100)
    np.testing.assert_allclose(spline(fractions*new_times[-1]), original(fractions*times[-1]), atol=1e-12)
    np.testing.assert_allclose(spline(new_times[-1]), points[-1], atol=1e-12)


def test_motion_budget_extends_only_within_tolerance_or_rejects():
    times = np.array([0., 1.])
    points = np.array([[0.]*7, [1.]*7])
    _, _, _, metrics = retime_reference(times, points, SearchSettings(joint_speed_rad_s=.067))
    assert 30 < metrics['expected_motion_s'] <= 33
    with pytest.raises(ValueError, match='headroom'):
        retime_reference(times, points, SearchSettings(joint_speed_rad_s=.01))


def test_runtime_filter_uses_higher_limits_and_caps_corrections():
    limits = JointMotionLimits(1.2, 4.)
    filter_ = JointCommandFilter(np.zeros(7), limits)
    previous = np.zeros(7)
    peak = 0.
    for _ in range(35):
        filter_.next_goal([1., 0, 0, 0, 0, 0, 0], .02)
        assert np.max(np.abs(filter_.velocity-previous)) <= 4*.02+1e-12
        assert np.max(np.abs(filter_.velocity)) <= 1.2+1e-12
        peak = max(peak, float(np.max(np.abs(filter_.velocity))))
        previous = filter_.velocity.copy()
    assert peak > .8


def test_short_bottle_motion_keeps_contact_stable_under_placement_error():
    import time
    from kinematics import load_robot
    from robust_scene import SceneSnapshot, build_scene_model
    from robust_planner import generate_candidate, physical_trial, uncertainty_scenarios
    path = ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not path.exists():
        pytest.skip('Menagerie asset not installed')
    _, q = load_robot(path, (.1, 0, 0), .79)
    scene = SceneSnapshot('bottle', (.326, -.12, .84), (1., 0., 0.), .034,
                          tuple(q), (0., 0., 0.), 0., base_height=.76, uncertainty_m=.001)
    kin = build_scene_model(path, dict(tip_offset_m=(.1, 0, 0), pelvis_height_m=.79), scene)
    settings = SearchSettings()
    candidate = generate_candidate(kin, scene, 0, (0, 0, 0), (0, 0), settings, time.perf_counter()+30)
    assert 27 <= candidate.motion_metrics['expected_motion_s'] <= 33
    assert candidate.motion_metrics['speed_limit_fraction'] <= .8
    assert candidate.motion_metrics['acceleration_limit_fraction'] <= .8
    trial = physical_trial(kin, scene, candidate, uncertainty_scenarios(settings, scene)[2], time.perf_counter()+30)
    assert trial['completed'] and trial['success'], trial


def test_arm_motion_negotiates_and_enforces_same_stream_limits():
    from align_button import ArmMotion
    class Arm:
        def configure_motion_limits(self, limits):
            self.limits = limits
            return limits.to_dict()
        def current_arm(self): return (0.,)*14
        def set_hold(self, current): pass
    limits = JointMotionLimits(1.2, 4.)
    arm = Arm()
    motion = ArmMotion(arm, {}, limits=limits)
    assert arm.limits == motion.limits == limits
    motion.target_velocity[0] = .86
    motion.stream([.018, 0, 0, 0, 0, 0, 0], velocity=[.9, 0, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError, match='上限'):
        motion.stream([.04, 0, 0, 0, 0, 0, 0], velocity=[1.1, 0, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError, match='上限'):
        motion.stream([.04, 0, 0, 0, 0, 0, 0], velocity=[1.3, 0, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(motion.current_target(), [.018, 0, 0, 0, 0, 0, 0])


@pytest.mark.parametrize('fail_enable', [False, True])
def test_execution_clock_includes_release_even_after_partial_enable(monkeypatch, fail_enable):
    import robust_execution as execution
    from robust_scene import SceneSnapshot
    scene = SceneSnapshot('button', (.36, -.12, 1.), (1., 0., 0.), .035,
                          (0.,)*29, (0.,)*3, 0.)
    candidate = SimpleNamespace(q=np.zeros((1, 7)), tau=np.zeros((1, 7)), press_start=0)
    plan = SimpleNamespace(scene=scene, candidate=candidate)
    monkeypatch.setattr(execution, 'build_scene_model', lambda *a: object())
    monkeypatch.setattr(execution, 'validate_start', lambda *a,**k: [])
    class Motion:
        clock = 0.
        release_error = None
        def wait(self, seconds): self.clock += seconds
        def enable(self):
            self.wait(ENABLE_DURATION_S)
            if fail_enable: raise RuntimeError('enable failed')
        def close(self): self.wait(RELEASE_DURATION_S)
    motion = Motion()
    result = execution.execute(plan, '', {}, motion, lambda: (scene.q, scene.imu, motion.clock, 0., 0.),
                               lambda: None, SearchSettings(), now=lambda: motion.clock, diagnostic=True)
    assert result['release_completed']
    assert result['release_s'] == pytest.approx(RELEASE_DURATION_S)
    assert result['motion_s'] == pytest.approx(ENABLE_DURATION_S+RELEASE_DURATION_S if fail_enable else MOTION_OVERHEAD_S)
    assert result['completed'] is (not fail_enable)
