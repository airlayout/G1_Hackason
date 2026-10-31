"""Bounded IK search and scene options, with no renderer or physics execution."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[1]/'sim')]
from bottle import BottlePushRoutine, BottleSettings
from bottle_layout import project_bottle_bounds, project_table_edges, validate_layout_xy
from run_realistic_bottle import bottle_world_xy, build_argument_parser, main


class FakeKin:
    qadr = np.arange(22, 29)

    def __init__(self, *, fail=False):
        self.data = SimpleNamespace(qpos=np.arange(29)*.01)
        self.original = self.data.qpos.copy()
        self.fail = fail
        self.calls, self.paths = [], []

    def update(self, q):
        self.data.qpos = q.copy()

    def solve(self, target, normal, *, seed=None):
        self.calls.append(None if seed is None else seed.copy())
        # Like the real solver, it may change internal FK while computing.
        self.data.qpos[:] = -1
        joints = np.full(7, 1. if seed is None else 2.)
        return SimpleNamespace(joint_angles=joints)

    def validate_path(self, goal, face, normal, contact):
        np.testing.assert_array_equal(self.data.qpos, self.original)
        self.paths.append((goal.copy(), contact))
        if self.fail or goal[0] == 1.:
            raise ValueError('collision')


def routine(kin, motion=None, refresh=lambda: None):
    return BottlePushRoutine(kin, motion, None, refresh, lambda: 0., BottleSettings(.75), .015)


def test_seed_is_not_a_waypoint_and_path_always_starts_at_actual_pose():
    kin = FakeKin()
    r = routine(kin)
    solution = r.solve_pose([.3, -.2, .83], [.34, -.2, .83], contact=True)
    assert len(kin.calls) == len(kin.paths) == 2
    np.testing.assert_allclose(kin.calls[1]-kin.original[kin.qadr], [-.4, -.4, -.6, 0, 0, 0, 0])
    assert solution.joint_angles[0] == 2.
    assert all(contact for _, contact in kin.paths)
    np.testing.assert_array_equal(kin.data.qpos, kin.original)
    assert r.report['ik_searches'][0]['attempts'][0]['reason'] == 'collision'
    assert r.report['ik_searches'][0]['accepted']


def test_exhausted_search_is_bounded_restores_state_and_never_moves():
    kin = FakeKin(fail=True)
    motion = SimpleNamespace(move=lambda *a, **k: pytest.fail('must not command failed plan'))
    r = routine(kin, motion)
    with pytest.raises(ValueError, match='27候補'):
        r.move([.3, -.2, .83], [.34, -.2, .83])
    assert len(kin.calls) == 27
    np.testing.assert_array_equal(kin.data.qpos, kin.original)


def test_monitor_failure_aborts_without_searching_other_seeds():
    kin = FakeKin()
    def disconnected(*a, **k):
        raise RuntimeError('state disconnected')
    kin.solve = disconnected
    with pytest.raises(RuntimeError, match='disconnected'):
        routine(kin).solve_pose([.3, -.2, .83], [.34, -.2, .83])
    np.testing.assert_array_equal(kin.data.qpos, kin.original)


def test_move_revalidates_after_refresh_before_publishing():
    kin = FakeKin()
    refresh_count = []
    def refresh():
        refresh_count.append(1)
        if len(refresh_count) == 2:
            kin.fail = True
    motion = SimpleNamespace(move=lambda *a, **k: pytest.fail('fresh path is blocked'))
    with pytest.raises(ValueError, match='collision'):
        routine(kin, motion, refresh).move([.3, -.2, .83], [.34, -.2, .83])
    assert len(refresh_count) == 2


def test_layout_arguments_preserve_defaults_and_stance_perturbations():
    parser = build_argument_parser()
    defaults = parser.parse_args(['--out', '/tmp/unused'])
    assert (defaults.bottle_x, defaults.bottle_y) == (.33, -.25)
    args = parser.parse_args(['--out', '/tmp/unused', '--bottle-x', '.36', '--bottle-y', '-.13'])
    r = SimpleNamespace(stance_yaw=np.pi/2, stance_xy=np.array([.01, -.02]))
    assert bottle_world_xy(args, r) == pytest.approx((-.12, -.38))
    args.fault = 'unreachable'
    assert bottle_world_xy(args, r) == pytest.approx((.45, -.25))


@pytest.mark.parametrize('x,y', [(np.nan, -.2), (.33, np.inf), (.7, -.2), (.33, -.01)])
def test_invalid_layout_is_rejected_before_rendering_or_output_creation(tmp_path, x, y):
    with pytest.raises(SystemExit) as exc:
        main(['--out', str(tmp_path/'unused'), '--bottle-x', str(x), '--bottle-y', str(y)])
    assert exc.value.code == 2 and not (tmp_path/'unused').exists()


def test_configured_layout_uses_existing_travel_limits():
    target = validate_layout_xy(.36, -.13, BottleSettings(.75))
    assert target.face_xyz == pytest.approx((.326, -.13, .83))


def test_visibility_checks_whole_bottle_and_post_push_position():
    intr = SimpleNamespace(width=640, height=480, fx=500., fy=500., cx=319.5, cy=239.5)
    transform = np.array([[0., 0., 1., 0.], [-1., 0., 0., -.25], [0., -1., 0., .9], [0., 0., 0., 1.]])
    view = project_bottle_bounds((.37, -.25), BottleSettings(.75), transform, intr)
    assert view['in_front'] and view['margin_px'] > 0
    clipped = project_bottle_bounds((.37, -.5), BottleSettings(.75), transform, intr)
    assert clipped['margin_px'] < 0
    assert project_bottle_bounds((-.37, -.25), BottleSettings(.75), transform, intr)['in_front'] is False


def test_static_table_visibility_checks_one_edge_per_axis():
    intr = SimpleNamespace(width=640, height=480, fx=500., fy=500., cx=319.5, cy=239.5)
    transform = np.diag([1., -1., -1., 1.])
    transform[:3, 3] = [.525, -.29, 1.5]
    edges = project_table_edges(BottleSettings(.75), transform, intr)
    assert all(edge['visible'] for edge in edges.values())
    assert edges['x_max']['visible_span_m'] == pytest.approx(.62)
    transform[1, 3] = 10
    assert not any(edge['visible'] for edge in project_table_edges(BottleSettings(.75), transform, intr).values())


def test_explicit_ik_seed_is_clipped_and_fk_state_is_restored(monkeypatch):
    import kinematics
    from trajectory import ARM_JOINTS
    kin = FakeKin()
    kin.model = SimpleNamespace(jnt_range=np.tile([-1., 1.], (7, 1)),
        joint=lambda name: SimpleNamespace(id=ARM_JOINTS.index(name)))
    seeds = []
    def solve(model, data, target, seed, rest, qadr, dadr, normal, position_tolerance_m):
        seeds.append(seed.copy())
        data.qpos[:] = 123.
        return SimpleNamespace(joint_angles=np.zeros(7))
    monkeypatch.setattr(kinematics, 'solve_ik', solve)
    kin.dadr = kin.qadr
    kinematics.ArmKinematics.solve(kin, np.zeros(3), np.ones(3), seed=np.full(7, 5.))
    np.testing.assert_allclose(seeds[0], .975)
    np.testing.assert_array_equal(kin.data.qpos, kin.original)
    with pytest.raises(ValueError, match='初期値'):
        kinematics.ArmKinematics.solve(kin, np.zeros(3), np.ones(3), seed=[np.nan]*7)


def test_layout_only_mode_never_constructs_the_dynamic_world(monkeypatch, tmp_path):
    import run_realistic_bottle as harness
    monkeypatch.setattr(harness, 'World', lambda *a: pytest.fail('physics must not start'))
    monkeypatch.setattr(harness, 'check_layouts', lambda args: {'static_only': True, 'ready': False})
    assert main(['--out', str(tmp_path/'static'), '--check-layout']) == 1
    assert (tmp_path/'static/layout_checks.json').is_file()


@pytest.mark.parametrize('flag', ['--video', '--stop-before-contact', '--fault=target-loss'])
def test_layout_diagnostic_does_not_accept_dynamic_modes(tmp_path, flag):
    with pytest.raises(SystemExit) as exc:
        main(['--out', str(tmp_path/'unused'), '--check-layout', flag])
    assert exc.value.code == 2
