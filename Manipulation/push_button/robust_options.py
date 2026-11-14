"""Explicit opt-in; legacy parsers and their default behaviour remain unchanged."""
import argparse
import json
import sys
from robust_conditions import BUTTON_CONDITION_COUNT,BUTTON_POSITION_SPAN_M

BUTTON_DEFAULTS = dict(stroke=.015, planning_budget_s=30., planning_max_candidates=50,
    planning_scenarios=BUTTON_CONDITION_COUNT,planning_position_span_m=BUTTON_POSITION_SPAN_M)

def selected(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument('--motion-planner', choices=('legacy', 'robust'))
    pre.add_argument('--trial-config')
    args, _ = pre.parse_known_args(argv)
    if args.motion_planner is not None:
        return args.motion_planner == 'robust'
    try:
        with open(args.trial_config) as f:
            values = json.load(f)
    except (OSError,TypeError,ValueError):
        values = {}  # The selected legacy parser reports invalid configuration.
    if not isinstance(values,dict):
        values = {}
    return (args.motion_planner or values.get('motion_planner','legacy')) == 'robust'


def add_arguments(parser):
    parser.add_argument('--motion-planner', choices=('legacy','robust'), default='legacy')
    parser.add_argument('--trial-config')
    parser.add_argument('--planning-budget-s', type=float, default=10.)
    parser.add_argument('--planning-max-rollouts', type=int, default=1000)
    parser.add_argument('--planning-scenarios', type=int, default=3)
    parser.add_argument('--planning-position-span-m', type=float, default=.003)
    parser.add_argument('--button-feedback', choices=('off','joint'), default='off')
    parser.add_argument('--button-policy', choices=('standard','collection'), default='standard')
    parser.add_argument('--button-feedback-timeout-s', type=float, default=120.)
    parser.add_argument('--bottle-feedback', choices=('off','relative'), default='off')
    parser.add_argument('--planning-workers', type=int, default=8)
    parser.add_argument('--planning-max-candidates', type=int, default=50)
    parser.add_argument('--planning-seed', type=int, default=0)
    parser.add_argument('--position-error-m', type=float, default=.001)
    parser.add_argument('--normal-error-deg', type=float, default=1.)
    parser.add_argument('--tracking-error-rad', type=float, default=.001)
    parser.add_argument('--mass-range', type=float, nargs=2, default=(.28,.42))
    parser.add_argument('--friction-range', type=float, nargs=2, default=(.24,.36))
    parser.add_argument('--bottle-base-height', type=float)
    parser.add_argument('--contact-height', type=float, default=.08)
    parser.add_argument('--stroke', type=float, default=.0015)
    parser.add_argument('--push-distance', type=float, default=.05)
    parser.add_argument('--gravity-scale', type=float, default=1.)
    parser.add_argument('--arm-speed-multiplier', type=float, default=1.)
    parser.add_argument('--motion-duration-s', type=float, default=30.)
    parser.add_argument('--joint-speed-rad-s', type=float, default=1.2)
    parser.add_argument('--joint-acceleration-rad-s2', type=float, default=4.)


def parse(parser, argv=None, *, task=None):
    from robust_planner import SearchSettings
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument('--trial-config')
    pre.add_argument('--task')
    initial, _ = pre.parse_known_args(argv)
    values = {}
    if initial.trial_config:
        with open(initial.trial_config) as f:
            values = json.load(f)
        if not isinstance(values, dict):
            parser.error('trial-config must contain an object')
        allowed = {a.dest for a in parser._actions}
        if set(values)-allowed:
            parser.error('Unknown robust settings: '+str(sorted(set(values)-allowed)))
    effective_task = task or initial.task or values.get('task',parser.get_default('task'))
    if effective_task == 'button':
        parser.set_defaults(**BUTTON_DEFAULTS)
    if initial.trial_config:
        parser.set_defaults(**values)
        for action in parser._actions:
            if action.dest in values:
                action.required = False
    args = parser.parse_args(argv)
    for action in parser._actions:
        if (action.choices is not None and getattr(args,action.dest,None) is not None
                and getattr(args,action.dest) not in action.choices):
            parser.error(f'Invalid {action.dest}')
    settings = SearchSettings(budget_s=args.planning_budget_s, max_rollouts=args.planning_max_rollouts,
        scenarios=args.planning_scenarios, position_span_m=args.planning_position_span_m,
        button_feedback=args.button_feedback, button_policy=args.button_policy,
        button_feedback_timeout_s=args.button_feedback_timeout_s,
        bottle_feedback=args.bottle_feedback, workers=args.planning_workers,
        max_candidates=args.planning_max_candidates, seed=args.planning_seed,
        position_error_m=args.position_error_m, normal_error_deg=args.normal_error_deg,
        tracking_error_rad=args.tracking_error_rad, mass_range=tuple(args.mass_range),
        friction_range=tuple(args.friction_range), gravity_scale=args.gravity_scale,
        motion_duration_s=args.motion_duration_s, arm_speed_multiplier=args.arm_speed_multiplier,
        joint_speed_rad_s=args.joint_speed_rad_s,
        joint_acceleration_rad_s2=args.joint_acceleration_rad_s2)
    return args, settings
