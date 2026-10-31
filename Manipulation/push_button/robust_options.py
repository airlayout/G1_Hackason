"""Explicit opt-in; legacy parsers and their default behaviour remain unchanged."""
import argparse
import json
import sys



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
    parser.add_argument('--planning-scenarios', type=int, default=20)
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


def parse(parser, argv=None):
    from robust_planner import SearchSettings
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument('--trial-config')
    initial, _ = pre.parse_known_args(argv)
    if initial.trial_config:
        with open(initial.trial_config) as f:
            values = json.load(f)
        if not isinstance(values, dict):
            parser.error('trial-config must contain an object')
        allowed = {a.dest for a in parser._actions}
        if set(values)-allowed:
            parser.error('Unknown robust settings: '+str(sorted(set(values)-allowed)))
        parser.set_defaults(**values)
        for action in parser._actions:
            if action.dest in values:
                action.required = False
    args = parser.parse_args(argv)
    for action in parser._actions:
        if action.choices is not None and getattr(args,action.dest,None) not in action.choices:
            parser.error(f'Invalid {action.dest}')
    settings = SearchSettings(budget_s=args.planning_budget_s, max_rollouts=args.planning_max_rollouts,
        scenarios=args.planning_scenarios, workers=args.planning_workers,
        max_candidates=args.planning_max_candidates, seed=args.planning_seed,
        position_error_m=args.position_error_m, normal_error_deg=args.normal_error_deg,
        tracking_error_rad=args.tracking_error_rad, mass_range=tuple(args.mass_range),
        friction_range=tuple(args.friction_range), gravity_scale=args.gravity_scale)
    return args, settings
