"""ボトルの現場試行設定。カメラ校正と制御の異常終了条件とは分離する。"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import json
import math
from pathlib import Path
import sys


class TrialPause(RuntimeError):
    def __init__(self, reason: str, *, code: str = 'measurement', automatic: bool = False):
        super().__init__(reason)
        self.code, self.automatic = code, automatic


class ObservationUnavailable(RuntimeError):
    """新しい観測を待つ必要がある。通信・指令の異常とは区別する。"""


@dataclass(frozen=True)
class TrialPolicy:
    profile: str = 'strict'
    table_edge_tolerance: float = .004
    table_x_edge: str = 'auto'
    table_y_edge: str = 'auto'
    position_stability_m: float = .004
    waist_delta_deg: float = math.degrees(.025)
    imu_delta_deg: float = 2.
    leg_speed_rad_s: float = .15
    gyro_rad_s: float = .10
    motion_persistence_s: float = .10
    stable_hold_s: float = .30
    record_rgbd: bool = False

    @property
    def field(self) -> bool:
        return self.profile == 'field'

    def __post_init__(self):
        if self.profile not in ('strict', 'field'):
            raise ValueError('profileはstrictまたはfieldです')
        if not .004 <= self.table_edge_tolerance <= .015:
            raise ValueError('台の対辺許容差は4〜15mmが必要です')
        if self.table_x_edge not in ('auto', 'x_min', 'x_max') or self.table_y_edge not in ('auto', 'y_min', 'y_max'):
            raise ValueError('台の基準辺が不正です')
        if self.field and 'auto' in (self.table_x_edge, self.table_y_edge):
            raise ValueError('fieldでは台のX辺とY辺を固定してください')
        if (self.table_x_edge == 'auto') != (self.table_y_edge == 'auto'):
            raise ValueError('台のX辺・Y辺は両方とも指定してください')
        limits = ((self.position_stability_m, .001, .02), (self.waist_delta_deg, .1, 10.),
                  (self.imu_delta_deg, .1, 10.), (self.leg_speed_rad_s, .01, 1.),
                  (self.gyro_rad_s, .01, 1.), (self.motion_persistence_s, .02, 1.),
                  (self.stable_hold_s, .1, 2.))
        if any(not math.isfinite(v) or not lo <= v <= hi for v, lo, hi in limits):
            raise ValueError('試行の許容幅・監視時間が範囲外です')
        if type(self.record_rgbd) is not bool:
            raise ValueError('record_rgbdは真偽値が必要です')
        if not self.field and (self.waist_delta_deg != math.degrees(.025) or self.imu_delta_deg != 2.
                or self.leg_speed_rad_s != .15 or self.gyro_rad_s != .1
                or self.motion_persistence_s != .1 or self.stable_hold_s != .3):
            raise ValueError('姿勢・速度監視の調整には--profile fieldを指定してください')


def add_trial_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--profile', choices=('strict', 'field'), default='strict')
    parser.add_argument('--trial-config', help='現場の設定を保存したJSON。CLI指定を優先する')
    parser.add_argument('--table-x-edge', choices=('auto', 'x_min', 'x_max'))
    parser.add_argument('--table-y-edge', choices=('auto', 'y_min', 'y_max'))
    parser.add_argument('--position-stability-m', type=float)
    parser.add_argument('--waist-delta-deg', type=float)
    parser.add_argument('--imu-delta-deg', type=float)
    parser.add_argument('--leg-speed-rad-s', type=float)
    parser.add_argument('--gyro-rad-s', type=float)
    parser.add_argument('--motion-persistence-s', type=float)
    parser.add_argument('--stable-hold-s', type=float)
    parser.add_argument('--record-rgbd', action=argparse.BooleanOptionalAction, default=None)


def resolve_trial_arguments(parser, argv=None):
    """CLI > JSON > プロファイル。未指定のCLI既定値でJSONを上書きしない。"""
    raw = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument('--profile', choices=('strict', 'field'), default='strict')
    pre.add_argument('--trial-config')
    probe, _ = pre.parse_known_args(raw)
    values = {}
    if probe.trial_config:
        values = json.loads(Path(probe.trial_config).read_text())
        if not isinstance(values, dict):
            raise ValueError('trial-configはJSONオブジェクトが必要です')
    explicit = {token.split('=', 1)[0] for token in raw if token.startswith('--')}
    profile = probe.profile if '--profile' in explicit else values.get('profile', probe.profile)
    preset = asdict(TrialPolicy())
    if profile == 'field':
        preset.update(profile='field', table_edge_tolerance=.015, table_x_edge='x_min', table_y_edge='y_min',
                      position_stability_m=.008, waist_delta_deg=3., imu_delta_deg=3., record_rgbd=True)
    elif profile != 'strict':
        raise ValueError('profileはstrictまたはfieldです')
    configurable = set(preset) | {'push_distance', 'table_height', 'table_bounds', 'contact_height',
        'clearance', 'target_pixel', 'track_table', 'gravity_scale', 'dynamics_feedforward',
        'confidence', 'detector_imgsz', 'device', 'weights', 'camera_config', 'host'}
    unknown = set(values)-configurable
    if unknown:
        raise ValueError(f'trial-configの未知の項目: {sorted(unknown)}')
    for action in parser._actions:
        if action.dest not in values or action.dest == 'profile':
            continue
        value = values[action.dest]
        if action.type is not None:
            if action.nargs is not None:
                if not isinstance(value, list) or len(value) != action.nargs:
                    raise ValueError(f'{action.dest}の要素数が不正です')
                values[action.dest] = [action.type(item) for item in value]
            else:
                if isinstance(value, bool):
                    raise ValueError(f'{action.dest}は数値が必要です')
                values[action.dest] = action.type(value)
        elif isinstance(action, (argparse._StoreTrueAction, argparse.BooleanOptionalAction)):
            if type(value) is not bool:
                raise ValueError(f'{action.dest}は真偽値が必要です')
        if action.choices is not None and values[action.dest] not in action.choices:
            raise ValueError(f'{action.dest}の値が不正です')
    defaults = {**preset, 'push_distance': .02 if profile == 'field' else .05,
                'track_table': False, **values, 'profile': profile}
    # JSONで台高を指定できる。指定必須のままの引数にはset_defaultsだけでは効かない。
    original = [(action, action.required, action.default) for action in parser._actions]
    parser_defaults = parser._defaults.copy()
    for action in parser._actions:
        if action.dest in defaults:
            action.required = False
    parser.set_defaults(**defaults)
    try:
        args = parser.parse_args(raw)
    finally:
        for action, required, default in original:
            action.required, action.default = required, default
        parser._defaults = parser_defaults
    policy = TrialPolicy(**{key: getattr(args, key) for key in preset})
    return args, policy


def trial_exit_code(report: dict, policy: TrialPolicy, diagnostic: bool = False) -> int:
    if policy.field and (report.get('fatal_error') or report.get('release_error')):
        return 2
    if diagnostic:
        return 0 if report.get('diagnostic_completed') and report.get('home_completed') and report.get('release_completed') else 1
    complete = all(report.get(key) for key in ('completed', 'motion_completed', 'home_completed', 'release_completed'))
    if policy.field:
        return 0 if complete and not report.get('error') else 1
    return 0 if report.get('completed') and report.get('visual_success') is True else 1
