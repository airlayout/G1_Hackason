"""ボタン用の経路事前検査。方向を固定の+Xとせず、計測したパネル法線を使う。"""
from itertools import product

import numpy as np

from vision import validate_reachable_target

STAGED_CLEARANCES_M = (.10, .07, .05, .03)


def validate_fk_margin(calibration, policy):
    margin = (float(calibration['button_radius_m']) - float(calibration['tip_radius_m'])
              - float(calibration['position_uncertainty_m']))
    if policy.fk_tolerance_m > margin + 1e-12:
        raise ValueError(f'FK許容誤差がボタンと手先・校正の余裕({margin*1000:.1f}mm)を超えます')
    return margin


def solve_checked(kin, xyz, face, normal, *, contact=False, searches=None):
    """数値初期値を変えても、毎回実際の現在姿勢から経路を検査する。"""
    contact = bool(contact)
    original = kin.data.qpos.copy()
    row = {'target_xyz_m': np.asarray(xyz).tolist(), 'allow_tip_contact': contact,
           'attempts': [], 'accepted': False}
    if searches is not None:
        searches.append(row)
    offsets = [(0., 0., 0.)] + [o for o in product((-.4, 0., .4), (-.4, 0., .4), (-.6, 0., .6))
                                  if o != (0., 0., 0.)]
    try:
        for offset in offsets:
            attempt = {'shoulder_seed_offset_rad': list(offset)}
            row['attempts'].append(attempt)
            try:
                if offset == (0., 0., 0.):
                    solution = kin.solve(np.asarray(xyz), normal)
                else:
                    seed = original[kin.qadr].copy()
                    seed[:3] += offset
                    solution = kin.solve(np.asarray(xyz), normal, seed=seed)
                kin.update(original)
                kin.validate_path(solution.joint_angles, face, normal, contact)
            except ValueError as exc:
                attempt['reason'] = str(exc)
            else:
                attempt['accepted'] = row['accepted'] = True
                return solution
            finally:
                kin.update(original)
    finally:
        kin.update(original)
    raise ValueError(f'経路検査付きIKが{len(offsets)}候補で失敗しました: {row["attempts"][-1]["reason"]}')


def preview_button_path(kin, observation, radius, args, policy, *, clearances=None, searches=None):
    """接近から押下・退避・往路逆再生までを検査し、モデル状態は復元する。"""
    validate_reachable_target(observation.target)
    face = np.array(observation.target.face_xyz)
    normal = np.array(observation.target.press_direction)
    if clearances is None:
        clearances = (STAGED_CLEARANCES_M if policy.approach_mode == 'staged' else
                      ((.09, args.clearance) if args.alignment_mode == 'single' else (args.clearance,)))
    original = kin.data.qpos.copy()
    poses, approach = {}, [original[kin.qadr].copy()]

    def plan(name, distance, contact=False):
        goal = solve_checked(kin, face+(distance-radius)*normal, face, normal,
                             contact=contact, searches=searches).joint_angles
        poses[name] = goal.tolist()
        q = kin.data.qpos.copy()
        q[kin.qadr] = goal
        kin.update(q)
        return goal

    try:
        for index, distance in enumerate(clearances, 1):
            approach.append(plan(f'approach_{index}', -distance))
        if not getattr(args, 'stop_before_contact', False):
            # 接触直前〜終点は5mm以下の分割で通す。
            distances = np.linspace(-.003, args.stroke, max(2, int(np.ceil((args.stroke+.003)/.005))+1))
            for index, distance in enumerate(distances):
                plan('contact' if index == 0 else f'press_{index}', distance, distance >= 0)
            plan('retract', -args.clearance, True)
        for goal in reversed(approach):
            kin.validate_path(goal, face, normal)
            q = kin.data.qpos.copy()
            q[kin.qadr] = goal
            kin.update(q)
    finally:
        kin.update(original)
    return poses
