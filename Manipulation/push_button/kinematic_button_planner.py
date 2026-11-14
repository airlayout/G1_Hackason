"""Opt-in button planning with static geometry checks and no physical rollout."""
from dataclasses import asdict
from itertools import product
import time

from robust_planner import PlanResult, generate_candidate
from robust_scene import build_scene_model


def search_kinematic(path, calibration, scene, settings, *, monitor=lambda: None):
    if scene.task != 'button' or scene.button_policy != 'collection' or settings.button_policy != 'collection' or settings.scenarios != 1 or settings.button_feedback != 'joint':
        raise ValueError('Kinematic prototype requires a collection button with joint feedback and one nominal scene')
    started = time.perf_counter()
    deadline = started + settings.budget_s
    plan = PlanResult(scene)
    report = plan.report
    report.update(schema='g1-kinematic-button-plan-v1', validation_method='kinematic',
        physical_simulation_performed=False, settings=asdict(settings), candidates=[],
        candidates_attempted=0, physically_evaluated_candidates=0, condition_trials_started=0,
        completed_rollouts=0, aborted_rollouts=0, accepted_conditions=0,
        static_checks=0, first_accepted_s=None, stages_wall_s={},
        limitations=['no dynamics, tracking, contact force, press depth, balance or physical return validation'])
    t = time.perf_counter()
    kin = build_scene_model(path, calibration, scene)
    report['stages_wall_s']['scene'] = time.perf_counter()-t
    offsets = [(0.,0.,0.)]+[v for v in product((-.4,0.,.4),(-.4,0.,.4),(-.6,0.,.6)) if v != (0.,0.,0.)]
    t = time.perf_counter()
    for index in range(settings.max_candidates):
        monitor()
        if time.perf_counter() >= deadline:
            break
        row = {'index': index, 'accepted': False}
        report['candidates'].append(row)
        report['candidates_attempted'] += 1
        report['static_checks'] += 1
        original = kin.data.qpos.copy()
        try:
            waypoint = [(0.,0.),(-.04,.04),(.04,.04),(0.,.08)][index % 4]
            candidate = generate_candidate(kin, scene, index, offsets[index % len(offsets)], waypoint, settings, deadline)
            if time.perf_counter() >= deadline:
                row['reason'] = 'deadline'
                break
            monitor()
        except (ValueError, TimeoutError) as exc:
            row['reason'] = str(exc)
            continue
        finally:
            kin.update(original)
        row.update(accepted=True, min_clearance_m=float(candidate.min_clearance_m),
                   min_joint_margin_rad=float(candidate.min_joint_margin_rad),
                   duration_s=float(candidate.duration_s))
        plan.candidate = candidate
        report['first_accepted_s'] = time.perf_counter()-started
        report['kinematic_checked'] = True
        break
    report['stages_wall_s']['candidate_and_static'] = time.perf_counter()-t
    report['total_wall_s'] = time.perf_counter()-started
    if plan.candidate is None:
        report['kinematic_checked'] = False
        report['reason'] = 'No candidate passed kinematic and collision checks within budget'
    return plan
