"""Offline timing only. Does not import the SDK or connect to any robot/camera."""
import argparse
import cProfile
from dataclasses import asdict, replace
import json
from pathlib import Path
import pstats
import sys
import time


def summarize_stats(profile, output):
    profile.dump_stats(str(output))
    stats = pstats.Stats(profile).stats
    rows, categories = [], {}
    for (filename, line, name), (primitive, calls, exclusive, cumulative, callers) in stats.items():
        rows.append(dict(file=filename, line=line, function=name, calls=calls,
                         exclusive_s=exclusive, cumulative_s=cumulative))
        if 'rollout' in name or filename.endswith('/mujoco/rollout.py'):
            category = 'physics_rollout'
        elif 'mj_geomDistance' in name:
            category = 'collision_distance'
        elif 'mj_fwdPosition' in name:
            category = 'forward_position_and_collision'
        elif 'mj_forward' in name:
            category = 'full_forward_dynamics'
        elif '__copy__' in name or filename.endswith('/copy.py'):
            category = 'copy_model_and_data'
        elif 'lstsq' in name or 'svd' in name:
            category = 'linear_solve'
        elif 'slsqp' in filename.lower():
            category = 'trajectory_time_optimization'
        else:
            category = 'other_python_numpy_and_wrappers'
        categories[category] = categories.get(category, 0.)+exclusive
    return dict(exclusive_categories_s=categories,
                total_profiled_s=sum(v['exclusive_s'] for v in rows),
                top_cumulative=sorted(rows, key=lambda v:v['cumulative_s'], reverse=True)[:35],
                top_exclusive=sorted(rows, key=lambda v:v['exclusive_s'], reverse=True)[:35])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['search', 'detail', 'feedback-detail'])
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    session = Path(__file__).resolve().parent
    output = Path(json.loads((session/'performance_current.json').read_text())['directory'])
    root = session.parents[2]
    source = root/'Manipulation/push_button' if args.mode == 'feedback-detail' else output/'baseline/Manipulation/push_button'
    sys.path[:0] = [str(source), str(source/'real')]
    import numpy as np
    import robust_planner as planner
    from robust_scene import SceneSnapshot, build_scene_model

    saved = json.loads((session/'diagnostic_20261101_112438/result.json').read_text())
    scene = SceneSnapshot(**saved['plan']['scene'])
    calibration, model_path = saved['calibration'], saved['arguments']['model']
    settings = planner.SearchSettings(**saved['settings'])
    settings = replace(settings, workers=args.workers)
    result = dict(mode=args.mode, source=str(source), settings=asdict(settings),
                  robot_connected=False)
    if args.mode == 'search':
        started = time.perf_counter()
        plan = planner.search(model_path, calibration, scene, settings)
        result.update(started_s=started, elapsed_s=time.perf_counter()-started, plan=plan.to_dict())
        for candidate in result['plan'].get('candidates', []):
            submitted = candidate.get('profile_submitted_at_s')
            profiles = [v['profile'] for v in candidate.get('trials', []) if 'profile' in v]
            if submitted and profiles:
                candidate['dispatch_summary'] = dict(
                    first_task_start_after_submit_s=min(v['started_s'] for v in profiles)-submitted,
                    last_task_end_after_submit_s=max(v['finished_s'] for v in profiles)-submitted,
                    worker_compute_sum_s=sum(v['finished_s']-v['started_s'] for v in profiles),
                    workers=len({v['pid'] for v in profiles}))
        destination = output/f'baseline_search_{args.workers}workers.json'
    else:
        started = time.perf_counter()
        kin = build_scene_model(model_path, calibration, scene)
        result['model_build_s'] = time.perf_counter()-started
        if args.mode == 'detail':
            started = time.perf_counter()
            candidate = planner.generate_candidate(kin, scene, 0, (0,0,0), (0,0), settings,
                                                    time.perf_counter()+30)
            result['candidate_unprofiled_s'] = time.perf_counter()-started
            arrays = {key:getattr(candidate,key) for key in ['q','tcp','tau']}
            np.savez_compressed(output/'baseline_candidate.npz', **arrays)
            metadata = {key:getattr(candidate,key) for key in ['index','phases','press_start','press_end',
                        'retract_end','duration_s','min_joint_margin_rad','min_clearance_m','motion_metrics']}
            (output/'baseline_candidate.json').write_text(json.dumps(metadata,indent=2)+'\n')
            profile = cProfile.Profile()
            original = kin.data.qpos.copy()
            profile.enable()
            planner.generate_candidate(kin,scene,0,(0,0,0),(0,0),settings,time.perf_counter()+60)
            profile.disable()
            kin.update(original)
            result['candidate_profile'] = summarize_stats(profile,output/'baseline_candidate.pstats')
        else:
            metadata = json.loads((output/'baseline_candidate.json').read_text())
            arrays = np.load(output/'baseline_candidate.npz')
            candidate = planner.Candidate(**metadata, **{key:arrays[key] for key in ['q','tcp','tau']})
            candidate.motion_metrics['bottle_feedback'] = dict(observation_noise_m=settings.position_error_m,
                                                              gravity_scale=settings.gravity_scale)
        condition = planner.uncertainty_scenarios(settings,scene)[0]
        started = time.perf_counter()
        trial = planner.physical_trial(kin, scene, candidate, condition, time.perf_counter()+60)
        result['trial_unprofiled_s'] = time.perf_counter()-started
        result['trial_unprofiled'] = trial
        profile = cProfile.Profile()
        profile.enable()
        trial = planner.physical_trial(kin, scene, candidate, condition, time.perf_counter()+60)
        profile.disable()
        result['trial_profiled'] = trial
        result['trial_profile'] = summarize_stats(profile,output/f'{args.mode}_trial.pstats')
        result['workload'] = dict(control_samples=len(candidate.q),
            simulated_trajectory_s=candidate.duration_s,physics_step_s=.002,
            expected_physics_steps=len(candidate.q)*10,
            robot_dof=kin.model.nv,geometry_count=kin.model.ngeom)
        destination = output/f'{args.mode}.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(destination), elapsed_s=result.get('elapsed_s'),
                         trial_unprofiled_s=result.get('trial_unprofiled_s'),
                         candidate_unprofiled_s=result.get('candidate_unprofiled_s'),
                         completed_rollouts=result.get('plan',{}).get('completed_rollouts')),
                     ensure_ascii=False),flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
