"""Wall-clock attribution for one offline rollout, with low-overhead scoped timers."""
import json
from pathlib import Path
import sys
import time


def main():
    session = Path(__file__).resolve().parent
    output = Path(json.loads((session/'performance_current.json').read_text())['directory'])
    mode = sys.argv[1]
    root = session.parents[2]
    source = (output/'baseline/Manipulation/push_button' if mode == 'baseline'
              else root/'Manipulation/push_button')
    sys.path[:0] = [str(source), str(source/'real')]
    import numpy as np
    import mujoco
    import robust_planner as planner
    from robust_scene import SceneSnapshot, build_scene_model
    from kinematics import ArmKinematics

    saved = json.loads((session/'diagnostic_20261101_112438/result.json').read_text())
    scene = SceneSnapshot(**saved['plan']['scene'])
    settings = planner.SearchSettings(**saved['settings'])
    kin = build_scene_model(saved['arguments']['model'],saved['calibration'],scene)
    arrays = np.load(output/'baseline_candidate.npz')
    metadata = json.loads((output/'baseline_candidate.json').read_text())
    candidate = planner.Candidate(**metadata, **{key:arrays[key] for key in ['q','tcp','tau']})
    if mode != 'baseline':
        candidate.motion_metrics['bottle_feedback'] = dict(observation_noise_m=settings.position_error_m,
                                                          gravity_scale=settings.gravity_scale)
    timers = {}
    originals = []

    def instrument(namespace, attribute, label):
        original = getattr(namespace, attribute)
        originals.append((namespace, attribute, original))
        def call(*args, **kwargs):
            started = time.perf_counter()
            try:
                return original(*args, **kwargs)
            finally:
                value = timers.setdefault(label, dict(calls=0, wall_s=0.))
                value['calls'] += 1
                value['wall_s'] += time.perf_counter()-started
        setattr(namespace, attribute, call)

    for namespace, attribute, label in [
        (planner,'clone_kin','copy_model_and_data'),
        (planner.rollout,'rollout','physics_integration'),
        (planner,'contact_metrics','collision_clearance_checks'),
        (mujoco,'mj_fwdPosition','forward_position_checks'),
        (mujoco,'mj_geomDistance','geometry_distance_subcalls'),
        (mujoco,'mj_forward','full_forward_subcalls'),
        (planner,'allocate_bottle_phases','duration_optimization'),
        (planner,'retime_reference','retime_spline'),
        (ArmKinematics,'solve','inverse_kinematics'),
        (ArmKinematics,'validate_path','path_validation'),
    ]:
        instrument(namespace,attribute,label)
    if mode != 'baseline':
        instrument(planner,'reference_command','feedback_command_generation')
    started = time.perf_counter()
    try:
        trial = planner.physical_trial(kin,scene,candidate,planner.uncertainty_scenarios(settings,scene)[0],
                                       time.perf_counter()+30)
        elapsed = time.perf_counter()-started
        nonoverlap = ['copy_model_and_data','physics_integration','collision_clearance_checks',
                      'forward_position_checks','feedback_command_generation']
        accounted = sum(timers.get(k,{}).get('wall_s',0) for k in nonoverlap)
        result = dict(mode=mode, source=str(source), total_wall_s=elapsed, trial=trial,
                      timers=timers.copy(), other_setup_python_metrics_and_cleanup_s=elapsed-accounted)
        if mode == 'baseline':
            timers.clear()
            started = time.perf_counter()
            planner.generate_candidate(kin,scene,0,(0,0,0),(0,0),settings,time.perf_counter()+30)
            result['candidate_total_s'] = time.perf_counter()-started
            result['candidate_timers'] = timers.copy()
    finally:
        for namespace, attribute, original in reversed(originals):
            setattr(namespace,attribute,original)
    (output/f'scoped_{mode}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))


if __name__ == '__main__':
    main()
