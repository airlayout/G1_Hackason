#!/usr/bin/env python3
"""Offline comparison of a saved field scene; never imports the SDK or camera."""
import argparse
from dataclasses import replace
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import robust_planner as planner
from robust_scene import SceneSnapshot, build_scene_model


def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def summarize(plan):
    return dict(accepted=plan.candidate is not None,wall_s=plan.report['total_wall_s'],
        candidate_count=len(plan.report['candidates']),completed=plan.report['completed_rollouts'],
        aborted=plan.report['aborted_rollouts'],reason=plan.report.get('reason'),
        workers=plan.report['runtime']['workers'])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log',type=Path,default=ROOT/'LOG/robust-lab-20261101/live/diagnostic_20261101_112438/result.json')
    p.add_argument('--model',type=Path,default=ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml')
    p.add_argument('--baseline-ref',default='95f2afc')
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args(argv)
    if args.output.exists(): p.error('Use a new output directory')
    saved = json.loads(args.log.read_text())
    scene = SceneSnapshot(**saved['plan']['scene'])
    calibration = saved['calibration']
    args.output.mkdir(parents=True)
    # Give spawned workers a real importable baseline module. Only the planner
    # differs: scene construction for a saved snapshot is shared by both runs.
    source = subprocess.check_output(['git','show',f'{args.baseline_ref}:Manipulation/push_button/robust_planner.py'],
                                     cwd=ROOT.parents[1],text=True)
    with tempfile.TemporaryDirectory(prefix='baseline-',dir=args.output.resolve()) as temporary:
        Path(temporary,'baseline_robust_planner.py').write_text(source)
        sys.path.insert(0,temporary)
        try:
            baseline = importlib.import_module('baseline_robust_planner')
            old_settings = baseline.SearchSettings(**saved['settings'])
            old = baseline.search(args.model,calibration,scene,old_settings)
            write(args.output/'baseline_search.json',old.to_dict())
        finally:
            sys.path.remove(temporary)
            sys.modules.pop('baseline_robust_planner',None)
    settings = replace(planner.SearchSettings(**saved['settings']),scenarios=3,
                       position_span_m=.003,bottle_feedback='off')
    new = planner.search(args.model,calibration,scene,settings)
    write(args.output/'relaxed_search.json',new.to_dict())
    # Replay the exact stored trajectory too, separately from generating a new one.
    performance = args.log.parents[2]/'performance'
    metadata = json.loads((performance/'baseline_candidate.json').read_text())
    with np.load(performance/'baseline_candidate.npz') as arrays:
        candidate = planner.Candidate(**metadata,**{k:arrays[k] for k in ('q','tcp','tau')})
    kin = build_scene_model(args.model,calibration,scene)
    conditions = planner.uncertainty_scenarios(settings,scene)
    trials = [planner.physical_trial(kin,scene,candidate,condition,time.perf_counter()+30)
              for condition in conditions]
    write(args.output/'saved_trajectory_replay.json',dict(conditions=conditions,trials=trials))
    summary = dict(baseline_ref=args.baseline_ref,scene_log=str(args.log),model=str(args.model),
        robot_connected=False,baseline=summarize(old),relaxed=summarize(new),
        calibration_uncertainty_m=scene.uncertainty_m,planning_position_span_m=settings.position_span_m,
        replay_successes=sum(t.get('success',False) and t['completed'] for t in trials),
        replay_forward_mm=[t.get('displacement_m',[None])[0]*1000 if 'displacement_m' in t else None for t in trials],
        limitations=['Saved scene; no live perception, SDK startup or free-standing balance validation',
                     'Both searches use the current shared scene model; baseline planner is loaded from git'])
    write(args.output/'summary.json',summary)
    print(json.dumps(summary,ensure_ascii=False))
    return 0 if new.candidate is not None and summary['replay_successes'] == 3 else 1


if __name__ == '__main__':
    raise SystemExit(main())
