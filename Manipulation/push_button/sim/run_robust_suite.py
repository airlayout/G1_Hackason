#!/usr/bin/env python3
"""Paired staged/robust cases, independent calibration/depth errors, JSON summary."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--model',default='_local/bottle_push/menagerie/unitree_g1/g1.xml')
    p.add_argument('--seeds',type=int,nargs='+',default=[10000,10001])
    p.add_argument('--video',action='store_true')
    p.add_argument('--controllers',choices=('staged','robust'),nargs='+',default=['staged','robust'])
    p.add_argument('--planning-budget-s',type=float,default=10.)
    p.add_argument('--planning-scenarios',type=int,default=3)
    p.add_argument('--planning-position-span-m',type=float,default=.003)
    p.add_argument('--bottle-feedback',choices=('off','relative'),default='off')
    p.add_argument('--holdout-scenarios',type=int,default=0)
    p.add_argument('--motion-duration-s',type=float,default=30.)
    p.add_argument('--joint-speed-rad-s',type=float,default=1.2)
    p.add_argument('--joint-acceleration-rad-s2',type=float,default=4.)
    args = p.parse_args()
    if args.out.exists():
        p.error('Use a new output directory')
    args.out.mkdir(parents=True)
    entry = Path(__file__).with_name('run_robust_push_mujoco.py')
    env = dict(os.environ,MUJOCO_GL=os.environ.get('MUJOCO_GL','egl'),OPENBLAS_NUM_THREADS='1')
    import numpy as np
    rows = []
    for seed in args.seeds:
        rng = np.random.default_rng(seed)
        offset = rng.uniform(-.01,.01,3)
        calibration = rng.uniform(-.001,.001,3)
        for task,direction in [('button','up'),('button','down'),('bottle','up')]:
            for controller in args.controllers:
                stem = f'{task}_{direction}_{seed}_{controller}'
                path = args.out/(stem+'.json')
                command = [sys.executable,str(entry),'--task',task,'--button-direction',direction,
                    '--controller',controller,'--model',args.model,'--report-out',str(path),
                    '--planning-budget-s',str(args.planning_budget_s),'--planning-scenarios',str(args.planning_scenarios),
                    '--planning-position-span-m',str(args.planning_position_span_m),
                    '--bottle-feedback',args.bottle_feedback,
                    '--holdout-scenarios',str(args.holdout_scenarios),
                    '--motion-duration-s',str(args.motion_duration_s),
                    '--joint-speed-rad-s',str(args.joint_speed_rad_s),
                    '--joint-acceleration-rad-s2',str(args.joint_acceleration_rad_s2),
                    '--evaluation-seed',str(seed),'--sensor-noise-m','.0005',
                    '--offset',*map(str,offset),'--calibration-offset',*map(str,calibration)]
                if args.video:
                    command += ['--video-out',str(args.out/(stem+'.webm'))]
                with (args.out/(stem+'.log')).open('w') as log:
                    completed = subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
                data = json.loads(path.read_text()) if path.exists() else {}
                plan,execution,evaluation = (data.get(k,{}) for k in ('plan','execution','evaluation'))
                rows.append(dict(case=stem,seed=seed,task=task,direction=direction,controller=controller,
                    exit_code=completed.returncode,success=bool(data.get('completed')),error=execution.get('error',data.get('error')),
                    stopped_for_measurement=execution.get('stopped_for_measurement'),
                    planning_wall_s=plan.get('total_wall_s',0),total_wall_s=data.get('total_wall_s'),
                    simulation_s=evaluation.get('simulation_s'),completed_rollouts=plan.get('completed_rollouts',0),
                    motion_s=execution.get('motion_s'),trajectory_s=execution.get('trajectory_s'),
                    enable_s=execution.get('enable_s'),release_s=execution.get('release_s'),
                    speed_limit_fraction=execution.get('speed_limit_fraction'),
                    acceleration_limit_fraction=execution.get('acceleration_limit_fraction'),
                    planned_motion_metrics=plan.get('trajectory',{}).get('motion_metrics'),
                    holdout=data.get('holdout'),
                    aborted_rollouts=plan.get('aborted_rollouts',0),
                    min_sampled_obstacle_clearance_m=evaluation.get('min_sampled_obstacle_clearance_m'),
                    max_visual_correction_m=max((float(np.linalg.norm(v['offset_m']))
                        for v in execution.get('correction_steps',[])),default=None),evaluation=evaluation))
                (args.out/'summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
                print(stem,rows[-1]['success'],rows[-1]['error'],flush=True)
    return 0 if all(r['success'] for r in rows if r['controller']=='robust') else 1


if __name__ == '__main__':
    raise SystemExit(main())
