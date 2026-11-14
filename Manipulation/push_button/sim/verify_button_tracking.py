#!/usr/bin/env python3
"""Offline feedback verification from a saved hardware plan; never opens DDS/SSH."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from button_tracking import controller_metadata
from robust_scene import SceneSnapshot, build_scene_model
from robust_planner import Candidate, SearchSettings, physical_trial, uncertainty_scenarios


def reference_from_recording(report, settings):
    """Remove old open-loop overtravel and return over the checked approach."""
    scene = SceneSnapshot(**dict(report['plan']['scene'],button_policy=settings.button_policy))
    ref = report['plan']['trajectory']
    qs,tcps = np.asarray(ref['q']),np.asarray(ref['tcp'])
    depths = (tcps-scene.face)@np.asarray(scene.normal)+scene.tip_radius
    eligible = np.flatnonzero((np.arange(len(qs)) >= ref['press_start']) & (depths >= scene.stroke-1e-8))
    if not len(eligible):
        raise ValueError('Recorded reference does not reach the measured stroke')
    cut = int(eligible[0])
    qs,tcps = qs[:cut+1].copy(),tcps[:cut+1].copy()
    if depths[cut] > scene.stroke:
        fraction = (scene.stroke-depths[cut-1])/(depths[cut]-depths[cut-1])
        qs[-1] = qs[-2]+fraction*(qs[-1]-qs[-2])
        tcps[-1] = tcps[-2]+fraction*(tcps[-1]-tcps[-2])
    full_q = np.r_[qs,qs[-2::-1]]
    full_tcp = np.r_[tcps,tcps[-2::-1]]
    labels = ref['phases'][:cut+1]+['return']*(len(qs)-1)
    metrics = dict(limits=settings.motion_limits.to_dict(),button_feedback=controller_metadata(),
        button_tracking_settings=asdict(settings))
    candidate = Candidate(0,full_q,full_tcp,labels,ref['press_start'],cut,len(full_q)-1,
        len(full_q)*.02,0.,0.,np.zeros_like(full_q),metrics)
    return scene,candidate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--torque-effectiveness',type=float,default=1.)
    parser.add_argument('--budget-s',type=float,default=120.)
    args = parser.parse_args(argv)
    if not 0 < args.torque_effectiveness <= 1:
        parser.error('torque-effectiveness must be in (0,1]')
    report = json.loads(args.recording.read_text())
    settings = SearchSettings(button_feedback='joint',button_policy='collection',scenarios=1)
    scene,candidate = reference_from_recording(report,settings)
    kin = build_scene_model(args.model,report['calibration'],scene)
    condition = uncertainty_scenarios(settings,scene)[0]
    condition['torque_effectiveness'] = args.torque_effectiveness
    result = physical_trial(kin,scene,candidate,condition,time.perf_counter()+args.budget_s)
    result.update(robot_commands_sent=False,source_recording=str(args.recording),
        torque_effectiveness_assumption=args.torque_effectiveness,
        fixed_pelvis=True,other_upper_body_joints_dynamic=True,controller=controller_metadata())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result.get('success') else 1


if __name__ == '__main__':
    raise SystemExit(main())
