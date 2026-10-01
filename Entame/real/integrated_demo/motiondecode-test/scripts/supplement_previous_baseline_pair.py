#!/usr/bin/env python3
"""Offline exact-FK analysis for a previous failing pair absent from today's baseline.

Uses the same measured_pose_clearances and same configured mesh pair; only this
supplement's output pair selection differs. Full-pair replay runs independently.
"""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
from common import NAMES,write_json
from scene import make_scene,clearance_pairs,measured_pose_clearances
from measure_hold_baseline_noise import series_stats,corr,summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();folder=args.directory
    snap=json.loads((folder/'fresh_q0.json').read_text());q0=np.array(snap['all_q'])
    raw=np.loadtxt(folder/'lowstate_samples.csv',delimiter=',',skiprows=1)
    times=raw[:,1];q=raw[:,5:34]
    model,data=make_scene();base=model.qpos0.copy()
    key='left_rubber_hand | left_hip_pitch_link'
    pairs=[p for p in clearance_pairs(model) if ' | '.join(p[2])==key]
    assert len(pairs)==1
    ds,_=measured_pose_clearances(model,data,base,pairs,q0);initial=ds[key]*1000
    values=[];last=time.monotonic()
    with (folder/'previous_hand_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['sample_id','received_monotonic_s',key+' [m]'])
        for i,pose in enumerate(q):
            ds,_=measured_pose_clearances(model,data,base,pairs,pose)
            values.append(ds[key]*1000);writer.writerow([i,times[i],ds[key]])
            if time.monotonic()-last>15:
                print(f'Previous-hand identical FK {i+1}/{len(q)}',flush=True);last=time.monotonic()
    x=np.array(values);delta=q-q0
    s=series_stats(x,initial);s.update(pair=key,units='mm',previous_failing_pair=True,
                                      currently_baseline_pair=bool(initial<15))
    s['fraction_worsening_over_current_tolerance']=float(np.mean(initial-x>.001))
    s['fraction_absolute_deviation_over_current_tolerance']=float(np.mean(abs(x-initial)>.001))
    s['sample_to_sample_abs_step_mm']=summary(abs(np.diff(x)))
    s['fraction_abs_steps_ge_0_012_mm']=float(np.mean(abs(np.diff(x))>=.012))
    s['step_vs_reception_interval_correlation']=corr(abs(np.diff(x)),np.diff(times))
    s['initial_minus_median_mm']=float(initial-np.median(x))
    correlations=[];jac=[]
    for j,name in enumerate(NAMES):
        up=q0.copy();down=q0.copy();up[j]+=1e-6;down[j]-=1e-6
        a,_=measured_pose_clearances(model,data,base,pairs,up)
        b,_=measured_pose_clearances(model,data,base,pairs,down)
        grad=(a[key]-b[key])*1000/2e-6;jac.append(grad)
        correlations.append({'joint':name,'pearson_r':corr(delta[:,j],x-initial),'sensitivity_mm_per_rad':grad})
    s['joint_correlations']=sorted(correlations,key=lambda z:abs(z['pearson_r'] or 0),reverse=True)
    residual=(x-initial)-delta@np.array(jac)
    s['same_FK_local_sensitivity_explanation']={'rmse_mm':float(np.sqrt(np.mean(residual**2))),
        'max_abs_residual_mm':float(max(abs(residual))),
        'r2':float(1-np.sum(residual**2)/np.sum((x-x.mean())**2))}
    write_json(folder/'previous_hand_noise_stats.json',s)
    print(json.dumps(s,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
