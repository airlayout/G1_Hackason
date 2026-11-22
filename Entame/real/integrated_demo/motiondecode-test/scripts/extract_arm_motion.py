#!/usr/bin/env python3
"""Extract a frame interval by joint NAME, then explicitly retime it in seconds."""
import argparse
from pathlib import Path
import numpy as np
from common import (ROOT, ARMS, ARM_NAMES, ARM_INDICES, SDK_EXAMPLE, URDF, read_motion,
                    output_path, write_json, digest, validate_arm_pose, retime_arm_window)
from scene import check_arm_path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('motion',type=Path)
    p.add_argument('--start-frame',required=True,type=int,help='Zero based, inclusive')
    p.add_argument('--end-frame',required=True,type=int,help='Zero based, exclusive')
    p.add_argument('--duration',required=True,type=float,help='AUTHORED output seconds; not source duration')
    p.add_argument('--smoothing-window',type=int,default=1,help='Odd centered box filter, edge padding, no unwrap')
    p.add_argument('--output',type=Path,default=ROOT/'output/selected_arms.csv')
    args=p.parse_args();a,header=read_motion(args.motion)
    if not 0<=args.start_frame<args.end_frame<=len(a):p.error('Invalid frame interval')
    if not np.isfinite(args.duration) or not 1<=args.duration<=3:p.error('First-test authored duration must be 1..3 s')
    q=a[args.start_frame:args.end_frame,7:][:,ARM_INDICES].copy()
    if len(q)<3 or np.max(np.abs(np.diff(q,axis=0)))>.15:
        p.error('Too short or discontinuous arm interval (>0.15 rad/frame)')
    validate_arm_pose(q)
    w=args.smoothing_window
    try:
        output_t,q=retime_arm_window(q,args.duration,w,50)
    except ValueError as exc:
        p.error(str(exc))
    validate_arm_pose(q)
    collision=check_arm_path(q)
    path=output_path(args.output)
    np.savetxt(path,np.column_stack((output_t,q)),delimiter=',',fmt='%.10f',comments='',
               header=','.join(['time_s']+[f'{j}(rad)' for j in ARM_NAMES]))
    downloads={x['local']:x for x in __import__('json').loads((ROOT/'data/metadata/downloads.json').read_text())}
    info=downloads.get(str(args.motion.resolve().relative_to(ROOT)),{})
    meta={'schema':1,'source':str(args.motion.resolve().relative_to(ROOT)),
          'source_sha256':digest(args.motion),'hf_path':info.get('path'),'hf_revision':info.get('revision'),
          'source_fps':None,'source_fps_status':'UNVERIFIED: README 120Hz vs third-party Viewer default 30',
          'start_frame_inclusive':args.start_frame,'end_frame_exclusive':args.end_frame,
          'timing_mode':'explicit_retiming','authored_duration_s':args.duration,'output_sample_hz':50,
          'phase':'quintic smoothstep','smoothing_window_frames':w,
          'arm_names':ARM_NAMES,'sdk_indices':ARM_INDICES,
          'mapping':[dict(j,csv_column_zero_based=header.index(j['csv_column'])) for j in ARMS],
          'sdk_example_sha256':digest(SDK_EXAMPLE),'official_urdf_sha256':digest(URDF),
          'csv_sha256':digest(path),'nominal_clearance':collision,
          'warning':'Kinematic candidate only. Actual current-pose transitions checked before control.'}
    write_json(path.with_suffix('.json'),meta)
    print(f'{path}\nExplicit authored duration={args.duration}s, source FPS remains UNKNOWN')
    print(collision)
    if not collision['passed']:raise SystemExit('Artifact saved for inspection; clearance FAILED, not playable')

if __name__=='__main__':main()
