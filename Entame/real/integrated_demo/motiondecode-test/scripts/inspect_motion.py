#!/usr/bin/env python3
import argparse
from pathlib import Path
from common import read_motion, stats, write_json, mapping_report

def main():
    p=argparse.ArgumentParser(description='Inspect G1 CSV; FPS unknown unless explicitly evaluated.')
    p.add_argument('motion',type=Path)
    p.add_argument('--fps',type=float,help='Hypothetical evaluation rate; does NOT certify source FPS')
    p.add_argument('--json',type=Path)
    p.add_argument('--mapping',type=Path)
    args=p.parse_args();a,header=read_motion(args.motion);s=stats(a,args.fps)
    for k,v in s.items():
        if k!='joints':print(f'{k}: {v}')
    print('joint | SDK | min/max rad | max rad/frame | max rad/s | total variation rad')
    for j in s['joints']:
        speed=f"{j['max_speed_rad_s']:.4f}" if j['max_speed_rad_s'] is not None else 'UNKNOWN'
        print(f"{j['name']:28} {j['sdk_index']:2} {j['min_rad']:8.4f} {j['max_rad']:8.4f} "
              f"{j['max_step_rad']:.5f} {speed:>9} {j['total_variation_rad']:.3f}")
    if args.json:write_json(args.json,s)
    if args.mapping:write_json(args.mapping,mapping_report(header))

if __name__=='__main__':main()
