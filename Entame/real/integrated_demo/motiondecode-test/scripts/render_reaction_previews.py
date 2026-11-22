#!/usr/bin/env python3
"""Render entry/reaction/return previews for strict-safe ranked candidates."""
import argparse
import csv
import html
import json
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path

import numpy as np

from common import (ROOT, ARM_INDICES, ARM_NAMES, NAMES, interpolate, output_path,
                    read_motion, retime_arm_window, write_json)
from play_g1_arms import INACTIVE, transition
from scene import make_scene, set_arms


def _trajectory(candidate,q0):
    source=ROOT/candidate['source_csv']
    motion,_=read_motion(source)
    raw=motion[candidate['start_frame']:candidate['end_frame'],7:][:,ARM_INDICES]
    clip_t,clip=retime_arm_window(raw,candidate['authored_duration_s'],
                                  candidate['smoothing_window_frames'],50)
    clip[:,INACTIVE]=q0[INACTIVE]
    entry=transition(q0,clip[0],candidate['entry_transition_s'])
    leave=transition(clip[-1],q0,candidate['return_transition_s'])
    entry_t=np.linspace(0,candidate['entry_transition_s'],len(entry))
    clip_t=clip_t+entry_t[-1]
    leave_t=np.linspace(clip_t[-1],clip_t[-1]+candidate['return_transition_s'],len(leave))
    t=np.concatenate((entry_t,clip_t[1:],leave_t[1:]))
    q=np.concatenate((entry,clip[1:],leave[1:]))
    boundaries=(entry_t[-1],clip_t[-1],leave_t[-1])
    phases=(['entry']*len(entry)+['reaction']*(len(clip)-1)+['return']*(len(leave)-1))
    return t,q,phases,boundaries


def _write_path(path,t,q,phases):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream)
        writer.writerow(['time_s','phase']+[f'{name}(rad)' for name in ARM_NAMES])
        for when,phase,pose in zip(t,phases,q):
            writer.writerow([f'{when:.10f}',phase]+[f'{x:.10f}' for x in pose])


def _render(path,candidate,t,q,boundaries,state):
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image,ImageDraw
    model,data=make_scene()
    base=model.qpos0.copy()
    for i,name in enumerate(NAMES):
        base[model.joint(name).qposadr[0]]=state['all_q'][i]
    camera=mujoco.MjvCamera();camera.distance=2.35;camera.elevation=-12;camera.azimuth=25
    options=mujoco.MjvOption();options.geomgroup[:]=0;options.geomgroup[0]=1;options.geomgroup[1]=1
    for g in range(model.ngeom):
        if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and model.geom_group[g]==0:
            model.geom_group[g]=3
    entry_end,clip_end,total=boundaries

    def frame(when,renderer):
        pose=interpolate(t,q,min(when,t[-1]))
        set_arms(model,data,pose,base)
        camera.lookat[:]=data.qpos[:3]+[0,0,.10]
        renderer.update_scene(data,camera=camera,scene_option=options)
        image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
        phase='ENTRY' if when<entry_end else ('REACTION' if when<clip_end else 'RETURN')
        draw.rectangle((0,0,640,66),fill=(15,21,29))
        draw.text((10,7),f'Rank {candidate["rank"]:02d} / {candidate["motion"][:58]}',fill='white')
        draw.text((10,26),f'{phase}  t={when:.2f}/{total:.2f}s  RIGHT ARM ONLY',fill='#8ee3ef')
        clear=candidate['phase_required_pair_clearance_m']
        draw.text((10,45),f'required-pair clearance mm E/C/R: '
                  f'{clear["entry"]*1000:.1f}/{clear["clip"]*1000:.1f}/{clear["return"]*1000:.1f}',
                  fill='#b8f2c4')
        return np.asarray(image)

    with mujoco.Renderer(model,height=480,width=640) as renderer:
        with imageio.get_writer(output_path(path),fps=30,codec='libx264',quality=7) as writer:
            for when in np.linspace(0,total,max(2,int(np.ceil(total*30))+1)):
                writer.append_data(frame(float(when),renderer))


def _write_index(path,items,report):
    cards=[]
    for item in items:
        c=item['candidate'];raw=c['phase_clearance_m'];required=c['phase_required_pair_clearance_m']
        cards.append(f'''<article>
<h2>Rank {c['rank']:02d}: {html.escape(c['motion'])}</h2>
<p><strong>{html.escape(c['reaction_type'])}</strong> — {html.escape(c['suggested_game_use'])}</p>
<video controls preload="metadata" src="{html.escape(item['mp4'])}"></video>
<dl>
<dt>Frames</dt><dd>[{c['start_frame']}, {c['end_frame']}) → {c['authored_duration_s']:.1f} s</dd>
<dt>Safety</dt><dd>{html.escape(c['safety_tier'])}</dd>
<dt>q0 max delta</dt><dd>{c['max_joint_delta_over_clip_from_q0_rad']:.3f} rad</dd>
<dt>Entry / return</dt><dd>{c['entry_transition_s']:.3f} / {c['return_transition_s']:.3f} s</dd>
<dt>Required-pair clearance E / C / R</dt><dd>{required['entry']*1000:.3f} / {required['clip']*1000:.3f} / {required['return']*1000:.3f} mm</dd>
<dt>Raw clearance E / C / R</dt><dd>{raw['entry']*1000:.3f} / {raw['clip']*1000:.3f} / {raw['return']*1000:.3f} mm (includes measured q0 baseline)</dd>
<dt>Velocity / acceleration</dt><dd>{c['max_velocity_rad_s']:.3f} rad/s / {c['max_acceleration_rad_s2']:.3f} rad/s²</dd>
</dl><p><a href="{html.escape(item['path_csv'])}">reviewed 14-arm path CSV</a></p>
</article>''')
    if not cards:
        cards=['<article><h2>No strict-safe preview candidates</h2></article>']
    body=f'''<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>G1 reaction candidates</title>
<style>body{{font:16px system-ui;background:#10161e;color:#edf4f7;margin:0;padding:28px}}main{{max-width:980px;margin:auto}}article{{background:#1a2530;border:1px solid #314252;border-radius:14px;padding:20px;margin:20px 0}}video{{width:100%;max-width:640px;background:#000}}dl{{display:grid;grid-template-columns:minmax(190px,1fr) 2fr;gap:8px 18px}}dt{{color:#9fc2cf}}dd{{margin:0}}a{{color:#8ee3ef}}code{{color:#b8f2c4}}</style></head>
<body><main><h1>G1 right-arm reaction candidates</h1>
<p>Receive-only q0 observationから厳格安全条件を通過した候補のみ。動画はentry / reaction / returnを含みます。</p>
<p>Result: <code>{html.escape(report['result'])}</code> / strict PASS {report['pass_counts']['strict_ready_le_0_8_rad']}</p>
{''.join(cards)}</main></body></html>'''
    output_path(path).write_text(body,encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ranked-json',type=Path,required=True)
    parser.add_argument('--state-json',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'output/reaction_candidates')
    args=parser.parse_args()
    report=json.loads(args.ranked_json.read_text());state=json.loads(args.state_json.read_text())
    if report.get('schema')!='motiondecode-test.reaction-search.v1':
        raise ValueError('Unsupported reaction ranking schema')
    if report.get('source_state_json')!=str(args.state_json.resolve()):
        raise ValueError('Ranking and q0 state do not match')
    if not report.get('receive_only') or report.get('command_publishers_created')!=0:
        raise ValueError('Ranking does not have receive-only provenance')
    output_dir=args.output_dir.resolve();output_dir.mkdir(parents=True,exist_ok=True)
    q0=np.asarray(state['arm_q'],dtype=float)
    items=[]
    for candidate in [x for x in report['candidates'] if x['strict_safety_passed']][:5]:
        stem=f'rank{candidate["rank"]:02d}'
        mp4=output_dir/f'{stem}.mp4';path_csv=output_dir/f'{stem}_arms.csv'
        t,q,phases,boundaries=_trajectory(candidate,q0)
        _write_path(path_csv,t,q,phases)
        print(f'Rendering {mp4} ({boundaries[-1]:.2f}s)',flush=True)
        _render(mp4,candidate,t,q,boundaries,state)
        items.append({'rank':candidate['rank'],'motion':candidate['motion'],
                      'mp4':mp4.name,'path_csv':path_csv.name,'candidate':candidate})
    index=output_dir/'index.html';_write_index(index,items,report)
    manifest={
        'schema':'motiondecode-test.reaction-previews.v1',
        'ranked_json':str(args.ranked_json.resolve()),'source_state_json':str(args.state_json.resolve()),
        'right_arm_only':True,'includes':['entry','reaction','return'],
        'command_publishers_created':0,
        'items':[{key:value for key,value in item.items() if key!='candidate'} for item in items],
        'index_html':str(index),
    }
    write_json(output_dir/'manifest.json',manifest)
    print(f'Index: {index}',flush=True)


if __name__=='__main__':
    raise SystemExit(main())
