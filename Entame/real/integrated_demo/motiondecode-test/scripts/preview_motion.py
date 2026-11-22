#!/usr/bin/env python3
"""Offline MuJoCo FK preview. No dynamics or DDS. Raw FPS is never inferred."""
import argparse
import os
os.environ.setdefault('MUJOCO_GL','egl')
import math
from pathlib import Path
import numpy as np
from common import ROOT, ARMS, read_motion, read_arms, output_path, interpolate
from scene import make_scene, set_full, set_arms

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('motion',type=Path)
    p.add_argument('--arms',action='store_true')
    p.add_argument('--right-only',action='store_true',help='Nominal first trial: pin left arm to its first frame')
    p.add_argument('--display-fps',type=float,help='Raw CSV visualization cadence ONLY; source FPS remains unknown')
    p.add_argument('--speed',type=float,default=1)
    p.add_argument('--start-frame',type=int,default=0)
    p.add_argument('--end-frame',type=int)
    p.add_argument('--mp4',type=Path)
    p.add_argument('--sheet',type=Path)
    p.add_argument('--view-offset',type=float,default=25)
    p.add_argument('--interactive',action='store_true')
    args=p.parse_args()
    if args.right_only and not args.arms:p.error('--right-only requires --arms')
    if not np.isfinite(args.speed) or args.speed<=0:p.error('speed must be positive')
    if args.arms:
        t,q,meta=read_arms(args.motion);t=t/args.speed
        label=f'ARMS ONLY / explicit time / speed {args.speed:g}'
        if args.right_only:
            for i,j in enumerate(ARMS):
                if j['name'].startswith('left_'):q[:,i]=q[0,i]
            label=f'RIGHT ARM / left pinned for preview / speed {args.speed:g}'
    else:
        a,_=read_motion(args.motion)
        end=args.end_frame if args.end_frame is not None else len(a)
        if not 0<=args.start_frame<end<=len(a):p.error('Invalid frame interval')
        a=a[args.start_frame:end]
        if args.display_fps is None or not np.isfinite(args.display_fps) or args.display_fps<=0:
            p.error('Raw source FPS is unknown: specify --display-fps for visualization only')
        t=np.arange(len(a))/(args.display_fps*args.speed)
        label=f'RAW FK / SOURCE FPS UNKNOWN / display {args.display_fps:g} fps'
    model,data=make_scene()
    import mujoco
    camera=mujoco.MjvCamera();camera.distance=2.45;camera.elevation=-13
    options=mujoco.MjvOption();options.geomgroup[:]=0;options.geomgroup[1]=1;options.geomgroup[0]=1
    # Hide duplicated collision meshes, but retain the floor and small feet spheres.
    for g in range(model.ngeom):
        if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and model.geom_group[g]==0:
            model.geom_group[g]=3
    def pose(when):
        if args.arms:
            set_arms(model,data,interpolate(t,q,when));yaw=0
        else:
            k=min(int(round(when*args.display_fps*args.speed)),len(a)-1)
            origin=np.array([a[0,0],a[0,1],0])
            set_full(model,data,a[k],origin)
            w,x,y,z=a[k,3:7];yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
        camera.lookat[:]=data.qpos[:3]+[0,0,.1]
        camera.azimuth=yaw+args.view_offset
    if args.interactive:
        import time
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model,data) as viewer:
            start=time.monotonic()
            while viewer.is_running():
                pose(min(time.monotonic()-start,t[-1]))
                viewer.cam.lookat[:]=camera.lookat
                viewer.sync();time.sleep(.02)
        return
    if not(args.mp4 or args.sheet):p.error('Use --sheet, --mp4 or --interactive')
    from PIL import Image,ImageDraw
    def render(when):
        pose(when);renderer.update_scene(data,camera=camera,scene_option=options)
        im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,640,48),fill=(16,22,30))
        draw.text((10,7),args.motion.stem[:80],fill='white')
        draw.text((10,25),label+f' / t={when:.2f}s',fill='#a6e0e8')
        return im
    with mujoco.Renderer(model,height=480,width=640) as renderer:
        if args.sheet:
            sheet=Image.new('RGB',(640*4,480*3))
            for i,when in enumerate(np.linspace(0,t[-1],12)):
                sheet.paste(render(when),(i%4*640,i//4*480))
            sheet.save(output_path(args.sheet));print(args.sheet)
        if args.mp4:
            import imageio.v2 as imageio
            with imageio.get_writer(output_path(args.mp4),fps=30,codec='libx264',quality=7) as writer:
                for when in np.linspace(0,t[-1],max(2,int(np.ceil(t[-1]*30))+1)):
                    writer.append_data(np.array(render(when)))
            print(args.mp4)

if __name__=='__main__':main()
