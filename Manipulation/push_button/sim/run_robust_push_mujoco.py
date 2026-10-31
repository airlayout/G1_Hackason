#!/usr/bin/env python3
"""RGB-D -> reconstructed scene -> robust search -> shared streaming execution.

The fixture detector uses a segmentation bbox (explicit oracle). --weights uses
YOLO instead. Object truth is used only to make images and evaluate the outcome.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kinematics import load_robot
from robust_scene import SceneSnapshot, build_scene_model, estimate_bottle, contact_metrics
from robust_planner import search, PERIOD, evaluate_holdout
from robust_perception import Observation, ObservationWindow, snapshot
from robust_execution import execute, validate_start
from robust_options import add_arguments, parse
from vision import CameraIntrinsics, estimate_button_target


class World:
    def __init__(self,args,calibration,fixture):
        kin = build_scene_model(args.model,calibration,fixture)
        self.eval_kin = kin
        self.model,self.data = kin.model,kin.data
        self.data.ctrl[:] = fixture.q
        self.qadr,self.actuator = kin.qadr,kin.actuator
        self.arm_geoms = kin.arm_geoms
        self.target = np.array(fixture.q[22:29]); self.tau = np.zeros(7)
        self.home = self.target.copy(); self.release_error = None
        self.fixture,self.args = fixture,args
        self.peak_press = self.max_tilt = self.max_unexpected_force = 0.
        self.min_obstacle_clearance = .05
        self.unexpected_collision = False
        self.physics_ticks = 0
        self.target_body = self.model.body('robust_target').id
        if fixture.task == 'button':
            self.model.geom_rgba[self.model.geom('robust_target_geom').id,:3] = .15
        self.initial_object = self.data.xpos[self.target_body].copy()
        position = np.array([-.20,-.62,1.28]); target = np.array(fixture.face)
        back = position-target; back /= np.linalg.norm(back)
        right = np.cross([0,0,1],back); right /= np.linalg.norm(right)
        up = np.cross(back,right)
        self.rotation = np.column_stack((right,-up,-back))
        self.transform = np.eye(4); self.transform[:3,:3] = self.rotation; self.transform[:3,3] = position
        self.renderer = mujoco.Renderer(self.model,height=240,width=320)
        self.camera = mujoco.MjvCamera(); self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = target
        offset = position-target
        self.camera.distance = np.linalg.norm(offset)
        self.camera.azimuth = np.degrees(np.arctan2(offset[1],offset[0]))+180
        self.camera.elevation = -np.degrees(np.arcsin(offset[2]/self.camera.distance))
        self.model.vis.global_.fovy = 45.
        focal = .5*240/math.tan(math.radians(45)/2)
        self.intrinsics = CameraIntrinsics(focal,focal,160.,120.)
        self.writer = None
        if args.video_out:
            import cv2
            Path(args.video_out).parent.mkdir(parents=True,exist_ok=True)
            self.writer = cv2.VideoWriter(args.video_out,cv2.VideoWriter_fourcc(*'VP80'),5.,(320,240))
            if not self.writer.isOpened():
                raise RuntimeError('Video writer failed')
        self.video_next = 0.

    def now(self):
        return self.data.time

    def configuration(self):
        if self.args.fault == 'state-disconnect' and self.now() > 3.:
            raise RuntimeError('Injected state disconnect')
        return self.data.qpos[:29].copy(),self.fixture.imu,self.now(),0.,0.

    def enable(self):
        pass

    def start_approach(self):
        pass

    def stream(self,goal,*,velocity=None):
        self.target = np.array(goal)

    def wait(self,duration):
        steps = max(0,math.ceil(duration/self.model.opt.timestep-1e-8))
        for _ in range(steps):
            self.data.ctrl[self.actuator] = self.target
            self.data.qfrc_applied[self.qadr] = self.tau
            mujoco.mj_step(self.model,self.data)
            self.physics_ticks += 1
            if self.physics_ticks % 10 == 0:
                try:
                    self.min_obstacle_clearance = min(self.min_obstacle_clearance,
                        contact_metrics(self.eval_kin,self.fixture,contact=True))
                except ValueError:
                    self.unexpected_collision = True
            if self.fixture.task == 'button':
                self.peak_press = max(self.peak_press,float(self.data.qpos[self.model.joint('robust_object_joint').qposadr[0]]))
                if self.peak_press >= .9*self.fixture.stroke:
                    self.model.geom_rgba[self.model.geom('robust_target_geom').id,:3] = 1.
            else:
                self.max_tilt = max(self.max_tilt,math.degrees(math.acos(np.clip(self.data.xmat[self.target_body,8],-1,1))))
            for contact_index, contact in enumerate(self.data.contact):
                pair = {int(contact.geom1),int(contact.geom2)}
                if pair & self.arm_geoms:
                    if pair == {self.model.geom('button_tip').id,self.model.geom('robust_target_geom').id}:
                        continue
                    force = np.zeros(6); mujoco.mj_contactForce(self.model,self.data,contact_index,force)
                    self.max_unexpected_force = max(self.max_unexpected_force,float(np.linalg.norm(force[:3])))
            if self.writer is not None and self.now() >= self.video_next:
                self.renderer.update_scene(self.data,camera=self.camera)
                rgb = self.renderer.render()
                import cv2
                frame = rgb[:,:,::-1].copy()
                cv2.putText(frame,f'{self.fixture.task}  t={self.now():.1f}s',(8,18),
                            cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                cv2.putText(frame,f'press={1000*self.peak_press:.2f}mm',(8,38),
                            cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                self.writer.write(frame); self.video_next = self.now()+.2

    def close(self):
        pass  # release in the shared runner; render resources remain for outcome images

    def finish(self):
        if self.writer is not None:
            self.writer.release()
        self.renderer.close()


class Observer:
    """Deterministic simulated camera sampling; no pause/settle requests during execution."""
    def __init__(self,world,args,calibration):
        self.w,self.args,self.calibration = world,args,calibration
        self.window = ObservationWindow()
        self.value = None
        self.last_capture = -np.inf
        self.frame_id = 0
        self.frames = []
        self.detector = None
        self.base = args.bottle_base_height
        self.initial_box = None
        self.transform = world.transform.copy()
        self.transform[:3,3] += args.calibration_offset
        if args.weights:
            from bottle import make_bottle_detector
            if args.task == 'bottle':
                self.detector = make_bottle_detector(args.weights)
            else:
                from j1gen_camera import J1CameraRuntime
                runtime = J1CameraRuntime()
                cls = runtime.module('perception_bridge').perception('detector.yolo_detector').YoloDetector
                self.detector = cls(model_name=args.weights,classes=['button'],confidence_threshold=.25,device='cpu',imgsz=640)

    def capture(self):
        w = self.w
        w.renderer.update_scene(w.data,camera=w.camera)
        rgb = w.renderer.render().copy()
        self.last_rgb = rgb
        w.renderer.enable_depth_rendering(); depth = w.renderer.render().copy(); w.renderer.disable_depth_rendering()
        if self.args.sensor_noise_m:
            rng = np.random.default_rng([self.args.evaluation_seed,self.frame_id])
            depth += rng.normal(0.,self.args.sensor_noise_m,depth.shape)
        if self.detector is None:
            w.renderer.enable_segmentation_rendering(); seg = w.renderer.render().copy(); w.renderer.disable_segmentation_rendering()
            mask = seg[:,:,0] == w.model.geom('robust_target_geom').id
            ys,xs = np.where(mask)
            if len(xs) < 30:
                raise RuntimeError('Fixture target outside image')
            box = (int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1))
            if self.args.task == 'button':
                if self.initial_box is None:
                    self.initial_box = box
                box = self.initial_box
        else:
            detections = self.detector.detect(rgb[:,:,::-1])
            if len(detections) != 1:
                raise RuntimeError('Detector did not select exactly one target')
            box = detections[0].bbox
        if self.args.task == 'bottle':
            g = estimate_bottle(depth,box,w.intrinsics,self.transform,base_height=self.base)
        else:
            target = estimate_button_target(depth,box,w.intrinsics,self.transform,
                expected_face_xyz=None if self.value is None else self.value.geometry['face'],
                tracking=self.value is not None)
            g = dict(face=target.face_xyz,normal=target.press_direction,radius=self.calibration['button_radius_m'])
        self.frame_id += 1
        stamp = w.now()
        if self.args.fault == 'stale-images' and stamp > 3.:
            stamp -= 1.
        if self.args.fault == 'target-loss' and stamp > 3.:
            raise RuntimeError('Injected target loss')
        q,imu,*_ = w.configuration()
        value = Observation(g,tuple(q),tuple(imu),stamp,self.frame_id)
        completed = self.window.consume(value,w.now())
        self.last_capture = w.now()
        if completed is not None:
            self.value = completed
            self.frames.append(dict(frame_id=self.frame_id,frame_ids=completed.frame_ids,captured_at_s=stamp,geometry=g))
        return rgb

    def initial(self):
        # Initial acquisition only; hold measured arm posture with nominal gravity.
        for _ in range(3):
            self.w.wait(.2); self.capture()
        if self.value is None:
            raise RuntimeError('Initial RGB-D not stable')
        if self.args.task == 'bottle':
            self.base = self.value.geometry['base_height']
        return self.value

    def latest(self):
        if self.w.now()-self.last_capture >= .2-1e-8:
            try:
                self.capture()
            except ValueError:
                self.last_capture = self.w.now()
        if self.value is None or not 0 <= self.w.now()-self.value.captured_at <= .5:
            raise RuntimeError('Stale simulated perception')
        return self.value


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=('button','bottle'),default='button')
    p.add_argument('--model',default='_local/bottle_push/menagerie/unitree_g1/g1.xml')
    p.add_argument('--weights')
    p.add_argument('--button-direction',choices=('up','down'),default='up')
    p.add_argument('--report-out',required=True)
    p.add_argument('--video-out')
    p.add_argument('--snapshot')
    p.add_argument('--benchmark-only',action='store_true')
    p.add_argument('--controller',choices=('robust','staged'),default='robust',
                   help='staged: production controller with the same B geometry for comparison')
    p.add_argument('--holdout-scenarios',type=int,default=0)
    p.add_argument('--holdout-seed',type=int,default=10000)
    p.add_argument('--fault',choices=('none','stale-images','target-loss','state-disconnect'),default='none')
    p.add_argument('--offset',type=float,nargs=3,default=(0.,0.,0.))
    p.add_argument('--calibration-offset',type=float,nargs=3,default=(0.,0.,0.))
    p.add_argument('--sensor-noise-m',type=float,default=0.)
    p.add_argument('--evaluation-seed',type=int,default=10000)
    add_arguments(p)
    p.set_defaults(motion_planner='robust')
    args,settings = parse(p,argv)
    report = dict(schema='g1-robust-sim-v1',completed=False,arguments=vars(args),settings=asdict(settings),
                  detector='yolo' if args.weights else 'oracle-bbox',limitations=[
                  'Fixed pelvis; no free-standing balance validation',
                  'Simulated 5Hz RGB-D sampling; perception wall cost is not injected into physics time'])
    world = None
    start_wall = time.perf_counter()
    try:
        calibration = dict(tip_offset_m=(.1,0.,0.),tip_radius_m=.015,pelvis_height_m=.79,
                           position_uncertainty_m=.001,button_radius_m=.035)
        _,rest = load_robot(Path(args.model).resolve(),calibration['tip_offset_m'],.79)
        face = np.array((.326,-.12,.84) if args.task=='bottle' else (.36,-.12,1.05 if args.button_direction=='up' else .98))
        face += args.offset
        fixture = SceneSnapshot(args.task,tuple(face),(1.,0.,0.),.034 if args.task=='bottle' else .035,
            tuple(rest),(0.,0.,0.),0.,base_height=.76+args.offset[2] if args.task=='bottle' else None,
            uncertainty_m=.001,stroke=args.stroke,push_distance=args.push_distance)
        world = World(args,calibration,fixture)
        # Initial gravity support, identical nominal policy to the execution torque path.
        world.tau = np.clip(world.data.qfrc_bias[world.qadr],[-25]*4+[-5]*3,[25]*4+[5]*3)*settings.gravity_scale
        observer = Observer(world,args,calibration)
        initial = observer.initial()
        scene = snapshot(args.task,initial,calibration,args)
        if args.snapshot:
            import PIL.Image
            Path(args.snapshot).parent.mkdir(parents=True,exist_ok=True)
            PIL.Image.fromarray(observer.last_rgb).save(args.snapshot)
        def monitor():
            validate_start(scene,world.configuration(),observer.latest())
        if args.controller == 'staged':
            from robust_baseline import run as staged_run
            report['execution'] = staged_run(world,observer,scene,args,calibration,settings)
        else:
            plan = search(args.model,calibration,scene,settings,monitor=monitor)
            report['plan'] = plan.to_dict()
            if plan.candidate is None:
                return 1
            if args.holdout_scenarios:
                report['holdout'] = evaluate_holdout(args.model,calibration,plan,settings,
                    seed=args.holdout_seed,count=args.holdout_scenarios)
            if args.benchmark_only:
                report['planning_completed'] = True
                return 0
            report['execution'] = execute(plan,args.model,calibration,world,world.configuration,observer.latest,settings,
                now=world.now,set_torque=lambda tau:setattr(world,'tau',tau))
        moved = world.data.xpos[world.target_body]-world.initial_object
        physical = (world.peak_press >= .9*args.stroke if args.task=='button' else
                    abs(moved[0]-args.push_distance) <= .01 and abs(moved[1]) <= .01 and world.max_tilt <= 10)
        report['evaluation'] = dict(physical_success=bool(physical),peak_press_m=world.peak_press,
            displacement_m=moved.tolist(),max_tilt_deg=world.max_tilt,max_unexpected_arm_force_n=world.max_unexpected_force,
            min_sampled_obstacle_clearance_m=world.min_obstacle_clearance,unexpected_collision=world.unexpected_collision,
            simulation_s=world.now())
        report['completed'] = bool(report['execution']['completed'] and physical and world.max_unexpected_force < 1.
                                   and not world.unexpected_collision)
        report['perception_frames'] = observer.frames
        return 0 if report['completed'] else 1
    except (ValueError,RuntimeError,OSError) as exc:
        report['error'] = str(exc); return 2
    finally:
        report['total_wall_s'] = time.perf_counter()-start_wall
        if world is not None:
            world.finish()
        path = Path(args.report_out); path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        plan_data,execution_data = report.get('plan',{}),report.get('execution',{})
        holdout = report.get('holdout',{})
        print(json.dumps(dict(task=args.task,controller=args.controller,completed=report['completed'],
            planning_completed=report.get('planning_completed',False),report_out=args.report_out,
            error=execution_data.get('error',report.get('error')),planning_wall_s=plan_data.get('total_wall_s'),
            completed_rollouts=plan_data.get('completed_rollouts'),aborted_rollouts=plan_data.get('aborted_rollouts'),
            holdout_completed=holdout.get('completed'),holdout_successes=holdout.get('successes'),
            evaluation=report.get('evaluation'),total_wall_s=report['total_wall_s']),ensure_ascii=False))


if __name__ == '__main__':
    raise SystemExit(main())
