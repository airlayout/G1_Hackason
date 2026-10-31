"""Evaluate the real bottle entrypoint on calibrated realistic RGB-D and arm I/O.

Production entrypoint, perception, kinematics, controller, motion interpolation,
camera freshness checks and release ramp are reused. A deterministic simulation
scheduler replaces the ArmMotion publishing thread and hardware transports.
"""
from __future__ import annotations

import argparse
import builtins
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time as wall_time
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO/'Manipulation/push_button/real'),
                str(REPO/'Manipulation/push_button/sim'),
                str(REPO/'Manipulation/push_button'), str(REPO/'Button_Press/Yada')]
from common.config import load_config
from common.realism import SensorModel, sample_realism, sway_offset
import bottle
from bottle_layout import inspect_layout, validate_layout_xy
import push_bottle
import align_button
import push_button as sdk
import j1gen_camera
from kinematics import ArmKinematics, load_robot
from run_bottle_mujoco import build_scene
from trajectory import validate_arm_tau
from cartesian import CartesianJointController
from bottle_perception import PerceptionMailbox

MODEL = REPO/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
WEIGHTS = REPO/'yolo26n.pt'
BASE = ['bottle_eval_'+s for s in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]


class VirtualPerception(PerceptionMailbox):
    """Run the production engine, but expose its outputs only at job completion.

    Physics/arm publication continue while the sensor job is pending. Measured
    costs are charged once to the sensor job, never to the control caller.
    """
    def __init__(self, engine, now, wait, world):
        super().__init__(engine, now, wait)
        self.world, self.pending_result = world, None
        engine.now = engine.localizer.now = lambda: world.now()+(world.perception_cost if world.perception_busy else 0.)
        world.perception = self

    def tick(self):
        world = self.world
        if self.error is not None:
            return
        if self.pending_result is not None:
            ready, generation, observation, table, error = self.pending_result
            if world.now()+1e-9 < ready:
                return
            self.publish(generation, observation, table, error)
            self.pending_result = None
            if error is not None:
                return
        with self.lock:
            requested = self.pending is not None
        if not requested and world.now()+1e-9 < self.engine.next_frame:
            return
        generation, request, progress = self.take_request()
        if request is not None:
            self.engine.configure(request)
        world.perception_busy, world.perception_cost = True, 0.
        observation = table = error = None
        try:
            observation, table = self.engine.step(progress)
        except Exception as exc:
            error = exc
        finally:
            world.perception_busy = False
        self.pending_result = (world.now()+world.perception_cost, generation, observation, table, error)

    def close(self):
        self.world.perception = None
        self.engine.close()


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def rpy(quat):
    w, x, y, z = quat
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(np.clip(2*(w*y-z*x), -1, 1)),
                     math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def camera_profile_geometry(model, data, rest, profile, runtime=None):
    if profile == 'legacy':
        camera = model.camera('bottle_rgbd').id
        optical = np.eye(4)
        optical[:3, :3] = data.cam_xmat[camera].reshape(3, 3)@np.diag([1, -1, -1])
        optical[:3, 3] = data.cam_xpos[camera]
        return optical, 48.
    runtime = runtime or j1gen_camera.J1CameraRuntime()
    config = runtime.module('config').load_config('robot.yaml')
    head = runtime.module('camera_geometry').HeadCameraTransform(config)
    rotation, position = head.pelvis_from_optical(rest[12:15])
    pelvis = model.body('pelvis').id
    base = np.eye(4)
    base[:3, :3], base[:3, 3] = data.xmat[pelvis].reshape(3, 3), data.xpos[pelvis]
    optical = np.eye(4)
    optical[:3, :3], optical[:3, 3] = rotation, position
    return base@optical, float(config['head_camera']['fovy_deg'])


def bottle_world_xy(options, realism):
    xy = rotation([0, 0, -realism.stance_yaw])[:2, :2] @ np.array([options.bottle_x, options.bottle_y]) \
        + realism.stance_xy
    return np.array([.45, -.25]) if options.fault == 'unreachable' else xy


def check_layouts(options):
    """Optional static diagnostic; no World, Renderer, DDS, or mj_step calls."""
    from bottle import add_table
    settings = bottle.BottleSettings(.75)
    model, rest = load_robot(options.model, (.110, -.003, 0), .79, .015,
                             scene_builder=lambda root: add_table(root, settings))
    kin = ArmKinematics(model)
    kin.update(rest)
    rows = []
    for seed in options.seeds:
        realism = sample_realism(seed, load_config('contest.yaml')['realism'])
        xy = bottle_world_xy(options, realism)
        geometry_model, geometry_rest = model, rest
        if options.camera_profile == 'legacy':
            geometry_model, geometry_rest = load_robot(options.model, (.110, -.003, 0), .79, .015,
                scene_builder=lambda root: build_scene(root, settings, *xy))
        data = mujoco.MjData(geometry_model)
        data.qpos[:29] = geometry_rest
        mujoco.mj_forward(geometry_model, data)
        optical, fovy = camera_profile_geometry(geometry_model, data, geometry_rest, options.camera_profile)
        torso = geometry_model.body('torso_link').id
        torso_transform = np.eye(4)
        torso_transform[:3, :3] = data.xmat[torso].reshape(3, 3)
        torso_transform[:3, 3] = data.xpos[torso]
        local = np.linalg.inv(torso_transform)@optical
        local[:3, :3] = local[:3, :3]@rotation(realism.camera_rpy_offset)
        local[:3, 3] += realism.camera_xyz_offset
        actual_optical = torso_transform@local
        fy = 240/math.tan(math.radians(fovy)/2)
        intrinsics = SimpleNamespace(width=640, height=480, fx=fy, fy=fy, cx=319.5, cy=239.5)
        row = inspect_layout(kin, rest, xy, settings, actual_optical, intrinsics,
                             preview_step_m=options.preview_step_m)
        row.update(seed=seed, camera_profile=options.camera_profile, realism=realism.summary())
        rows.append(row)
    return {'static_only': True, 'dynamic_success_rate': None, 'checks': rows,
            'nominal_bottle_xy_m': [options.bottle_x, options.bottle_y],
            'preview_step_m': options.preview_step_m,
            'model_sha256': hashlib.sha256(options.model.read_bytes()).hexdigest(),
            'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
            'source_sha256': {name: hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in (
                'Manipulation/push_button/bottle_layout.py', 'Manipulation/push_button/bottle.py',
                'Manipulation/push_button/kinematics.py', 'Manipulation/push_button/sim/run_realistic_bottle.py')},
            'ready': all(row['ready'] for row in rows),
            'limitations': ['Static rest pose and sampled mount/stance errors only; sway, arm occlusion, '
                            'RGB-D recognition, motor tracking and dynamic push success are untested.']}


def validate_replay(options, sources):
    if options.timing_mode != 'replay':
        return
    manifest = json.loads((options.timing_log/'manifest.json').read_text())
    comparable = ('camera_profile', 'profile', 'fault', 'gravity_scale', 'detector_imgsz',
                  'preview_step_m', 'track_table', 'dynamics_feedforward', 'stop_before_contact',
                  'bottle_x', 'bottle_y')
    for name in comparable:
        defaults = {'bottle_x': .33, 'bottle_y': -.25}
        if manifest['options'].get(name, defaults.get(name)) != getattr(options, name, defaults.get(name)):
            raise ValueError(f'replayの実行条件が一致しません: {name}')
    for source in sources:
        if manifest['source_sha256'].get(source) != hashlib.sha256((REPO/source).read_bytes()).hexdigest():
            raise ValueError(f'replayのソースが一致しません: {source}')
    for name in ('model', 'weights'):
        if manifest[f'{name}_sha256'] != hashlib.sha256(getattr(options, name).read_bytes()).hexdigest():
            raise ValueError(f'replayの{name}が一致しません')
    for seed in options.seeds:
        if not (options.timing_log/f'seed_{seed}'/'processing_times.json').is_file():
            raise ValueError(f'replayの処理時間がありません: seed {seed}')


class World:
    def __init__(self, seed, out, options):
        self.seed, self.out, self.options = seed, out, options
        self.r = sample_realism(seed, load_config('contest.yaml')['realism'])
        self.sensor = SensorModel(self.r)
        self.sensor.rng = np.random.default_rng([seed, 2, 1])
        self.settings = bottle.BottleSettings(.75)
        xy = bottle_world_xy(options, self.r)
        self.xy = xy
        m, self.rest = load_robot(options.model, (.110, -.003, 0), .79, .015,
                                 scene_builder=lambda root: build_scene(root, self.settings, *xy))
        d = mujoco.MjData(m)
        d.qpos[:29] = self.rest
        mujoco.mj_forward(m, d)
        self.nominal_optical, self.camera_fovy = camera_profile_geometry(m, d, self.rest, options.camera_profile)
        torso = m.body('torso_link').id
        torso_transform = np.eye(4)
        torso_transform[:3, :3] = d.xmat[torso].reshape(3, 3)
        torso_transform[:3, 3] = d.xpos[torso]
        local = np.linalg.inv(torso_transform) @ self.nominal_optical
        xml = out/'scene_base.xml'
        mujoco.mj_saveLastXML(str(xml), m)
        spec = mujoco.MjSpec.from_file(str(xml))
        pelvis = spec.body('pelvis')
        for i, name in enumerate(BASE):
            pelvis.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_SLIDE if i < 3 else mujoco.mjtJoint.mjJNT_HINGE,
                             axis=[int(j == i % 3) for j in range(3)], damping=400 if i < 3 else 50)
            a = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.gaintype, a.biastype = mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = 12000 if i < 3 else 1500
            a.biasprm[1] = -a.gainprm[0]
        quat = np.empty(4)
        actual_rotation = local[:3, :3] @ rotation(self.r.camera_rpy_offset) @ np.diag([1., -1., -1.])
        actual_local = local.copy()
        actual_local[:3, :3] = local[:3, :3] @ rotation(self.r.camera_rpy_offset)
        actual_local[:3, 3] += self.r.camera_xyz_offset
        self.calibrated_optical = torso_transform @ actual_local
        mujoco.mju_mat2Quat(quat, actual_rotation.ravel())
        spec.body('torso_link').add_camera(name='evaluation_rgbd',
            pos=local[:3, 3]+self.r.camera_xyz_offset, quat=quat, fovy=self.camera_fovy)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.render_data = mujoco.MjData(self.model)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(29)]
        self.act = np.array([self.model.actuator(n).id for n in names])
        self.qadr = np.array([self.model.joint(n).qposadr[0] for n in names])
        self.vadr = np.array([self.model.joint(n).dofadr[0] for n in names])
        self.base_act = np.array([self.model.actuator(n).id for n in BASE])
        self.base_qadr = np.array([self.model.joint(n).qposadr[0] for n in BASE])
        self.data.qpos[self.qadr] = self.rest
        self.data.qpos[self.base_qadr] = sway_offset(self.r, 0.)
        self.model.dof_armature[self.vadr] += self.r.armature
        self.model.dof_frictionloss[self.vadr] += self.r.friction_nm
        self.model.actuator_gainprm[self.act, 0] *= self.r.kp_scale
        self.model.actuator_biasprm[self.act, 1] *= self.r.kp_scale
        self.original_gain = self.model.actuator_gainprm[self.act].copy()
        self.original_bias = self.model.actuator_biasprm[self.act].copy()
        self.pelvis = self.model.body('pelvis').id
        self.z_ff = float(self.model.body_subtreemass[self.pelvis])*9.81/12000
        self.body = self.model.body('push_bottle').id
        shoulder = self.model.body('right_shoulder_pitch_link').id
        descendants = {shoulder}
        for b in range(shoulder+1, self.model.nbody):
            if int(self.model.body_parentid[b]) in descendants:
                descendants.add(b)
        self.arm_geoms = {g for g in range(self.model.ngeom) if int(self.model.geom_bodyid[g]) in descendants}
        self.table = self.model.geom('push_table').id
        self.tip = self.model.site('button_tcp').id
        self.tip_geom = self.model.geom('button_tip').id
        self.goal, self.weight = self.rest.copy(), 0.
        self.torque = np.zeros(7)
        self.queue, self.images = deque(), deque(maxlen=60)
        self.motion = self.arm = None
        self.monitor_active, self.bad_since = False, None
        self.next_tick = self.next_image = self.next_video = 0.
        self.frame_id, self.publish_count, self.read_count = 0, 0, 0
        self.phase, self.phases = 'setup', []
        self.costs = []
        self.perception, self.perception_busy, self.perception_cost = None, False, 0.
        self.replay_costs = {}
        self.replay_indices = {}
        if options.timing_mode == 'replay':
            rows = json.loads((options.timing_log/f'seed_{seed}'/'processing_times.json').read_text())
            for row in rows:
                self.replay_costs.setdefault(row['stage'], []).append(float(row['duration_s']))
        self.monitor_errors = []
        self.fault_events = []
        self.last_packet_id = -1
        self.decoded = None
        self.inference, self.detections = [], []
        self.max_table_force = self.max_self_force = self.max_bottle_force = self.max_tilt = 0.
        self.max_non_tip_bottle_force = self.max_other_force = 0.
        self.last_observed_t, self.observed = -1., None
        self.trace = []
        self.rgbd_renderer = mujoco.Renderer(self.model, height=480, width=640)
        self.video_renderer = mujoco.Renderer(self.model, height=360, width=640)
        self.writer = (cv2.VideoWriter(str(out/'motion.webm'), cv2.VideoWriter_fourcc(*'VP80'), 8, (640, 400))
                       if options.video else None)
        if self.writer is not None and not self.writer.isOpened():
            raise RuntimeError('Cannot open video encoder')
        self.initial_position = None
        self.origin = 0.
        mujoco.mj_forward(self.model, self.data)

    def now(self):
        return float(self.data.time)

    def charge(self, stage, measured):
        duration = measured
        if self.options.timing_mode == 'replay':
            index = self.replay_indices.get(stage, 0)
            values = self.replay_costs.get(stage, [])
            if index >= len(values):
                raise RuntimeError(f'保存した処理時間が不足しています: {stage} #{index}')
            duration = values[index]
            self.replay_indices[stage] = index+1
        if not np.isfinite(duration) or duration < 0:
            raise ValueError('処理時間は0以上の有限値が必要です')
        self.costs.append({'stage': stage, 'time_s': self.now()-self.origin+self.perception_cost if self.perception_busy else self.now()-self.origin,
                           'duration_s': duration, 'measured_duration_s': measured,
                           'worker': 'perception' if self.perception_busy else 'control'})
        if self.perception_busy:
            self.perception_cost += duration
        else:
            self.advance(duration)

    def observe(self):
        if self.options.fault == 'state-disconnect' and self.phase == 'approach' and self.now()-self.phases[-1]['time_s']-self.origin > 1.:
            self.injected('state-disconnect')
            raise RuntimeError('模擬状態通信断')
        if self.last_observed_t != self.now():
            q, dq = self.sensor.joints(self.data.qpos[self.qadr].copy(), self.data.qvel[self.vadr].copy())
            # Base joint velocities approximate the IMU's angular-rate reading.
            bvel = [self.data.qvel[self.model.joint(n).dofadr[0]] for n in BASE[3:]]
            self.observed = (q, rpy(self.data.xquat[self.pelvis]), self.now(),
                             float(np.max(np.abs(dq[:12]))), float(np.max(np.abs(bvel))))
            self.last_observed_t = self.now()
        return self.observed

    def injected(self, fault):
        if not self.fault_events:
            self.fault_events.append({'fault': fault, 'time_s': self.now()-self.origin, 'phase': self.phase})

    def phase_changed(self, name):
        self.phase = name
        self.phases.append({'phase': name, 'time_s': self.now()-self.origin})
        print(json.dumps({'seed': self.seed, **self.phases[-1]}), flush=True)
        self.record()

    def submit(self, right, weight, torque):
        goal = self.rest.copy() if self.arm._hold is None else np.r_[self.rest[:15], self.arm._hold]
        goal[22:29] = right
        self.queue.append((self.now()+self.r.command_latency_s, goal, float(weight), np.array(torque)))
        self.publish_count += 1

    def monitor(self):
        if not self.monitor_active or self.motion is None:
            return
        try:
            config = self.observe()
            if self.motion.state_sink is not None:
                self.motion.state_sink(config)
            align_button.validate_stationary_pose(config, self.motion.calibration)
            tracking = np.max(np.abs(config[0][22:29]-self.motion.current_target()))
            if tracking > sdk.TRACKING_LIMIT:
                self.bad_since = self.now() if self.bad_since is None else self.bad_since
                if self.now()-self.bad_since > .5:
                    raise RuntimeError('腕の追従誤差が継続しています')
            else:
                self.bad_since = None
            self.motion.publish(config, tuple(self.motion.current_target()), self.motion.weight)
        except Exception as exc:
            self.monitor_errors.append({'time_s': self.now()-self.origin, 'error': str(exc),
                'leg_speed_rad_s': None if self.observed is None else self.observed[3],
                'gyro_rad_s': None if self.observed is None else self.observed[4]})
            self.motion.error = exc
            self.monitor_active = False

    def advance(self, duration):
        end = self.now()+max(float(duration), self.model.opt.timestep)
        while self.now() < end-1e-9:
            if self.perception is not None:
                self.perception.tick()
            if self.now()-self.origin > 150:
                raise RuntimeError('Evaluation exceeded 150 simulated seconds')
            if self.now()+1e-9 >= self.next_tick:
                self.monitor()
                self.log_state()
                self.next_tick = self.now()+.02
            while self.queue and self.queue[0][0] <= self.now()+1e-9:
                _, self.goal, self.weight, self.torque = self.queue.popleft()
            self.data.ctrl[self.act] = self.goal
            # SDK gains and zero feedforward at weight=1; stand hold at weight=0.
            for j in range(15, 29):
                a = self.act[j]
                kp = (15. if j in (19, 20, 21, 26, 27, 28) else 40.)*self.r.kp_scale[j]
                self.model.actuator_gainprm[a, 0] = (1-self.weight)*self.original_gain[j, 0]+self.weight*kp
                self.model.actuator_biasprm[a, 1] = -self.model.actuator_gainprm[a, 0]
                self.model.actuator_biasprm[a, 2] = (1-self.weight)*self.original_bias[j, 2]-self.weight*1.5
            self.data.qfrc_applied[self.vadr[22:29]] = self.weight*self.torque
            base = sway_offset(self.r, self.now())
            base[2] += self.z_ff
            self.data.ctrl[self.base_act] = base
            mujoco.mj_step(self.model, self.data)
            tilt = math.degrees(math.acos(float(np.clip(self.data.xmat[self.body].reshape(3, 3)[2, 2], -1, 1))))
            self.max_tilt = max(self.max_tilt, tilt)
            for i, contact in enumerate(self.data.contact):
                pair = {int(contact.geom1), int(contact.geom2)}
                if not pair.intersection(self.arm_geoms):
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                n = float(np.linalg.norm(force[:3]))
                if self.table in pair:
                    self.max_table_force = max(self.max_table_force, n)
                elif any(self.model.geom_bodyid[g] == self.body for g in pair):
                    self.max_bottle_force = max(self.max_bottle_force, n)
                    if self.tip_geom not in pair:
                        self.max_non_tip_bottle_force = max(self.max_non_tip_bottle_force, n)
                elif all(self.model.geom_bodyid[g] != 0 for g in pair):
                    self.max_self_force = max(self.max_self_force, n)
                else:
                    self.max_other_force = max(self.max_other_force, n)
            if self.now()+1e-9 >= self.next_image:
                self.capture()
                self.next_image = self.now()+1/self.r.image_fps
            if self.writer is not None and self.now()+1e-9 >= self.next_video:
                self.record()
                self.next_video = self.now()+1/8

    def capture(self):
        # Save exposure-time physics at the configured FPS. Render only delivered
        # frames: discarded frames cannot affect control or the physics trajectory.
        self.frame_id += 1
        self.images.append((self.now(), self.frame_id, self.data.qpos.copy(), self.data.qvel.copy(),
                            rpy(self.data.xquat[self.pelvis]).copy()))

    def render_packet(self, row, protocol):
        stamp, idx, qpos, qvel, imu = row
        if idx == self.last_packet_id:
            return self.decoded
        data = self.render_data
        data.qpos[:], data.qvel[:], data.time = qpos, qvel, stamp
        mujoco.mj_forward(self.model, data)
        self.rgbd_renderer.update_scene(data, camera='evaluation_rgbd')
        noise = SensorModel(self.r)
        noise.rng = np.random.default_rng([self.seed, 2, 2, idx])
        rgb = noise.color(self.rgbd_renderer.render().copy())
        self.rgbd_renderer.enable_depth_rendering()
        depth = noise.depth(self.rgbd_renderer.render().copy())
        self.rgbd_renderer.disable_depth_rendering()
        fy = 240/math.tan(math.radians(self.camera_fovy)/2)
        frame = protocol.RgbdFrame(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            np.round(depth*1000).clip(0, 65535).astype(np.uint16), .001,
            protocol.Intrinsics(640, 480, fy, fy, 319.5, 239.5), 1800000000.+stamp, idx)
        packet = protocol.encode_rgbd(frame)
        self.decoded = protocol.decode_rgbd(packet)
        self.last_packet_id = idx
        frame_dir = self.out/'frames'
        frame_dir.mkdir(exist_ok=True)
        (frame_dir/f'{idx:06d}.rgbd').write_bytes(packet)
        dump(frame_dir/f'{idx:06d}.json', {'captured_at_s': stamp, 'received_at_s': self.now(),
             'motor_q_rad': qpos[self.qadr].tolist(), 'imu_rpy_rad': imu.tolist()})
        return self.decoded

    def record(self):
        self.video_renderer.update_scene(self.data, camera='bottle_demo')
        image = cv2.cvtColor(self.video_renderer.render(), cv2.COLOR_RGB2BGR)
        canvas = np.zeros((400, 640, 3), np.uint8)
        canvas[:360] = image
        label = f'seed {self.seed} | {self.phase} | t={self.now()-self.origin:.2f}s'
        cv2.putText(canvas, label, (10, 385), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
        if self.writer is not None:
            self.writer.write(canvas)
        cv2.imwrite(str(self.out/'final.jpg'), canvas)
        self.log_state()

    def log_state(self):
        self.trace.append({'time_s': self.now()-self.origin, 'phase': self.phase,
                           'q': self.data.qpos[self.qadr].tolist(), 'command': self.goal.tolist(),
                           'weight': self.weight, 'bottle_position': self.data.xpos[self.body].tolist()})
        self.trace[-1]['right_tau_nm'] = self.torque.tolist()
        self.trace[-1]['imu_rpy_rad'] = rpy(self.data.xquat[self.pelvis]).tolist()
        self.trace[-1]['evaluation_pelvis_xyz_m'] = self.data.xpos[self.pelvis].tolist()
        self.trace[-1]['evaluation_tip_xyz_m'] = self.data.site_xpos[self.tip].tolist()

    def dispose(self):
        self.record()
        if self.writer is not None:
            self.writer.release()
        self.rgbd_renderer.close()
        self.video_renderer.close()
        dump(self.out/'states.json', self.trace)


def run(seed, out, options):
    out.mkdir(parents=True, exist_ok=True)
    world = World(seed, out, options)
    original = (push_bottle.ArmSdk, push_bottle.ArmMotion, push_bottle.J1RgbdCamera,
                push_bottle.make_bottle_detector, push_bottle.BottlePushRoutine,
                push_bottle.time, align_button.time, j1gen_camera.time, sdk.time, builtins.input,
                push_bottle.make_perception_worker)
    original_motion = push_bottle.ArmMotion
    virtual = SimpleNamespace(monotonic=world.now, time=lambda: 1800000000.+world.now(), sleep=world.advance)
    runtime = j1gen_camera.J1CameraRuntime()
    protocol = runtime.module('rgbd_protocol')
    detector = bottle.make_bottle_detector(str(options.weights), .25, 'cpu', runtime=runtime, imgsz=options.detector_imgsz)
    # Warm up model before the simulation starts. Timed inference remains active in trials.
    detector.detect(np.zeros((480, 640, 3), np.uint8))
    import torch
    torch.set_num_threads(2)

    class Arm(sdk.ArmSdk):
        def __init__(self, interface):
            self._hold = self._last_right = None
            self._last_weight = 0.
            world.arm = self
        def current_arm(self, timeout=5.):
            return tuple(world.observe()[0][15:29])
        def current_configuration(self):
            return world.observe()
        def stop_walking(self):
            world.advance(1.)
        def publish(self, right, weight, require_fresh_state=True, *, right_tau=None):
            sdk.validate_arm_q(right)
            if self._hold is None:
                raise RuntimeError('Missing hold configuration')
            torque = validate_arm_tau((0.,)*7 if right_tau is None else right_tau)
            if weight == 0:
                torque = (0.,)*7
            world.submit(right, weight, torque)
            self._last_right, self._last_weight = tuple(right), float(weight)
            self._last_right_tau = torque
        def close(self):
            self.release()

    class Motion(original_motion):
        def __init__(self, arm, calibration, **kwargs):
            super().__init__(arm, calibration, **kwargs)
            world.motion = self
        def enable(self):
            world.monitor_active = True
            for step in range(101):
                self.check()
                self.weight = step/100.
                self.wait(.02)
        def wait(self, seconds):
            self.check()
            world.advance(seconds)
            self.check()
        def close(self):
            world.monitor_active = False
            super().close()

    class Source:
        def read_rgbd(self, timeout_ms=None):
            if timeout_ms != 0:
                world.advance(.02)
            available = [row for row in world.images if row[0] <= world.now()-world.r.image_latency_s]
            if not available:
                return None
            frame = world.render_packet(available[-1], protocol)
            if options.fault == 'stale-images' and world.phase == 'remeasure':
                world.injected('stale-images')
                from dataclasses import replace
                frame = replace(frame, timestamp=frame.timestamp-1.)
            return frame
        def close(self):
            pass

    def camera(*args):
        cam = j1gen_camera.J1RgbdCamera.__new__(j1gen_camera.J1RgbdCamera)
        cam.source, cam.max_age_s = Source(), .5
        cam.last_id, cam.last_timestamp, cam.last = -1, -np.inf, None
        return cam

    class Detector:
        def detect(self, image):
            if world.arm is None:
                # The production entrypoint also warms up before opening the SDK.
                # Keep initialization outside trial costs and the initial RGB artifact.
                return detector.detect(image)
            start = wall_time.perf_counter()
            if options.fault == 'target-loss' and world.phase == 'remeasure':
                world.injected('target-loss')
            detections = ([] if options.fault == 'target-loss' and world.phase == 'remeasure'
                          else detector.detect(image))
            duration = wall_time.perf_counter()-start
            world.inference.append(duration)
            world.detections.append({'time_s': world.now()-world.origin,
                'detections': [{'class': d.class_name, 'confidence': d.confidence, 'bbox': list(d.bbox)} for d in detections]})
            if len(world.detections) == 1:
                cv2.imwrite(str(out/'initial_rgb.jpg'), image)
            world.charge('detector', duration)
            return detections

    class Routine(bottle.BottlePushRoutine):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.preview_step_m = options.preview_step_m
        def phase(self, name):
            world.phase_changed(name)
            super().phase(name)

    code, harness_error = None, None
    timing_patches = []
    def timed(name, function):
        def call(*args, **kwargs):
            started = wall_time.perf_counter()
            try:
                return function(*args, **kwargs)
            finally:
                elapsed = wall_time.perf_counter()-started
                world.charge(name, elapsed)
        return call
    start_wall = wall_time.perf_counter()
    try:
        push_bottle.time = align_button.time = j1gen_camera.time = sdk.time = virtual
        push_bottle.ArmSdk, push_bottle.ArmMotion = Arm, Motion
        push_bottle.J1RgbdCamera = camera
        push_bottle.make_bottle_detector = lambda *args, **kwargs: Detector()
        push_bottle.BottlePushRoutine = Routine
        push_bottle.make_perception_worker = lambda engine, now, wait: VirtualPerception(engine, now, wait, world)
        for owner, name in [(bottle, 'estimate_bottle_target'), (ArmKinematics, 'solve'),
                            (ArmKinematics, 'validate_path'),
                            (CartesianJointController, 'next_goal')]:
            p = patch.object(owner, name, timed(name, getattr(owner, name)))
            p.start()
            timing_patches.append(p)
        from table_reference import TableReference
        p = patch.object(TableReference,'update',timed('table_reference',TableReference.update))
        p.start()
        timing_patches.append(p)
        builtins.input = lambda prompt='': 'YES'  # Only reachable with the in-process fake SDK.
        world.advance(.5)
        world.initial_position = world.data.xpos[world.body].copy()
        world.origin = world.now()
        optical = world.calibrated_optical if options.profile == 'calibrated' else world.nominal_optical
        uncertainty = .05 if options.profile == 'miscalibrated' else .001
        calibration = {'schema': 'g1-button-calibration-v1', 'camera_body': 'torso_link',
            'T_base_optical': optical.tolist(), 'tip_offset_m': [.110, -.003, 0],
            'tip_radius_m': .015, 'pelvis_height_m': .79, 'position_uncertainty_m': uncertainty,
            'reference_waist_q_rad': world.rest[12:15].tolist(), 'reference_imu_rpy_rad': [0, 0, 0]}
        dump(out/'calibration.json', calibration)
        command = ['--model', str(options.model), '--weights', str(options.weights),
            '--base-transform', str(out/'calibration.json'), '--table-height', '.75',
            '--gravity-scale', str(options.gravity_scale), '--detector-imgsz', str(options.detector_imgsz),
            '--execute', '--calibrated', '--output', str(out/'controller.json'), '--network-interface', 'SIMULATION_ONLY']
        if options.track_table:
            command.append('--track-table')
        if options.dynamics_feedforward:
            command.append('--dynamics-feedforward')
        if options.stop_before_contact:
            command.append('--stop-before-contact')
        code = push_bottle.main(command)
        world.advance(.1)  # Deliver the final zero-weight command.
    except Exception as exc:
        import traceback
        harness_error = traceback.format_exc()
        print(harness_error, flush=True)
    finally:
        for p in reversed(timing_patches):
            p.stop()
        (push_bottle.ArmSdk, push_bottle.ArmMotion, push_bottle.J1RgbdCamera,
         push_bottle.make_bottle_detector, push_bottle.BottlePushRoutine,
         push_bottle.time, align_button.time, j1gen_camera.time, sdk.time, builtins.input,
         push_bottle.make_perception_worker) = original
    report = json.loads((out/'controller.json').read_text()) if (out/'controller.json').exists() else {}
    displacement = world.data.xpos[world.body]-world.initial_position if world.initial_position is not None else np.zeros(3)
    tilt = math.degrees(math.acos(float(np.clip(world.data.xmat[world.body].reshape(3, 3)[2, 2], -1, 1))))
    home_error = float(np.max(np.abs(world.data.qpos[world.qadr][22:29]-world.motion.home))) if world.motion else None
    success = bool(code == 0 and report.get('completed') and report.get('visual_success') is True
        and abs(displacement[0]-.05) <= .01 and abs(displacement[1]) <= .01 and tilt <= 15
        and world.max_table_force < .1 and world.max_self_force < .1
        and world.max_non_tip_bottle_force < .1 and world.max_other_force < .1
        and home_error is not None and home_error <= .05 and world.weight == 0. and np.all(world.torque == 0))
    result = {'seed': seed, 'set': 'bottle_realistic', 'profile': options.profile,
        'camera_profile': options.camera_profile, 'timing_mode': options.timing_mode,
        'fault': options.fault, 'exit_code': code, 'success': success,
        'realism': world.r.summary(), 'bottle_initial_xy_m': world.xy.tolist(),
        'simulation_duration_s': world.now()-world.origin, 'wall_duration_s': wall_time.perf_counter()-start_wall,
        'phases': world.phases, 'controller': report, 'harness_error': harness_error,
        'monitor_errors': world.monitor_errors,
        'fault_events': world.fault_events,
        'evaluation': {'ground_truth_used_for_control': False, 'bottle_displacement_m': displacement.tolist(),
            'bottle_final_tilt_deg': tilt, 'bottle_max_tilt_deg': world.max_tilt,
            'max_arm_table_contact_n': world.max_table_force, 'max_self_contact_n': world.max_self_force,
            'max_non_tip_bottle_contact_n': world.max_non_tip_bottle_force,
            'max_other_arm_contact_n': world.max_other_force,
            'max_arm_bottle_contact_n': world.max_bottle_force, 'home_joint_error_rad': home_error,
            'released': world.weight == 0. and bool(np.all(world.torque == 0)), 'publish_count': world.publish_count,
            'inference_count': len(world.inference), 'max_inference_s': max(world.inference, default=0)},
        'artifacts': {'video': str(out/'motion.webm'), 'final': str(out/'final.jpg')}}
    if options.stop_before_contact and options.profile == 'calibrated' and options.fault == 'none':
        expected = (code == 0 and report.get('diagnostic_completed') is True
                    and report.get('home_completed') is True and not report.get('motion_completed')
                    and float(np.linalg.norm(displacement)) < .001 and world.max_bottle_force < .1)
    elif options.profile == 'miscalibrated':
        expected = code == 2 and '不確かさ' in report.get('error','') and world.publish_count == 0
    elif options.fault == 'unreachable':
        expected = code != 0 and any(word in report.get('error','') for word in ('IK','到達','範囲')) and world.publish_count == 0
    elif options.fault in ('target-loss', 'stale-images'):
        expected = bool(world.fault_events) and code != 0 and bool(report.get('error')) and report.get('home_completed') is True
    elif options.fault == 'state-disconnect':
        expected = (bool(world.fault_events) and code != 0 and '模擬状態通信断' in report.get('error','')
                    and not report.get('home_completed')
                    and not any(p['phase'] in ('retract','abort_return','home') for p in world.phases))
    else:
        expected = success
    result['expected_outcome'] = bool(expected and harness_error is None and result['evaluation']['released']
        and world.max_table_force < .1 and world.max_self_force < .1)
    result['expected_outcome'] &= world.max_non_tip_bottle_force < .1 and world.max_other_force < .1
    world.dispose()
    dump(out/'result.json', result)
    dump(out/'detections.json', world.detections)
    dump(out/'processing_times.json', world.costs)
    dump(out/'realism_full.json', {key: value.tolist() if isinstance(value, np.ndarray) else value
         for key, value in asdict(world.r).items()})
    print(json.dumps({'seed': seed, 'success': success, 'error': report.get('error'),
        'last_phase': report.get('phase'), 'displacement_mm': (displacement*1000).round(3).tolist()}), flush=True)
    return result


def build_argument_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(5)))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--model', type=Path, default=MODEL)
    parser.add_argument('--weights', type=Path, default=WEIGHTS)
    parser.add_argument('--profile', choices=['calibrated', 'miscalibrated', 'uncalibrated'], default='calibrated')
    parser.add_argument('--gravity-scale', type=float, default=1.)
    parser.add_argument('--detector-imgsz', type=int, default=960)
    parser.add_argument('--preview-step-m', type=float, default=.005, help='事前検査の刻み幅。比較試験のみ変更する')
    parser.add_argument('--fault', choices=['none', 'target-loss', 'stale-images', 'unreachable', 'state-disconnect'], default='none')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--camera-profile', choices=['legacy', 'head'], default='legacy')
    parser.add_argument('--timing-mode', choices=['measured', 'replay'], default='measured')
    parser.add_argument('--timing-log', type=Path, help='replay用の過去の評価出力ディレクトリ')
    parser.add_argument('--stop-before-contact', action='store_true')
    parser.add_argument('--bottle-x', type=float, default=.33, help='立ち位置の乱れ適用前のボトル中心X[m]')
    parser.add_argument('--bottle-y', type=float, default=-.25, help='立ち位置の乱れ適用前のボトル中心Y[m]')
    parser.add_argument('--check-layout', action='store_true', help='描画・物理ステップなしの配置/IK検査だけを行う')
    parser.add_argument('--track-table', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--dynamics-feedforward',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--min-successes', type=int, help='必要な期待結果達成数。省略時は全試行')
    return parser


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    args.model, args.weights = args.model.resolve(), args.weights.resolve()
    if args.timing_mode == 'replay' and (args.timing_log is None or not args.timing_log.is_dir()):
        parser.error('replayには既存の--timing-logディレクトリが必要です')
    try:
        validate_layout_xy(args.bottle_x, args.bottle_y, bottle.BottleSettings(.75))
    except ValueError as exc:
        parser.error(str(exc))
    if args.check_layout and (args.timing_mode != 'measured' or args.fault != 'none' or args.video
                             or args.stop_before_contact):
        parser.error('--check-layoutはreplay・故障注入・録画・接触前診断と併用できません')
    if args.out.exists():
        parser.error('既存の試験結果は上書きしません。新しい--outを指定してください')
    if not np.isfinite(args.preview_step_m) or args.preview_step_m <= 0:
        parser.error('preview-step-mは正の有限値が必要です')
    if args.check_layout:
        result = check_layouts(args)
        args.out.mkdir(parents=True)
        dump(args.out/'layout_checks.json', result)
        return 0 if result['ready'] else 1
    import torch
    torch.set_num_threads(2)
    paths = ['Manipulation/push_button/bottle.py', 'Manipulation/push_button/real/push_bottle.py',
             'Manipulation/push_button/real/align_button.py', 'Manipulation/push_button/alignment.py',
             'Manipulation/push_button/real/j1gen_camera.py', 'Manipulation/push_button/real/push_button.py',
             'Button_Press/Yada/common/realism.py', 'Button_Press/Yada/configs/contest.yaml']
    paths += ['Manipulation/push_button/gravity.py', 'Manipulation/push_button/kinematics.py',
              'Manipulation/push_button/trajectory.py', 'Manipulation/push_button/real/arm_bridge.py',
              'Manipulation/push_button/cartesian.py', 'Manipulation/push_button/pose_history.py',
              'Manipulation/push_button/table_reference.py', 'Manipulation/push_button/return_path.py',
              'Button_Press/J1-gen/common/camera_rgbd.py', 'Button_Press/J1-gen/common/rgbd_protocol.py',
              'Perception/common/detector/yolo_detector.py', 'Manipulation/push_button/sim/run_realistic_bottle.py']
    paths += ['Manipulation/push_button/bottle_perception.py', 'Button_Press/J1-gen/configs/robot.yaml',
              'Button_Press/J1-gen/common/camera_geometry.py', 'Manipulation/push_button/bottle_layout.py',
              'Manipulation/push_button/sim/run_bottle_mujoco.py']
    try:
        validate_replay(args, paths)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    args.out.mkdir(parents=True)
    for source in paths:
        destination = args.out/'source'/source
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((REPO/source).read_bytes())
    manifest = {'seeds': args.seeds, 'mujoco': mujoco.__version__, 'numpy': np.__version__,
        'source_sha256': {p: hashlib.sha256((REPO/p).read_bytes()).hexdigest() for p in paths},
        'model_sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
        'weights_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        'git_status_before': subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=REPO, text=True),
        'host': {'cpu_count': os.cpu_count(), 'affinity': sorted(os.sched_getaffinity(0)),
                 'python': sys.version, 'executable': sys.executable,
                 'preloaded_triton': getattr(sys.modules.get('triton'), '__version__', None),
                 'render_environment': {k: os.environ.get(k) for k in ('MUJOCO_GL', 'LIBGL_ALWAYS_SOFTWARE',
                     '__EGL_VENDOR_LIBRARY_FILENAMES', 'CUDA_VISIBLE_DEVICES')},
                 'load_average': list(os.getloadavg()), 'torch_threads': torch.get_num_threads(),
                 'thread_environment': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}},
        'options': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        'method': 'Production real/push_bottle.main with in-process ArmSdk transport, calibrated camera geometry, RGB-D wire format and validation, YOLO and BottlePushRoutine. Virtual scheduling replaces the ArmMotion thread and perception worker; interpolation, monitoring, torque and release are reused. Sensor jobs expose completed outputs after their measured or replayed costs while physics and 50Hz publication continue. Control computations advance physics independently. Only consumed frames are rendered, with frame-indexed and joint-sensor noise streams.',
        'motor_profile': {'major_joint_kp': 40, 'wrist_kp': 15, 'kd': 1.5, 'gravity_scale': args.gravity_scale,
            'kp_scale': [.9, 1.1], 'armature': 'Menagerie baseline plus sampled Yada increment',
            'friction': 'Menagerie baseline plus sampled Yada increment',
            'release': 'Production 101-step ramp; weight=0 stand hold approximates proprietary robot behavior'},
        'limitations': ['Adaptation of Yada realistic disturbance distribution to the bottle scene; not the Yada button benchmark.',
            'legacy uses the old virtual viewpoint; head uses J1-gen model mounting and field of view. Neither validates measured physical calibration.',
            'Base sway is driven by six compliant servos; free-standing balance and walking are not modeled.',
            'Bottle is rigid; PET deformation and transparent-surface depth physics are not modeled.',
            'SDK/DDS and camera networking are replaced in process; RGB-D packet encoding/decoding is exercised.',
            'The calibrated profile supplies the known synthetic camera extrinsics; it does not validate a real calibration procedure.',
            'Thread scheduling uses simulation time. Computation durations are measured on this host and are not deterministic.']}
    dump(args.out/'manifest.json', manifest)
    results = []
    for seed in args.seeds:
        print(f'START seed={seed}', flush=True)
        result = run(seed, args.out/f'seed_{seed}', args)
        results.append(result)
        dump(args.out/'results.json', results)
    passed = sum(r['expected_outcome'] for r in results)
    required = len(results) if args.min_successes is None else args.min_successes
    summary = {'successes': sum(r['success'] for r in results), 'expected_outcomes': passed,
               'trials': len(results), 'required': required, 'passed': passed >= required}
    dump(args.out/'summary.json', summary)
    print(json.dumps(summary), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
