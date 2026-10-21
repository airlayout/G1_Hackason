"""Record comparable realistic trials with wrist-servo diagnostics and raw state logs."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import html
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import cv2
import mujoco
import numpy as np

from Manipulation.wrist_press import evaluate
from Manipulation.wrist_press.runtime import enable_yada
from Manipulation.wrist_press.settings import load_settings
from Manipulation.wrist_press.simulation import make_mujoco_robot


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=plain) + '\n')


class Recorder:
    def __init__(self, robot, trial, out):
        self.robot, self.trial, self.out = robot, trial, out
        out.mkdir(parents=True, exist_ok=True)
        self.renderer = mujoco.Renderer(robot.model, height=480, width=640)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.20, -.16, 1.12]
        self.camera.distance, self.camera.azimuth, self.camera.elevation = 1.12, 75, -12
        self.writer = cv2.VideoWriter(str(out/'motion.webm'), cv2.VideoWriter_fourcc(*'VP90'), 10, (960, 656))
        if not self.writer.isOpened():
            self.renderer.close()
            raise RuntimeError('VP9 writer could not be opened')
        self.states = gzip.open(out/'states.jsonl.gz', 'wt')
        self.frames, self.next_t, self.phase = [], 0., None
        self.controller = None

    @staticmethod
    def label(image, text, x, y, size=.55, color=(240,244,248)):
        cv2.putText(image, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)

    def observe_control_state(self, controller, obs, command):
        self.controller = controller
        state = {'t':obs.t, 'phase':command.phase, 'q_observed':obs.q, 'dq_observed':obs.dq,
                 'q_command':command.q, 'tau_ff':command.tau_ff, 'error':command.error,
                 'qpos':self.robot.data.qpos.copy(), 'qvel':self.robot.data.qvel.copy(),
                 'ctrl':self.robot.data.ctrl.copy(), 'sim_time':self.robot.data.time,
                 'target':asdict(controller.target) if controller.target is not None else None,
                 'frozen_target':asdict(controller.freeze_target) if controller.freeze_target is not None else None,
                 'alignment_bias':controller.alignment_bias.copy(),
                 'wrist_servo':controller.servo_diagnostic,
                 'alignment_camera':controller.report.alignment_camera,
                 'surface_camera':controller.report.surface_camera,
                 'perception_error':controller.vision.error,
                 'score_diag':self.robot.hall.diagnostics(),
                 'lit':[b.name for b in self.robot.scene.buttons if self.robot.hall.lit(b.name)]}
        self.states.write(json.dumps(state, default=plain) + '\n')

    def annotated_wrist(self, frame):
        image = frame.rgb.copy()
        c = self.controller
        if c is None:
            return image
        # The observed box and computed fingertip have different provenance.
        candidates = c.wrist_filter.items
        observed = candidates[-1] if candidates else c.target
        if (observed is not None and observed.camera == "wrist" and len(observed.bbox) == 4
                and -.02 <= frame.captured_at-observed.captured_at <= c.cfg['max_image_age_s']):
            box = observed.bbox
            center = tuple(int(round(v)) for v in ((box[0]+box[2])/2, (box[1]+box[3])/2))
            cv2.circle(image, center, 5, (80, 240, 160), 2)
            self.label(image, 'BUTTON / IMAGE', 5, image.shape[0]-27, .35, (80, 240, 160))
        point = c.robot.tip(frame.q)  # Exposure-time joints, never simulator tip truth.
        T = frame.T_pelvis_optical
        optical = T[:3, :3].T @ (point-T[:3, 3])
        if optical[2] > .001:
            pixel = frame.K @ optical
            u, v = pixel[:2]/pixel[2]
            if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
                cv2.drawMarker(image, (int(u), int(v)), (240, 120, 230), cv2.MARKER_CROSS, 13, 2)
        self.label(image, 'TIP / JOINT COMPUTATION', 5, image.shape[0]-10, .35, (240, 120, 230))
        return image

    def capture(self, obs, command):
        if command.phase != self.phase:
            prefix = f'{len(self.frames):04d}_{command.phase}'
            cv2.imwrite(str(self.out/f'{prefix}_head.png'), cv2.cvtColor(obs.rgb, cv2.COLOR_RGB2BGR))
            arrays = {'q':obs.q, 'dq':obs.dq, 'K':obs.K, 'head_depth':obs.depth,
                      'head_image_t':np.array(obs.image_t)}
            if obs.wrist is not None:
                cv2.imwrite(str(self.out/f'{prefix}_wrist.png'), cv2.cvtColor(obs.wrist.rgb,cv2.COLOR_RGB2BGR))
                arrays.update(wrist_depth=obs.wrist.depth, wrist_K=obs.wrist.K,
                              wrist_T=obs.wrist.T_pelvis_optical, wrist_q=obs.wrist.q,
                              wrist_image_t=np.array(obs.wrist.captured_at))
            np.savez_compressed(self.out/f'{prefix}_sensors.npz', **arrays)
            self.phase = command.phase
        if obs.t+1e-6 < self.next_t and not command.done:
            return
        self.next_t = obs.t+.1
        self.renderer.update_scene(self.robot.data, camera=self.camera)
        canvas = np.full((656,960,3),(20,25,33),np.uint8)
        canvas[44:524,:640] = self.renderer.render()
        canvas[44:284,640:] = cv2.resize(obs.rgb,(320,240),interpolation=cv2.INTER_AREA)
        if obs.wrist is not None:
            canvas[284:524,640:] = cv2.resize(self.annotated_wrist(obs.wrist),(320,240),interpolation=cv2.INTER_AREA)
        lit = [b.name for b in self.robot.scene.buttons if self.robot.hall.lit(b.name)]
        self.label(canvas,f'YADA REALISTIC / SEED {self.trial.seed}',16,29,.65)
        self.label(canvas,f'SIM {obs.t:5.2f} s / 1x',690,29)
        for x,y,width,title in [(0,44,640,'MUJOCO / OVERVIEW'),(640,44,320,'HEAD RGB'),(640,284,320,'WRIST RGB')]:
            cv2.rectangle(canvas,(x,y),(x+width,y+28),(20,25,33),-1)
            self.label(canvas,title,x+12,y+20)
        self.label(canvas,f'PHASE: {command.phase.upper()}',16,550,.65)
        self.label(canvas,f'TARGET: {self.trial.target.upper()} / LIGHT: {"ON" if self.trial.target in lit else "OFF"}',385,550,.65,
                   (115,235,175) if self.trial.target in lit else (240,244,248))
        diag = self.controller.servo_diagnostic if self.controller else {}
        enabled = self.controller.wrist_servo_enabled if self.controller else False
        status = diag.get('status', 'legacy alignment')
        self.label(canvas, f'SERVO {"ON" if enabled else "OFF"} / {status}'[:108],16,577,.48)
        error = diag.get('error_vector_m')
        step = diag.get('correction_vector_m')
        metrics = 'ALIGN ERROR -- / STEP --'
        if error is not None:
            metrics = f'ALIGN ERROR {1000*np.linalg.norm(error):.2f} mm / STEP {1000*np.linalg.norm(step):.2f} mm'
        sources = self.controller.report if self.controller else None
        self.label(canvas, metrics + f' / ALIGN {sources.alignment_camera if sources else None} / SURFACE {sources.surface_camera if sources else None}',16,603,.45)
        self.label(canvas,(command.error or 'Actual simulation / tip is computed from joints / no target truth in control')[:119],16,630,.44)
        bgr = cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR)
        self.writer.write(bgr)
        cv2.imwrite(str(self.out/'poster.jpg'),bgr,[cv2.IMWRITE_JPEG_QUALITY,82])
        self.frames.append({'t':round(obs.t,4),'phase':command.phase,'lit':lit,
                            'head_image_t':float(obs.image_t),
                            'wrist_image_t':float(obs.wrist.captured_at) if obs.wrist else None,
                            'wrist_servo':diag})

    def close(self):
        self.writer.release()
        self.renderer.close()
        self.states.close()
        save(self.out/'frames.json',{'fps':10,'frames':self.frames})


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',required=True)
    ap.add_argument('--config')
    ap.add_argument('--servo',choices=['on','off'],required=True)
    ap.add_argument('--seeds',type=int,nargs='+',default=list(range(1000,1020)))
    args=ap.parse_args()
    out=Path(args.out).resolve()
    out.mkdir(parents=True,exist_ok=True)
    if (out/'results.json').exists():
        raise RuntimeError('Refusing to overwrite an existing batch')
    cfg=load_settings(args.config,mode='button',detector='yolo')
    cfg['wrist_servo']['enabled'] = args.servo == 'on'
    enable_yada()
    from common.config import load_config
    from contest.interface import TaskInfo
    from contest.task import make_trial
    contest_cfg=load_config('contest.yaml')
    hash_paths=sorted(Path('Manipulation/wrist_press').rglob('*.py'))+[Path('Manipulation/wrist_press/config.yaml'),Path(cfg['button_weights'])]
    metadata={'started_at_utc':datetime.now(timezone.utc).isoformat(),'seeds':args.seeds,'eval_set':'realistic',
              'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'runtime':json.loads(Path('_local/wrist_press/current.json').read_text()),
              'packages':{p:importlib.metadata.version(p) for p in ['mujoco','pin','coal','ultralytics','numpy','torch']},
              'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in hash_paths},
              'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'original_batch':'_local/wrist_press/results/realistic_20_20261021/results.json',
              'servo_enabled':cfg['wrist_servo']['enabled'], 'config':cfg,
              'recording':'Actual repeated simulations, 10 fps at simulation speed; no replay.',
              'diagnostics':'Controller state is read after act; simulator scores are logged only, never supplied to controller.',
              'limitations':['Existing full-cycle wrapper uses exact joint state and ideal wrist images.',
                             'Recording adds wall-clock overhead; planner has wall-clock time budgets.']}
    save(out/'metadata.json',metadata)
    results=[]
    started=time.perf_counter()
    for seed in args.seeds:
        print(f'[batch] START {len(results)+1}/{len(args.seeds)} seed={seed}',flush=True)
        trial=make_trial(seed,contest_cfg,eval_set='realistic')
        episode=out/str(seed)
        robot=make_mujoco_robot(trial.scene_cfg,contest_cfg,cfg,realism=trial.realism)
        recorder=Recorder(robot,trial,episode)
        try:
            kp,kd=robot.upper_gains()
            task=TaskInfo(trial.instruction,trial.target,'mujoco',cfg['cycle_timeout_s'],.02,False,kp,kd)
            result=evaluate.run_episode(robot,cfg,task,recorder=recorder)
            result.update(seed=seed,sim='mujoco',eval_set='realistic',recorded=True,
                          realism=trial.realism.summary(),conditions=trial.params,
                          instruction=trial.instruction,video=f'{seed}/motion.webm')
        except Exception:
            result={'seed':seed,'target':trial.target,'success':False,'physical_press_success':False,
                    'returned':False,'error':traceback.format_exc(),'recording_error':True,
                    'sim':'mujoco','eval_set':'realistic','video':f'{seed}/motion.webm'}
        finally:
            recorder.close()
            robot.close()
        save(episode/'result.json',result)
        results.append(result)
        payload={'config':cfg,'summary':{'trials':len(results),'successes':sum(r['success'] for r in results),
                 'physical_presses':sum(r['physical_press_success'] for r in results),
                 'returned':sum(r['returned'] for r in results),'wall_time_s':time.perf_counter()-started},'results':results}
        save(out/'results.json',payload)
        write_gallery(out, results)
        print('[batch] RESULT '+json.dumps({k:v for k,v in result.items() if k not in ('trace','perception_ms','planning_ms','realism','conditions','alignment_diagnostics')},ensure_ascii=False),flush=True)
    metadata['finished_at_utc']=datetime.now(timezone.utc).isoformat()
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in metadata['sha256'].items())
    save(out/'metadata.json',metadata)
    print('[batch] COMPLETE '+json.dumps(payload['summary']),flush=True)


def write_gallery(out, results):
    cards=[]
    for r in results:
        seed=r['seed']
        title=html.escape(f"{seed}: {'成功' if r['success'] else r.get('error') or '復帰したが不点灯'}")
        cards.append(f'<article><h2>{title}</h2><video controls preload="none" poster="{seed}/poster.jpg" src="{seed}/motion.webm"></video><p><a href="{seed}/result.json">結果・診断</a> / <a href="{seed}/states.jsonl.gz">状態ログ</a></p></article>')
    (out/'index.html').write_text('<!doctype html><html lang="ja"><meta charset="utf-8"><title>realistic 手首サーボ評価</title><style>body{font:16px system-ui;background:#131923;color:#eee;margin:24px}article{margin:24px 0}video{width:min(960px,100%)}a{color:#8ccaff}</style><h1>realistic 手首サーボ評価</h1><p>緑丸＝画像のボタン中心、紫十字＝関節角から計算した指先。画像上の両者の重なりを直接の制御目標にはしていません。映像の時刻はシミュレーション時間です。</p>'+''.join(cards)+'</html>')


if __name__=='__main__':
    main()
