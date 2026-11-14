"""Production staged controllers in the same RGB-D/B-geometry fixture.

Only the bottle table-outline guard is replaced by the shared B geometry, so the
comparison isolates stopping/remeasuring and planning rather than table sensing.
"""
from collections import Counter
from types import SimpleNamespace
import numpy as np

from trajectory import interpolate
from return_path import ApproachPath
from kinematics import load_robot, ArmKinematics
from robust_planner import PERIOD, reference_torque
from bottle import BottlePushRoutine, BottleSettings, BottleTarget, BottleObservation
from alignment import Observation as ButtonObservation
from vision import ButtonTarget, validate_reachable_target
from button_feedback import LampFrame


class Motion:
    def __init__(self,world,path,calibration,settings):
        self.w = world
        model,_ = load_robot(path,tuple(calibration['tip_offset_m']),calibration['pelvis_height_m'])
        self.kin = ArmKinematics(model)
        self.kin.update(np.array(world.configuration()[0]))
        self.home = world.current_target().copy()
        self.approach = None
        self.record = False
        self.release_error = self.error = None
        self.settings = settings

    def check(self):
        self.w.configuration()

    def enable(self):
        pass

    def current_target(self):
        return self.w.target.copy()

    def start_approach(self):
        self.approach = ApproachPath(self.current_target()); self.record = True

    def freeze_approach(self):
        self.approach.freeze(); self.record = False

    def stream(self,q,*,velocity=None):
        v = (np.array(q)-self.current_target())/PERIOD if velocity is None else velocity
        v = np.clip(v,-.8,.8)
        self.w.tau = reference_torque(self.kin,np.array(q),v,self.settings.gravity_scale)
        self.w.stream(q)
        if self.record:
            self.approach.append(q,PERIOD)

    def wait(self,duration):
        self.w.wait(duration)

    def move(self,goal,duration,settle_s=.3):
        start = self.current_target()
        duration = max(duration,1.5*np.max(np.abs(start-goal))/.8)
        for q in interpolate(start,goal,duration):
            self.stream(q); self.wait(PERIOD)
        self.wait(settle_s)

    def replay(self,targets,validate):
        for q in targets:
            validate(q); self.stream(q); self.wait(PERIOD)

    def close(self):
        pass


class Localizer:
    def __init__(self,observer,scene):
        self.o,self.s = observer,scene
        self.first = True
        self.measurements,self.diagnostics = [],[]
        self.rejections = Counter()
        self.last_lamp_frame = None

    def observe(self):
        if self.first:
            observed = self.o.value; self.first = False
        else:
            self.o.window.samples.clear()
            for _ in range(3):
                self.o.w.wait(.2); self.o.capture()
            observed = self.o.latest()
        g = observed.geometry
        self.measurements.append(dict(captured_at_s=observed.captured_at,face_xyz_m=g['face']))
        if self.s.task == 'bottle':
            target = BottleTarget((g['face'][0]+g['radius'],g['face'][1]),g['radius'],g['face'],1.,1.,(0,0,1,1))
            return BottleObservation(target,observed.captured_at,len(self.measurements))
        target = ButtonTarget(g['face'],g['normal'],1.,1.,1.)
        self.last_lamp_frame = LampFrame(self.o.last_rgb[:,:,::-1].copy(),self.o.initial_box,
                                         observed.captured_at,observed.frame_id)
        return ButtonObservation(target,observed.captured_at,len(self.measurements))

    def expect_translation(self):
        reference = dict(self.o.window.reference)
        reference['face'] = tuple(np.array(self.s.face)+[self.s.push_distance,0,0])
        self.o.window.reference = reference
        self.o.window.samples.clear()

    def lamp_frames(self,after,timeout_s=2.):
        for _ in range(3):
            self.o.w.wait(.2); self.o.capture()
            yield LampFrame(self.o.last_rgb[:,:,::-1].copy(),self.o.initial_box,self.o.w.now(),self.o.frame_id)


def run(world,observer,scene,args,calibration,settings):
    from pathlib import Path
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'real'))
    world.current_target = lambda:world.target.copy()
    world.current_configuration = world.configuration
    motion = Motion(world,Path(args.model).resolve(),calibration,settings)
    model,_ = load_robot(Path(args.model).resolve(),tuple(calibration['tip_offset_m']),calibration['pelvis_height_m'])
    kin = ArmKinematics(model)
    def refresh():
        kin.update_fixed_base(world.configuration()[0],scene.imu)
    refresh()
    localizer = Localizer(observer,scene)
    if scene.task == 'bottle':
        class BStagedRoutine(BottlePushRoutine):
            def check_observation(self,observation,*,max_age_s=.5):
                if not 0 <= world.now()-observation.captured_at <= max_age_s:
                    raise ValueError('Stale baseline observation')
                validate_reachable_target(SimpleNamespace(face_xyz=observation.target.face_xyz,press_direction=(1,0,0)))
                return np.array(observation.target.face_xyz)
        bottle_settings = BottleSettings(scene.base_height,scene.push_distance,.08,.05,
                                         (scene.wall_x,scene.wall_x+1.5,-1.,1.))
        routine = BStagedRoutine(kin,motion,localizer,refresh,world.now,bottle_settings,scene.tip_radius,
                                scene.uncertainty_m,approach_mode='staged')
        report = routine.execute()
    else:
        sys_args = dict(correction_interval=2.,correction_duration=.6,settle_time=.6,phase_timeout=60.,
            alignment_mode='single',normal_motion='smooth',clearance=.05,stroke=scene.stroke,
            output=str(Path(args.report_out).with_suffix(''))+'_baseline.json',stop_before_contact=False)
        import align_button
        from trial_policy import ButtonTrialPolicy
        original_time = align_button.time
        align_button.time = SimpleNamespace(monotonic=world.now)
        cal = dict(calibration,reference_waist_q_rad=list(scene.q[12:15]),reference_imu_rpy_rad=list(scene.imu))
        try:
            report = align_button.execute_alignment(world,localizer,kin,cal,SimpleNamespace(**sys_args),motion=motion,
                policy=ButtonTrialPolicy(approach_mode='staged',fk_tolerance_m=.007,fk_abort_m=.010))
        except Exception as exc:
            report = getattr(exc,'button_report',dict(completed=False,error=str(exc)))
        finally:
            align_button.time = original_time
    report['baseline_adapter'] = 'Production staged controller; shared B geometry, table-outline guard excluded'
    report['stopped_for_measurement'] = report.get('approach_remeasurement_count',0)
    return report
