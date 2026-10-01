#!/usr/bin/env python3
"""Explicitly gated, one-shot, right-arm-only high-level G1 experiment."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time
import numpy as np
from common import (ROOT, ARMS, ARM_NAMES, NAMES, read_arms, validate_arm_pose,
                    smoothstep, interpolate, write_json, SDK_EXAMPLE, URDF, digest,
                    text_digest, acquire_file_lock)
from scene import (check_arm_path, check_baseline_contact_path, make_scene,
                   set_arms, clearance_pairs, pair_clearances)

ACTIVE=[i for i,j in enumerate(ARMS) if j['name'].startswith('right_')]
INACTIVE=[i for i in range(14) if i not in ACTIVE]
MAX_SPEED=.25
MAX_ACCEL=1.0
DT=.02

def validate_clip(t,q,meta,speed,blend):
    if not np.isfinite([speed,blend]).all() or not .3<=speed<=.5 or not 2<=blend<=15:
        raise ValueError('First test requires speed 0.3..0.5 and minimum blend 2..15 s')
    if not 1<=t[-1]<=3 or np.max(np.diff(t))>.021:
        raise ValueError('Need explicit 1..3 s clip sampled at >=50Hz')
    if (meta['sdk_example_sha256'] not in (digest(SDK_EXAMPLE),text_digest(SDK_EXAMPLE)) or
            meta['official_urdf_sha256'] not in (digest(URDF),text_digest(URDF))):
        raise ValueError('Mapping sources changed; re-extract and inspect')
    validate_arm_pose(q)
    velocity=np.diff(q[:,ACTIVE],axis=0)/np.diff(t)[:,None]*speed
    acceleration=np.diff(velocity,axis=0)/(np.diff(t)[:-1,None]/speed)
    if np.max(np.ptp(q[:,ACTIVE],axis=0))>.50:
        raise ValueError('Arm clip range >0.50 rad; too large for first trial')
    if np.max(np.abs(velocity))>MAX_SPEED:
        raise ValueError('Arm speed exceeds 0.25 rad/s; choose a slower/smaller interval')
    if np.max(np.abs(acceleration))>MAX_ACCEL:
        raise ValueError('Arm acceleration exceeds 1.0 rad/s^2; re-extract smoother interval')
    return velocity,acceleration

def transition_duration(a,b,minimum):
    delta=float(np.max(np.abs((b-a)[ACTIVE])))
    return max(minimum,1.875*delta/MAX_SPEED,np.sqrt(5.774*delta/MAX_ACCEL))

def transition(a,b,seconds):
    return np.array([a+(b-a)*smoothstep(u) for u in np.linspace(0,1,int(np.ceil(seconds/DT))+1)])

def prepare(t,q,snapshot,blend,speed):
    """Pure offline planning; also used with a saved receive-only snapshot."""
    neutral=np.array(snapshot['arm_q'],dtype=float);validate_arm_pose(neutral)
    selected_q=q.copy()
    q=q.copy();q[:,INACTIVE]=neutral[INACTIVE]
    max_delta=float(np.max(np.abs(q[:,ACTIVE]-neutral[ACTIVE])))
    transition_distance_pass=max_delta<=1.2
    model,data=make_scene();base=model.qpos0.copy()
    for i,name in enumerate(NAMES):base[model.joint(name).qposadr[0]]=snapshot['all_q'][i]
    entry=transition_duration(neutral,q[0],blend)
    leave=transition_duration(q[-1],neutral,blend)
    # Densely check actual 50Hz commands and entry/return paths in the current
    # full joint posture. Non-arm joint positions are NEVER sent to hardware.
    playback=np.array([interpolate(t,q,v) for v in np.linspace(0,t[-1],int(np.ceil(t[-1]/speed/DT))+1)])
    entry_path=transition(neutral,q[0],entry)
    return_path=transition(q[-1],neutral,leave)
    path=np.concatenate((entry_path,playback,return_path))
    check=check_baseline_contact_path(entry_path,playback,return_path,base,ACTIVE)
    nominal_clip=check_arm_path(selected_q)
    velocity=np.diff(q[:,ACTIVE],axis=0)/np.diff(t)[:,None]*speed
    acceleration=np.diff(velocity,axis=0)/(np.diff(t)[:-1,None]/speed)
    report={'entry_s':entry,'playback_s':float(t[-1]/speed),'return_s':leave,
            'weight_enable_s':5.,'weight_release_s':5.,
            'total_s':5+entry+t[-1]/speed+leave+5,'max_current_displacement_rad':max_delta,
            'transition_distance_limit_rad':1.2,
            'transition_distance_passed':transition_distance_pass,
            'max_velocity_rad_s':float(np.max(np.abs(velocity))),
            'max_acceleration_rad_s2':float(np.max(np.abs(acceleration))),
            'joint_limits_passed':True,
            'clearance':check,'collision':{
                'baseline_contact_aware':check,
                'nominal_selected_clip':nominal_clip,
                'actual_right_only_clip':check['phases']['clip'],
            },
            'neutral':'captured arm posture; operator confirmation REQUIRED before real control',
            'right_arm_only':True}
    report['passed']=bool(transition_distance_pass and check['passed'] and nominal_clip['passed'])
    report['failure_reasons']=([
        f'Current-to-clip displacement {max_delta:.3f} rad > 1.2'
    ] if not transition_distance_pass else [])
    if not check['passed']:report['failure_reasons'].append('Baseline-contact-aware path collision check failed')
    if not nominal_clip['passed']:report['failure_reasons'].append('Nominal selected clip collision check failed')
    return q,neutral,report

def banner(t,q,meta,speed,blend):
    velocity,acceleration=validate_clip(t,q,meta,speed,blend)
    print('\n=== G1 初回・右腕のみ / one-shot ===',flush=True)
    print(f"Motion: {meta['hf_path']} frames [{meta['start_frame_inclusive']}, {meta['end_frame_exclusive']})")
    print('動作: 右腕を小さく前に上げる「何かいた？」。左腕・腰・脚へ有効commandを送らない。')
    print(f'Source FPS: UNKNOWN. Explicit authored duration={t[-1]:.2f}s; speed={speed}; clip={t[-1]/speed:.2f}s')
    print(f'Entry/return >= {blend:.1f}s each, extended to enforce {MAX_SPEED} rad/s; weight ramps 5s each')
    print('Kp=60, Kd=1.5, dq=0, tau=0 (official arm7 example). rt/arm_sdk @50Hz; weight slot=29.')
    print('joint / SDK / start / minimum / maximum (rad) / max rad/s')
    for k,i in enumerate(ACTIVE):
        print(f"{ARM_NAMES[i]:28} {ARMS[i]['sdk_index']:2} {q[0,i]:+.4f} {q[:,i].min():+.4f} "
              f"{q[:,i].max():+.4f} {np.abs(velocity[:,k]).max():.4f}")
    print(f'Max speed={np.abs(velocity).max():.4f} rad/s; max acceleration={np.abs(acceleration).max():.4f} rad/s^2')
    print('Risk: 姿勢・手の型・把持物・干渉・立位制御は実機依存。FKは動力学/安全保証ではありません。')
    print('Ctrl+C: trajectory中断、zero-gain/zero-weight解放を試行。通信断/killでは解放を保証できません。',flush=True)

def check_feedback(reader,neutral,baseline,last_target,collision_context=None,next_target=None):
    from robot_transport import validate_state
    state=reader.get();current=validate_state(state)
    reader.check_exclusive(own_writer=True)
    if last_target is not None and np.max(np.abs((current-last_target)[ACTIVE]))>.35:
        raise RuntimeError('Arm tracking error >0.35 rad')
    if np.max(np.abs(current[INACTIVE]-neutral[INACTIVE]))>.10:
        raise RuntimeError('Uncommanded left arm moved; collision posture no longer valid')
    other=np.array([state.motor_state[i].q for i in range(15)])
    if np.max(np.abs(other-baseline))>.03:
        raise RuntimeError('Leg/waist posture changed; collision posture no longer valid')
    if collision_context is not None:
        model,data,pairs,baseline_distances=collision_context
        base=model.qpos0.copy()
        for i,name in enumerate(NAMES):base[model.joint(name).qposadr[0]]=state.motor_state[i].q
        for pose in (current,next_target):
            set_arms(model,data,pose,base)
            distances=pair_clearances(model,data,pairs)
            for pair,distance in distances.items():
                baseline=baseline_distances[pair]
                if baseline<.015:
                    if distance<baseline-1e-6:
                        raise RuntimeError(
                            f'Live baseline pair worsened {baseline:.6f}->{distance:.6f}m: {pair}')
                elif distance<.015:
                    raise RuntimeError(f'Live new clearance deficit {distance:.6f}m: {pair}')

def run_phase(writer,reader,neutral,baseline,seconds,pose_at,weight_at,collision_context=None):
    start=time.monotonic();deadline=start;last=None
    steps=int(np.ceil(seconds/DT))
    for k in range(steps+1):
        now=time.monotonic()
        if now-deadline>.10:raise RuntimeError('Control loop missed deadline by >100ms')
        if now<deadline:time.sleep(deadline-now)
        u=k/steps;target=pose_at(u)
        check_feedback(reader,neutral,baseline,last,collision_context,target)
        writer.write(target,weight_at(u));last=target
        deadline=start+(k+1)*seconds/steps

def execute(args,t,q):
    # This function is unreachable in dry-run. It is the only command path.
    from robot_transport import ReadOnlyState,ArmWriter,state_summary,validate_state
    if not sys.stdin.isatty():raise RuntimeError('Real motion requires an attended terminal')
    with (ROOT/'output/robot.lock').open('w') as lock:
        try:acquire_file_lock(lock)
        except BlockingIOError:raise RuntimeError('Another motiondecode-test controller is running')
        print(f'Receive-only preflight: NIC={args.network_interface}, DDS domain=0',flush=True)
        reader=ReadOnlyState(args.network_interface);writer=None;engaged=False
        try:
            state=reader.wait(15);validate_state(state,initial=True);reader.require_stationary()
            reader.check_exclusive()
            snapshot=state_summary(state)
            path,neutral,report=prepare(t,q,snapshot,args.blend,args.speed)
            write_json(ROOT/'output/preflight_live.json',report)
            print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
            if not report['passed']:
                raise RuntimeError('Live preflight FAILED: '+'; '.join(report['failure_reasons']))
            print('現場確認: 安定立位・周囲空き・両手空・他の腕制御停止・物理停止手段を準備。')
            print('現在の腕姿勢を戻り先neutralとして確認し、G1を見ながら ARMS と入力してください。')
            if input('> ').strip()!='ARMS':raise RuntimeError('Cancelled; no commands sent')
            # User may have moved the robot while reviewing. Re-read, never use
            # an old first pose, and refuse changes instead of replanning unseen.
            state=reader.get();current=validate_state(state,initial=True);reader.require_stationary()
            if np.max(np.abs(current-neutral))>.025:raise RuntimeError('Pose changed since review; rerun preflight')
            baseline=np.array(snapshot['all_q'][:15])
            if np.max(np.abs(np.array([m.q for m in state.motor_state[:15]])-baseline))>.025:
                raise RuntimeError('Leg/waist pose changed since review; rerun preflight')
            check_feedback(reader,neutral,baseline,None)
            reader.check_exclusive()
            print('開始: 右腕7軸 / entry -> clip -> captured neutral -> release',flush=True)
            model,data=make_scene();pairs=clearance_pairs(model)
            base=model.qpos0.copy()
            for i,name in enumerate(NAMES):base[model.joint(name).qposadr[0]]=snapshot['all_q'][i]
            set_arms(model,data,neutral,base)
            context=(model,data,pairs,pair_clearances(model,data,pairs))
            writer=ArmWriter(reader);engaged=True
            run_phase(writer,reader,neutral,baseline,5,lambda u:neutral,lambda u:u,context)
            run_phase(writer,reader,neutral,baseline,report['entry_s'],lambda u:neutral+(path[0]-neutral)*smoothstep(u),lambda u:1,context)
            run_phase(writer,reader,neutral,baseline,report['playback_s'],lambda u:interpolate(t,path,u*t[-1]),lambda u:1,context)
            run_phase(writer,reader,neutral,baseline,report['return_s'],lambda u:path[-1]+(neutral-path[-1])*smoothstep(u),lambda u:1,context)
            run_phase(writer,reader,neutral,baseline,5,lambda u:neutral,lambda u:1-u,context)
            engaged=False
            print('完了: captured neutralへ復帰し、weight=0を送信。',flush=True)
        finally:
            if writer is not None:
                if engaged:
                    print('中断: trajectory停止、zero-weight解放を試行。neutralへの移動は継続しません。',flush=True)
                    for _ in range(3):
                        try:writer.release_now()
                        except Exception as exc:print(f'Release failed: {exc}',file=sys.stderr)
                        time.sleep(DT)
                writer.close()
            reader.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('motion',type=Path)
    p.add_argument('--speed',type=float,default=.4)
    p.add_argument('--blend',type=float,default=2.)
    group=p.add_mutually_exclusive_group()
    group.add_argument('--dry-run',action='store_true')
    group.add_argument('--enable-real-robot',action='store_true')
    p.add_argument('--network-interface',
                   help='Required for real robot mode; never inferred or defaulted')
    p.add_argument('--state-json',type=Path,help='Offline preflight with saved snapshot; never authorizes real control')
    p.add_argument('--preflight-json',type=Path,
                   help='Offline preflight report path; use a new name to retain older reports')
    args=p.parse_args()
    if args.enable_real_robot and args.state_json:p.error('Real mode cannot use a saved pose')
    if args.enable_real_robot and not args.network_interface:
        p.error('Real mode requires an explicit --network-interface')
    try:
        t,q,meta=read_arms(args.motion);banner(t,q,meta,args.speed,args.blend)
        check=check_arm_path(q)
        print('Nominal arm-only clearance:',check,flush=True)
        if not check['passed']:raise ValueError('Nominal collision check failed')
        if not args.enable_real_robot:
            print('DRY-RUN: no SDK imported, no DDS initialized, no robot commands.',flush=True)
            if args.state_json:
                _,_,report=prepare(t,q,json.loads(args.state_json.read_text()),args.blend,args.speed)
                write_json(args.preflight_json or ROOT/'output/preflight_saved_state.json',report)
                print(json.dumps(report,indent=2,ensure_ascii=False))
                if not report['passed']:raise ValueError('Saved-pose preflight failed; live preflight must pass')
            else:print('Current arm pose / actual entry & return time: UNCHECKED; measured only in attended real preflight.')
            return 0
        def stop(signum,frame):raise KeyboardInterrupt(f'signal {signum}')
        signal.signal(signal.SIGTERM,stop)
        execute(args,t,q)
        return 0
    except (ValueError,RuntimeError,KeyboardInterrupt,EOFError,OSError,KeyError,TypeError) as exc:
        print(f'STOP: {exc}',file=sys.stderr,flush=True);return 2

if __name__=='__main__':raise SystemExit(main())
