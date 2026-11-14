#!/usr/bin/env python3
"""Prepare RGB-D/DDS, verify policy IO, then run one operator-requested trial."""
import argparse
from collections import Counter
from contextlib import contextmanager
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]
REAL = Path(__file__).resolve().parent
sys.path.insert(0,str(REAL.parent))
from robust_options import BUTTON_DEFAULTS
from robust_policy import COLLECTION_POLICY
from robust_conditions import (BUTTON_CONDITION_COUNT,validate_button_plan,
                               validate_button_scenario_settings)


class PhaseFailure(RuntimeError):
    def __init__(self,name,code):
        super().__init__(name+' が完了しませんでした')
        self.code = code


def write_json(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def local_path(value,*,resolve_symlinks=True):
    path = Path(value).expanduser()
    path = path if path.is_absolute() else ROOT/path
    # Following a venv's bin/python symlink selects the system interpreter and
    # loses that venv's site-packages. Data paths may still be resolved normally.
    return path.resolve() if resolve_symlinks else Path(os.path.abspath(path))


def motion_arguments(mode,trial,output,socket_path=None,state_path=None,*,task='bottle'):
    args = [str(REAL/'run_push.py'),'--task',task,'--motion-planner','robust',
        '--trial-config',str(trial),'--output',str(output)]
    if state_path is not None:
        args += ['--state-json',str(state_path)]
    else:
        args += ['--arm-socket',str(socket_path),'--execute','--calibrated']
    if mode != 'push':
        args += ['--stop-before-contact']
        if task == 'bottle':
            args += ['--static-target']
    elif task == 'bottle':
        args += ['--allow-stale-observation']
    if state_path is None and mode in ('precontact','push'):
        args += ['--execute-after-plan' if task == 'bottle' else '--execute-verified-plan',
                 '--planning-scenarios',str(json.loads(Path(trial).read_text()).get('planning_scenarios',BUTTON_CONDITION_COUNT) if task=='button' else 3)]
    elif task=='button':
        args += ['--planning-scenarios',str(json.loads(Path(trial).read_text()).get('planning_scenarios',BUTTON_CONDITION_COUNT))]
    return args


def validate_preview(report,expected_xy,uncertainty,*,task='bottle',button_validation='physical'):
    plan = report.get('plan',{})
    if not plan.get('accepted') or not report.get('planning_completed'):
        raise RuntimeError('経路を採用できません: '+str(report.get('error') or plan.get('reason')))
    scene = plan['scene']
    if task == 'button':
        validate_button_plan(plan, method=button_validation)
        return dict(face_xyz_m=scene['face'],normal=scene['normal'],stroke_m=scene['stroke'],
            completed_conditions=plan.get('accepted_conditions',plan['completed_rollouts']),
            validation_method=button_validation,trajectory_samples=len(plan['trajectory']['q']))
    centre = [scene['face'][0]+scene['radius'],scene['face'][1]]
    distance = sum((a-b)**2 for a,b in zip(centre,expected_xy))**.5
    if distance > uncertainty:
        raise RuntimeError(f'現在のボトル位置が登録した実測値と{distance*100:.1f}cm違います。配置・校正を確認してください')
    return dict(centre_xy_m=centre,measured_reference_xy_m=expected_xy,
        reference_difference_m=distance,completed_conditions=plan['completed_rollouts'],
        trajectory_samples=len(plan['trajectory']['q']))


def validate_button_trial(trial,calibration):
    """Reject missing measurements and bottle-only modes before opening SSH/DDS."""
    if trial.get('task','button') != 'button':
        raise ValueError('ボタン用のtrial_configを指定してください')
    for key,value in BUTTON_DEFAULTS.items():
        trial.setdefault(key,value)
    collection = trial.get('button_policy','standard') == 'collection'
    if trial.get('button_policy','standard') not in ('standard','collection') or trial.get('button_feedback','off') not in ('off','joint'):
        raise ValueError('Invalid button feedback/policy')
    if trial.get('button_press_mode','feedback') not in ('feedback','straight_trial'):
        raise ValueError('Invalid button press mode')
    if trial.get('button_press_mode','feedback') == 'straight_trial' and trial.get('button_feedback','off') != 'joint':
        raise ValueError('直進試験にはjoint補正を有効にしてください')
    if trial.get('button_validation','physical') not in ('physical','kinematic'):
        raise ValueError('Invalid button validation method')
    if trial.get('button_validation','physical') == 'kinematic' and (not collection or trial.get('button_feedback') != 'joint'):
        raise ValueError('運動学試作にはcollection方策とjoint補正が必要です')
    for key,low,high in (('planning_budget_s',0.,300.),('button_feedback_timeout_s',10.,300.)):
        value = trial.get(key,120.)
        if (isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value)
                or not (low <= value <= high if key=='button_feedback_timeout_s' else low < value <= high)):
            raise ValueError(key+'が範囲外です')
    stroke = trial.get('stroke')
    if isinstance(stroke,bool) or not isinstance(stroke,(int,float)) or not math.isfinite(stroke) or not .0015 <= stroke <= .015:
        raise ValueError('実測したstroke（1.5〜15mm）を設定するか--stroke-mmで指定してください')
    direction,pixel = trial.get('button_direction'),trial.get('target_pixel')
    if direction not in (None,'up','down'):
        raise ValueError('button_directionはup/downで指定してください')
    if pixel is not None and (not isinstance(pixel,(list,tuple)) or len(pixel)!=2 or
            any(isinstance(v,bool) or not isinstance(v,int) or v<0 for v in pixel)):
        raise ValueError('target_pixelは画像内の整数座標[U,V]で指定してください')
    if direction is None and pixel is None:
        raise ValueError('--button-direction up/downまたは--target-pixel U Vで対象を指定してください')
    for key in ('static_target','allow_stale_observation','execute_after_plan'):
        if trial.get(key):
            raise ValueError('ボタンではボトル用の'+key+'を使用できません')
    if trial.get('execute') or trial.get('execute_verified_plan'):
        raise ValueError('実行指定はtrialではなくランチャーのモードで選択してください')
    dimensions = [calibration.get(k) for k in ('button_radius_m','tip_radius_m','position_uncertainty_m')]
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in dimensions):
        raise ValueError('校正JSONに実測button_radius_m・tip_radius_m・position_uncertainty_mが必要です')
    button,tip,uncertainty = dimensions
    if not (0 < tip < .04 and tip <= button <= .06 and (collection or tip < button) and 0 <= uncertainty <= .02):
        raise ValueError('ボタン・手先の半径または校正の不確かさが範囲外です')
    if not collection and button-tip-uncertainty < .003:
        raise ValueError('ボタン半径から手先半径と校正不確かさを引いた余裕が3mm未満です。実測校正を確認してください')
    trial['task'] = 'button'
    trial['planning_scenarios'] = 1 if collection else BUTTON_CONDITION_COUNT
    validate_button_scenario_settings(trial['planning_scenarios'],trial['planning_position_span_m'],trial.get('button_policy','standard'))


def policy_summary(plan):
    return dict(accepted=plan.get('accepted'),
        validation_method=plan.get('validation_method','physical'),
        completed_conditions=plan.get('accepted_conditions',plan.get('completed_rollouts')),
        **{key:plan.get(key) for key in ('candidates_attempted','physically_evaluated_candidates',
            'condition_trials_started','completed_rollouts','aborted_rollouts','first_accepted_s')})


def button_result_summary(actual,mode,*,button_validation='physical'):
    execution = actual.get('execution',{})
    try:
        validate_button_plan(actual.get('plan',{}), method=button_validation)
        plan_error = None
    except ValueError as exc:
        plan_error = str(exc)
    returned = (plan_error is None and bool(actual.get('completed')) and execution.get('release_completed') is True
                and not actual.get('cleanup_errors'))
    completed = returned and (execution.get('diagnostic_completed') is True if mode=='precontact'
                              else actual.get('visual_success') is True and execution.get('motion_completed') is True)
    return dict(completed=bool(completed),motion_completed=bool(execution.get('motion_completed')),
        diagnostic_completed=bool(execution.get('diagnostic_completed')),
        home_completed=execution.get('home_completed'),release_completed=execution.get('release_completed'),
        home_max_error_rad=execution.get('home_max_error_rad'),warnings=execution.get('warnings',[]),
        visual_success=actual.get('visual_success'),lamp_status=actual.get('lamp_check',{}).get('status'),
        physical_press_success=None,
        plan_validation_error=plan_error,
        policy=policy_summary(actual.get('plan',{})))


def execute_child(command,log_path,env,*,input_text=None,timeout=100):
    with log_path.open('w') as log:
        process = subprocess.Popen(command,cwd=ROOT,env=env,stdin=subprocess.PIPE if input_text else subprocess.DEVNULL,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
        def relay():
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line,end='',flush=True)
        reader = threading.Thread(target=relay,daemon=True)
        reader.start()
        if input_text:
            process.stdin.write(input_text)
            process.stdin.close()
        try:
            return process.wait(timeout=timeout)
        except (KeyboardInterrupt,subprocess.TimeoutExpired):
            # Keep the SDK bridge alive while the controller releases its weight.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            raise
        finally:
            reader.join(timeout=3)


@contextmanager
def sdk_bridge(sdk_python,interface,run,tag,env):
    # Linux Unix sockets have a small pathname limit. Logs may live in a long
    # session directory, but every socket gets its own short directory in /tmp.
    with tempfile.TemporaryDirectory(prefix='g1-arm.',dir='/tmp') as directory:
        socket_path = Path(directory)/'arm.sock'
        with (run/(tag+'_bridge.log')).open('w') as log:
            process = subprocess.Popen([str(sdk_python),'-u',str(REAL/'arm_bridge.py'),
                '--arm','--network-interface',interface,'--socket',str(socket_path),
                '--status-out',str(run/(tag+'_bridge_status.json'))],
                cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                deadline = time.monotonic()+10
                while not socket_path.exists():
                    if process.poll() is not None:
                        raise RuntimeError('SDKブリッジが起動できません。'+str(run/(tag+'_bridge.log')))
                    if time.monotonic() > deadline:
                        raise RuntimeError('SDKブリッジの起動がタイムアウトしました')
                    time.sleep(.05)
                yield socket_path
            finally:
                # An EOF/watchdog releases active control before normal exit.
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=3)


def ensure_camera(setup,run):
    settings = dict(root=setup['remote_camera_root'],log_name='rgbd_'+run.name+'.log')
    remote = '''import json,socket,subprocess
from pathlib import Path
settings = json.loads(SETTINGS_JSON)
with socket.socket() as sock:
    listening = sock.connect_ex(('127.0.0.1',5556)) == 0
if listening:
    print(json.dumps(dict(started=False,already_listening=True)))
else:
    root = Path(settings['root']).expanduser()
    script = root/'real/depth_server/start_rgbd_server.sh'
    if not script.is_file():
        raise RuntimeError('RGB-D startup script missing: '+str(script))
    log = Path.home()/'button_press'/settings['log_name']
    with log.open('x') as output:
        process = subprocess.Popen(['bash',str(script),'--source','videohub','--max-fps','5','--no-legacy-rgb'],
            cwd=root,stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    print(json.dumps(dict(started=True,pid=process.pid,log=str(log))))
'''.replace('SETTINGS_JSON',repr(json.dumps(settings)))
    result = subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',setup['ssh_host'],'python3 -'],
        input=remote,text=True,capture_output=True,timeout=10)
    (run/'camera_ssh.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError('SSHまたはRGB-D配信の起動に失敗しました: '+result.stderr[-600:])
    camera = json.loads(result.stdout)
    write_json(run/'camera.json',camera)
    return camera


def print_execution_failure(run):
    path = run/'result.json'
    if not path.exists():
        path = run/'preview.json'
        if not path.exists():
            return
    report = json.loads(path.read_text())
    execution = report.get('execution',{})
    reason = execution.get('error') or report.get('error') or report.get('plan',{}).get('reason')
    if reason:
        print('停止理由:',reason)
    elif execution.get('completed') and report.get('visual_success') is False:
        print('腕の動作・復帰は完了しましたが、画像で押下結果を確認できませんでした。')
        if 'measured_displacement_m' in report:
            print('画像で測ったボトル移動量:',report['measured_displacement_m'],'m')
    else:
        print('終了理由: 詳細は実行ログを確認してください。')
    plan = report.get('plan',{})
    if plan and not plan.get('accepted'):
        print('探索結果:',plan.get('candidates_attempted',len(plan.get('candidates',[]))),
              '候補 / 物理評価:',plan.get('physically_evaluated_candidates',0),
              '/ 完走:',plan.get('completed_rollouts',0))
        # Group IK residuals by cause while retaining one concrete example.
        counts, examples = Counter(), {}
        for candidate in plan.get('candidates',[]):
            reasons = [candidate['reason']] if candidate.get('reason') else []
            if candidate.get('trials'):
                reasons = [str(failure) for trial in candidate['trials']
                           for failure in (trial.get('failure_reasons') or
                                           ([trial['reason']] if trial.get('reason') else []))] or reasons
            for detail in set(reasons):
                category = '手先のIKが収束しません' if detail.startswith('手先のIKが収束しません') else detail
                counts[category] += 1
                examples.setdefault(category,detail)
        for category, count in counts.most_common(5):
            print(f'候補・条件の却下理由 ({count}件): {examples[category]}')
        face = plan.get('scene',{}).get('face')
        if face is not None:
            print('認識したボタン/物体の位置 [m] (停止時の床座標):',face)
        if plan.get('physically_evaluated_candidates') == 0:
            print('物理シミュレーション開始前に全候補が却下されました。'
                  '認識位置・腕の可動範囲・接近経路の衝突を確認してください。')
    delta = execution.get('post_enable_delta')
    if delta:
        index = delta['max_joint_index']
        name = '左肩roll' if index == 16 else f'関節{index}'
        print(f'{name}の変化: {delta["max_joint_rad"]*180/3.141592653589793:.2f}度')
    print('接近指令数:',len(execution.get('commands',[])),
          '制御解除:',execution.get('release_completed','制御開始前'))
    bridge_path = run/'motion_bridge_status.json'
    if bridge_path.exists():
        bridge = json.loads(bridge_path.read_text())
        print('SDK側の解除送信完了:',bridge.get('release_completed'),
            '最終制御重み:',bridge.get('last_weight'))
        if bridge.get('error'):
            print('SDK側の停止理由:',bridge['error'])


def button_child_timeout(trial):
    # Initial RGB-D window + search + feedback + bounded recovery + IO/cleanup.
    motion_budget = (trial.get('button_feedback_timeout_s',120.) if trial.get('button_feedback') == 'joint'
        else trial.get('motion_duration_s',30.))
    return 30.+float(trial.get('planning_budget_s',30.))+float(motion_budget)+30.+30.


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=('bottle','button'),default='bottle')
    parser.add_argument('mode',choices=('check','precontact','push'),
        help='check: 重み0で入出力確認 / precontact: 接触手前往復 / push: 本押し')
    parser.add_argument('--setup',type=Path)
    parser.add_argument('--push-distance',type=float,
        help='ボトルの押し込み量[m]。シミュレーション・実機の両方へ同じ値を適用する')
    parser.add_argument('--stroke-mm',type=float,help='ボタンの実測ストローク[mm]（1.5〜15）')
    parser.add_argument('--button-direction',choices=('up','down'))
    parser.add_argument('--target-pixel',type=int,nargs=2,metavar=('U','V'))
    parser.add_argument('--button-feedback',choices=('off','joint'))
    parser.add_argument('--button-press-mode',choices=('feedback','straight_trial'))
    parser.add_argument('--button-policy',choices=('standard','collection'))
    parser.add_argument('--button-validation',choices=('physical','kinematic'),
        help='kinematic: 試作の静的経路・衝突検査（物理検証なし）')
    parser.add_argument('--button-feedback-timeout-s',type=float)
    parser.add_argument('--planning-budget-s',type=float,help='経路探索と物理検証の合計時間上限')
    parser.add_argument('--arm-speed-multiplier',type=float,help='腕の接近・帰還の速度倍率（0.1〜10）')
    parser.add_argument('--readiness-policy',choices=('strict','collection'),default='strict',
        help='起動前の状態監視。実機バッシュはボトル共通のcollectionを使用')
    parser.add_argument('--deploy-only',action='store_true',help='G1へコードを転送して終了（動作なし）')
    args = parser.parse_args(argv)
    if args.setup is None:
        args.setup = ROOT/('_local/robust_push' if args.task=='bottle' else '_local/robust_button')/'active_setup.json'
    if args.task == 'button' and args.push_distance is not None:
        parser.error('ボタンには--push-distanceではなく--stroke-mmを指定してください')
    if args.task == 'bottle' and any(v is not None for v in (args.stroke_mm,args.button_direction,args.target_pixel,args.button_feedback,args.button_policy,args.button_feedback_timeout_s,args.button_press_mode,args.button_validation)):
        parser.error('ボタン用の引数には--task buttonが必要です')
    if args.arm_speed_multiplier is not None and (args.task != 'button' or not math.isfinite(args.arm_speed_multiplier) or not 0.1 <= args.arm_speed_multiplier <= 10):
        parser.error('--arm-speed-multiplierはボタン用の0.1〜10で指定してください')
    if args.stroke_mm is not None and (not math.isfinite(args.stroke_mm) or not 1.5 <= args.stroke_mm <= 15):
        parser.error('--stroke-mm must be between 1.5 and 15mm')
    if args.push_distance is not None and (not math.isfinite(args.push_distance) or not 0 < args.push_distance <= .1):
        parser.error('--push-distance must be finite and between 0 and 0.1m')
    try:
        setup = json.loads(args.setup.read_text())
    except (OSError,ValueError) as exc:
        parser.error('実機設定を読めません: '+str(exc))
    required = {'sdk_python','vision_python','trial_config','ssh_host','remote_camera_root','remote_sdk_python'}
    if args.task == 'bottle':
        required.add('measured_bottle_xy_m')
    if not isinstance(setup,dict):
        parser.error('実機設定はJSONオブジェクトで指定してください')
    if required-set(setup):
        parser.error('実機設定に必要な項目がありません: '+', '.join(sorted(required-set(setup))))
    if args.task == 'button' and setup.get('execution_host') == 'g1' and os.environ.get('G1_BUTTON_ON_ROBOT') != '1':
        from button_on_g1 import run_on_g1
        try:
            return run_on_g1(ROOT,setup,list(sys.argv[1:] if argv is None else argv),deploy_only=args.deploy_only)
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
            print(str(exc),file=sys.stderr)
            return 2
    if args.deploy_only:
        parser.error('--deploy-onlyにはexecution_host=g1のボタン設定が必要です')
    for key in ('sdk_python','vision_python','trial_config'):
        setup[key] = str(local_path(setup[key],resolve_symlinks=key=='trial_config'))
        if not Path(setup[key]).is_file():
            parser.error('見つかりません: '+setup[key])
    base = ROOT/('_local/robust_push' if args.task=='bottle' else '_local/robust_button')/'runs'
    base.mkdir(parents=True,exist_ok=True)
    run = base/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.mode+'_'+uuid.uuid4().hex[:6])
    run.mkdir()
    print('ログ:',run,flush=True)
    report = dict(task=args.task,mode=args.mode,completed=False,io_passed=None,setup_file=str(args.setup),phases=[],
        readiness_policy=args.readiness_policy,
        flow='check-input-policy-output' if args.mode == 'check' else 'initialize-single-live-plan-execute')
    button_validation = 'physical'
    env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MUJOCO_GL='egl',
               YOLO_CONFIG_DIR=str(ROOT/'_local/bottle_push/ultralytics'))
    # Both tasks control the same robot; keep the original bottle lock shared.
    lock_path = ROOT/'_local/robust_push/runs/robot.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock = lock_path.open('a')
    try:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('別のrobust実行が動作中です。同時には実行しません')
        trial = json.loads(Path(setup['trial_config']).read_text())
        if args.push_distance is not None:
            trial['push_distance'] = args.push_distance
        if args.planning_budget_s is not None:
            trial['planning_budget_s'] = args.planning_budget_s
        if args.task == 'bottle':
            report['push_distance_m'] = trial.get('push_distance',.05)
        for key in ('model','weights','base_transform'):
            trial[key] = str(local_path(trial[key]))
            if not Path(trial[key]).is_file():
                raise RuntimeError('必要なファイルがありません: '+trial[key])
        calibration = json.loads(Path(trial['base_transform']).read_text())
        if args.task == 'button':
            for key,value in (('stroke',None if args.stroke_mm is None else args.stroke_mm/1000),
                              ('button_direction',args.button_direction),('target_pixel',args.target_pixel),('button_feedback',args.button_feedback),
                              ('button_policy',args.button_policy),('button_feedback_timeout_s',args.button_feedback_timeout_s),
                              ('button_validation',args.button_validation),
                              ('button_press_mode',args.button_press_mode),
                              ('planning_budget_s',args.planning_budget_s),('arm_speed_multiplier',args.arm_speed_multiplier)):
                if value is not None:
                    trial[key] = value
            validate_button_trial(trial,calibration)
            button_validation = trial.get('button_validation','physical')
            report['button_validation'] = button_validation
            if button_validation == 'kinematic':
                print('試作: 運動学・衝突のみを事前検査します。物理シミュレーションは行いません。',flush=True)
            report['stroke_m'] = trial['stroke']
            report['arm_speed_multiplier_requested'] = trial.get('arm_speed_multiplier',1.)
            print(f'腕の速度倍率（指定）: {report["arm_speed_multiplier_requested"]:g}倍',flush=True)
            if trial.get('button_press_mode') == 'straight_trial':
                print('直進押下: ボタン手前で位置合わせ後、目標補正を固定して押します。',flush=True)
        write_json(run/'calibration.json',calibration)
        trial['base_transform'] = str(run/'calibration.json')
        trial_path = run/'trial.json'
        write_json(trial_path,trial)
        write_json(run/'setup.json',setup)
        if args.task == 'button':
            report['execution_policy'] = COLLECTION_POLICY.to_dict() if args.mode!='check' else None
            report['planning_limits'] = dict(budget_s=trial['planning_budget_s'],
                max_candidates=trial['planning_max_candidates'],scenarios=trial['planning_scenarios'])
            print(f'ストローク: {trial["stroke"]*1000:g}mm / 探索上限: '
                  f'{trial["planning_budget_s"]:g}秒・{trial["planning_max_candidates"]}候補・'
                  f'{trial["planning_scenarios"]}条件 / 補正: {trial.get("button_feedback","off")} / 方策: {trial.get("button_policy","standard")}',flush=True)
            if args.mode != 'check':
                print('状態・更新400ms / 脚1.0rad/s / IMU角速度0.3rad/s / 同期送信',flush=True)
                if trial.get('button_feedback') == 'joint':
                    print('初期姿勢から関節角と胴体姿勢で補正。追従が遅れる間は目標の進行を保留し、補正を継続します。',flush=True)
        source_files = sorted(REAL.glob('*.py'))+sorted(REAL.glob('*.sh'))+sorted(REAL.parent.glob('*.py'))
        write_json(run/'source_sha256.json',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source_files})
        def phase(name,command,*,input_text=None,timeout=None):
            print(name,flush=True)
            row = dict(name=name,started_at=datetime.now().astimezone().isoformat())
            report['phases'].append(row)
            log_path = run/(f'{len(report["phases"]):02d}.log')
            row['log'] = str(log_path)
            row['exit_code'] = execute_child(command,log_path,env,input_text=input_text,**({} if timeout is None else {'timeout':timeout}))
            if row['exit_code']:
                raise PhaseFailure(name,row['exit_code'])
        print('1. SSH・RGB-D配信を準備',flush=True)
        report['camera'] = ensure_camera(setup,run)
        phase('2. 関節入力・立位・他の制御を確認', [setup['sdk_python'],str(REAL/'check_robust_readiness.py'),
            '--trial-config',str(trial_path),'--output-dir',str(run/'state'),
            '--ssh-host',setup['ssh_host'],'--policy',args.readiness_policy])
        if args.mode == 'check':
            preview = run/'preview.json'
            phase('3. RGB-D入力・認識・経路生成を確認',[setup['vision_python'],
                *motion_arguments(args.mode,trial_path,preview,state_path=run/'state/state.json',task=args.task)],
                **({'timeout':button_child_timeout(trial)} if args.task == 'button' else {}))
            report['policy'] = validate_preview(json.loads(preview.read_text()),setup.get('measured_bottle_xy_m'),
                float(calibration['position_uncertainty_m']),task=args.task,button_validation=button_validation)
            with sdk_bridge(setup['sdk_python'],trial['network_interface'],run,'io',env) as sock:
                phase('4. 方策出力→SDK→DDSを重み0で確認',[setup['sdk_python'],str(REAL/'check_arm_output.py'),
                    '--socket',str(sock),'--network-interface',trial['network_interface'],
                    '--plan',str(preview),'--output',str(run/'output_io.json'),
                    '--ssh-host',setup['ssh_host'],'--remote-python',setup['remote_sdk_python'],
                    '--remote-interface',setup.get('remote_network_interface','eth0'),
                    '--policy',args.readiness_policy])
            report['io_passed'] = True
            report['completed'] = True
            print('入力・方策生成・出力の疎通確認が完了しました。腕の制御は有効にしていません。',flush=True)
            return 0
        print('このコマンドの指定に従い、'+('接触手前確認' if args.mode=='precontact' else '本押し')+'を開始します。',flush=True)
        if args.mode == 'push' and args.task == 'bottle':
            print(f'ボトルの目標移動量: {report["push_distance_m"]*100:.1f}cm（計画・実機共通）',flush=True)
        elif args.task == 'button':
            print(f'ボタンの実測ストローク: {trial["stroke"]*1000:g}mm（計画・実機共通）',flush=True)
        with sdk_bridge(setup['sdk_python'],trial['network_interface'],run,'motion',env) as sock:
            conditions = trial['planning_scenarios'] if args.task=='button' else 3
            phase(('3. 運動学・衝突検査が通過したら即実行（物理検証なし）' if button_validation == 'kinematic'
                   else f'3. シミュレーション{conditions}条件が通過したら即実行'),[setup['vision_python'],
                *motion_arguments(args.mode,trial_path,run/'result.json',socket_path=sock,task=args.task)],
                **({'timeout':button_child_timeout(trial)} if args.task == 'button' else {}))
        actual = json.loads((run/'result.json').read_text())
        if args.task=='button':
            validate_button_plan(actual.get('plan',{}), method=button_validation)
        report['completed'] = bool(actual.get('completed'))
        if args.task == 'button':
            report.update(button_result_summary(actual,args.mode,button_validation=button_validation))
        report['policy'] = policy_summary(actual.get('plan',{}))
        execution = actual.get('execution',{})
        print('動作データを保存しました。制御解除:',execution.get('release_completed'),
            '帰還確認:',execution.get('home_completed'),
            'FK誤差最大:',round(execution.get('max_fk_error_m',0.)*1000,1),'mm',flush=True)
        if args.mode == 'push' and args.task == 'bottle' and actual.get('visual_success') is None:
            print('新しい結果画像が揃わないため、実際の押し込み量は未確認です。',flush=True)
        elif args.task == 'button' and args.mode == 'push':
            print('点灯確認:',{'lit':'増光を確認','no_rise':'増光なし','unknown':'判定不能'}.get(
                report.get('lamp_status'),'判定不能'),'（自動の押し直しは行いません）',flush=True)
        return 0 if report['completed'] else 1
    except KeyboardInterrupt:
        report['error'] = '操作者による中断'
        print('中断しました。制御解除の結果はログに保存します。',flush=True)
        return 130
    except Exception as exc:
        report['error'] = str(exc)
        print(str(exc),file=sys.stderr,flush=True)
        print_execution_failure(run)
        if args.task == 'button':
            return exc.code if isinstance(exc,PhaseFailure) and exc.code in (1,2,130) else 2
        return 1
    finally:
        if args.task == 'button' and (run/'result.json').is_file():
            try:
                actual = json.loads((run/'result.json').read_text())
                report.update(button_result_summary(actual,args.mode,button_validation=button_validation))
            except (OSError,ValueError,TypeError) as exc:
                report.update(completed=False,result_read_error=str(exc))
        write_json(run/'session.json',report)
        lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
