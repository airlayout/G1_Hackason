#!/usr/bin/env python3
"""Prepare RGB-D/DDS, verify policy IO, then run one operator-requested trial."""
import argparse
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


def write_json(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def local_path(value,*,resolve_symlinks=True):
    path = Path(value).expanduser()
    path = path if path.is_absolute() else ROOT/path
    # Following a venv's bin/python symlink selects the system interpreter and
    # loses that venv's site-packages. Data paths may still be resolved normally.
    return path.resolve() if resolve_symlinks else Path(os.path.abspath(path))


def motion_arguments(mode,trial,output,socket_path=None,state_path=None):
    args = [str(REAL/'run_push.py'),'--task','bottle','--motion-planner','robust',
        '--trial-config',str(trial),'--output',str(output)]
    if state_path is not None:
        args += ['--state-json',str(state_path)]
    else:
        args += ['--arm-socket',str(socket_path),'--execute','--calibrated']
    if mode != 'push':
        args += ['--stop-before-contact','--static-target']
    if state_path is None and mode in ('precontact','push'):
        args += ['--execute-after-plan','--planning-scenarios','3']
    return args


def validate_preview(report,expected_xy,uncertainty):
    plan = report.get('plan',{})
    if not plan.get('accepted') or not report.get('planning_completed'):
        raise RuntimeError('経路を採用できません: '+str(report.get('error') or plan.get('reason')))
    scene = plan['scene']
    centre = [scene['face'][0]+scene['radius'],scene['face'][1]]
    distance = sum((a-b)**2 for a,b in zip(centre,expected_xy))**.5
    if distance > uncertainty:
        raise RuntimeError(f'現在のボトル位置が登録した実測値と{distance*100:.1f}cm違います。配置・校正を確認してください')
    return dict(centre_xy_m=centre,measured_reference_xy_m=expected_xy,
        reference_difference_m=distance,completed_conditions=plan['completed_rollouts'],
        trajectory_samples=len(plan['trajectory']['q']))


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
        print('腕の動作・復帰は完了しましたが、画像で目標の押し込み量を確認できませんでした。')
        if 'measured_displacement_m' in report:
            print('画像で測ったボトル移動量:',report['measured_displacement_m'],'m')
    else:
        print('終了理由: 詳細は実行ログを確認してください。')
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('check','precontact','push'),
        help='check: 重み0で入出力確認 / precontact: 接触手前往復 / push: 本押し')
    parser.add_argument('--setup',type=Path,default=ROOT/'_local/robust_push/active_setup.json')
    parser.add_argument('--push-distance',type=float,
        help='ボトルの押し込み量[m]。シミュレーション・実機の両方へ同じ値を適用する')
    args = parser.parse_args(argv)
    if args.push_distance is not None and (not math.isfinite(args.push_distance) or not 0 < args.push_distance <= .1):
        parser.error('--push-distance must be finite and between 0 and 0.1m')
    try:
        setup = json.loads(args.setup.read_text())
    except (OSError,ValueError) as exc:
        parser.error('実機設定を読めません: '+str(exc))
    required = {'sdk_python','vision_python','trial_config','ssh_host','remote_camera_root',
                'remote_sdk_python','measured_bottle_xy_m'}
    if not isinstance(setup,dict):
        parser.error('実機設定はJSONオブジェクトで指定してください')
    if required-set(setup):
        parser.error('実機設定に必要な項目がありません: '+', '.join(sorted(required-set(setup))))
    for key in ('sdk_python','vision_python','trial_config'):
        setup[key] = str(local_path(setup[key],resolve_symlinks=key=='trial_config'))
        if not Path(setup[key]).is_file():
            parser.error('見つかりません: '+setup[key])
    base = ROOT/'_local/robust_push/runs'
    base.mkdir(parents=True,exist_ok=True)
    run = base/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.mode+'_'+uuid.uuid4().hex[:6])
    run.mkdir()
    print('ログ:',run,flush=True)
    report = dict(mode=args.mode,completed=False,io_passed=None,setup_file=str(args.setup),phases=[],
        flow='check-input-policy-output' if args.mode == 'check' else 'initialize-single-live-plan-execute')
    env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MUJOCO_GL='egl',
               YOLO_CONFIG_DIR=str(ROOT/'_local/bottle_push/ultralytics'))
    lock = (base/'robot.lock').open('a')
    try:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('別のrobust実行が動作中です。同時には実行しません')
        trial = json.loads(Path(setup['trial_config']).read_text())
        if args.push_distance is not None:
            trial['push_distance'] = args.push_distance
        report['push_distance_m'] = trial.get('push_distance',.05)
        for key in ('model','weights','base_transform'):
            trial[key] = str(local_path(trial[key]))
            if not Path(trial[key]).is_file():
                raise RuntimeError('必要なファイルがありません: '+trial[key])
        calibration = json.loads(Path(trial['base_transform']).read_text())
        write_json(run/'calibration.json',calibration)
        trial['base_transform'] = str(run/'calibration.json')
        trial_path = run/'trial.json'
        write_json(trial_path,trial)
        write_json(run/'setup.json',setup)
        source_files = sorted(REAL.glob('*.py'))+sorted(REAL.parent.glob('*.py'))
        write_json(run/'source_sha256.json',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source_files})
        def phase(name,command,*,input_text=None):
            print(name,flush=True)
            row = dict(name=name,started_at=datetime.now().astimezone().isoformat())
            report['phases'].append(row)
            log_path = run/(f'{len(report["phases"]):02d}.log')
            row['log'] = str(log_path)
            row['exit_code'] = execute_child(command,log_path,env,input_text=input_text)
            if row['exit_code']:
                raise RuntimeError(name+' が完了しませんでした')
        print('1. SSH・RGB-D配信を準備',flush=True)
        report['camera'] = ensure_camera(setup,run)
        phase('2. 関節入力・立位・他の制御を確認', [setup['sdk_python'],str(REAL/'check_robust_readiness.py'),
            '--trial-config',str(trial_path),'--output-dir',str(run/'state'),
            '--ssh-host',setup['ssh_host']])
        if args.mode == 'check':
            preview = run/'preview.json'
            phase('3. RGB-D入力・認識・経路生成を確認',[setup['vision_python'],
                *motion_arguments(args.mode,trial_path,preview,state_path=run/'state/state.json')])
            report['policy'] = validate_preview(json.loads(preview.read_text()),setup['measured_bottle_xy_m'],
                float(calibration['position_uncertainty_m']))
            with sdk_bridge(setup['sdk_python'],trial['network_interface'],run,'io',env) as sock:
                phase('4. 方策出力→SDK→DDSを重み0で確認',[setup['sdk_python'],str(REAL/'check_arm_output.py'),
                    '--socket',str(sock),'--network-interface',trial['network_interface'],
                    '--plan',str(preview),'--output',str(run/'output_io.json'),
                    '--ssh-host',setup['ssh_host'],'--remote-python',setup['remote_sdk_python'],
                    '--remote-interface',setup.get('remote_network_interface','eth0')])
            report['io_passed'] = True
            report['completed'] = True
            print('入力・方策生成・出力の疎通確認が完了しました。腕の制御は有効にしていません。',flush=True)
            return 0
        print('このコマンドの指定に従い、'+('接触手前確認' if args.mode=='precontact' else '本押し')+'を開始します。',flush=True)
        if args.mode == 'push':
            print(f'ボトルの目標移動量: {report["push_distance_m"]*100:.1f}cm（計画・実機共通）',flush=True)
        with sdk_bridge(setup['sdk_python'],trial['network_interface'],run,'motion',env) as sock:
            phase('3. シミュレーション3条件が通過したら即実行',[setup['vision_python'],
                *motion_arguments(args.mode,trial_path,run/'result.json',socket_path=sock)])
        actual = json.loads((run/'result.json').read_text())
        report['completed'] = bool(actual.get('completed'))
        report['policy'] = dict(accepted=actual.get('plan',{}).get('accepted'),
            completed_conditions=actual.get('plan',{}).get('completed_rollouts'))
        execution = actual.get('execution',{})
        print('動作データを保存しました。制御解除:',execution.get('release_completed'),
            '帰還確認:',execution.get('home_completed'),
            'FK誤差最大:',round(execution.get('max_fk_error_m',0.)*1000,1),'mm',flush=True)
        return 0 if report['completed'] else 1
    except KeyboardInterrupt:
        report['error'] = '操作者による中断'
        print('中断しました。制御解除の結果はログに保存します。',flush=True)
        return 130
    except Exception as exc:
        report['error'] = str(exc)
        print(str(exc),file=sys.stderr,flush=True)
        print_execution_failure(run)
        return 1
    finally:
        write_json(run/'session.json',report)
        lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
