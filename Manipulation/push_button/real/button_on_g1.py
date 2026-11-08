"""Upload a separate code release and invoke the canonical launcher on G1.

Existing calibration, weights, Python environments and logs stay in the configured
runtime. A fresh release prevents editing source used by a running controller.
"""
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import uuid


def source_archive(root):
    buffer = io.BytesIO()
    base = root/'Manipulation/push_button'
    with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
        for path in sorted(base.glob('*.py'))+sorted((base/'real').glob('*.py'))+sorted((base/'real').glob('*.sh')):
            archive.add(path,arcname=str(path.relative_to(root)),recursive=False)
    return buffer.getvalue()


INSTALL = '''import io,json,pathlib,sys,tarfile
base=pathlib.Path(sys.argv[1])
release=base/'_local/robust_button/code_releases'/sys.argv[2]
if not (base/'_local/robust_button/active_setup.json').is_file():
    raise SystemExit('G1 runtime has no active_setup.json: '+str(base))
release.mkdir(parents=True,exist_ok=False)
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz') as archive:
    for entry in archive.getmembers():
        path=pathlib.PurePosixPath(entry.name)
        if not entry.isfile() or path.is_absolute() or '..' in path.parts or path.parts[:2]!=('Manipulation','push_button'):
            raise SystemExit('Invalid code archive entry')
        target=release/entry.name
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(archive.extractfile(entry).read())
        target.chmod(entry.mode & 0o777)
for name in ('_local','Button_Press','Perception'):
    (release/name).symlink_to(base/name,target_is_directory=True)
print(json.dumps({'release':str(release),'runtime':str(base)}))
'''


def run_on_g1(root, setup, arguments, *, deploy_only=False):
    host = setup['ssh_host']
    runtime = setup['remote_runtime_root']
    if not isinstance(runtime,str) or not runtime.startswith('/'):
        raise ValueError('remote_runtime_rootにはG1上の絶対パスが必要です')
    ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',host]
    install = shlex.join(['python3','-c',INSTALL,runtime,uuid.uuid4().hex])
    result = subprocess.run(ssh+[install],input=source_archive(root),capture_output=True,timeout=60)
    if result.returncode:
        raise RuntimeError('G1へのコード転送に失敗しました: '+result.stderr.decode(errors='replace')[-1000:])
    deployed = json.loads(result.stdout)
    print('G1実行コード:',deployed['release'],flush=True)
    print('校正・重み・実行環境:',runtime,flush=True)
    if deploy_only:
        print('転送が完了しました。実機動作は開始していません。',flush=True)
        return 0
    forwarded = []
    source = iter(arguments)
    for arg in source:
        if arg == '--setup':
            next(source)
        elif arg.startswith('--setup=') or arg == '--deploy-only':
            continue
        else:
            forwarded.append(arg)
    forwarded += ['--setup',str(Path(runtime)/'_local/robust_button/active_setup.json')]
    # Preserve the already configured ARM Python and loopback-SSH wrappers.
    wrapper_bin = str(Path(runtime)/'_local/robust_button/bin')
    remote_python = setup.get('remote_sdk_python','python3')
    script = ('import os,pathlib,sys; b=pathlib.Path(sys.argv[1]); '
        'os.environ["G1_BUTTON_ON_ROBOT"]="1"; '
        'os.environ["PATH"]=str(b)+os.pathsep+os.environ["PATH"]; '
        'p=str(b/"python3") if (b/"python3").is_file() else sys.argv[2]; '
        'os.chdir(pathlib.Path(sys.argv[3]).resolve().parents[3]); '
        'os.execvp(p,[p]+sys.argv[3:])')
    command = shlex.join(['python3','-c',script,wrapper_bin,remote_python,
        str(Path(deployed['release'])/'Manipulation/push_button/real/robust_session.py'),*forwarded])
    return subprocess.call(ssh[:1]+['-tt']+ssh[1:]+[command])
