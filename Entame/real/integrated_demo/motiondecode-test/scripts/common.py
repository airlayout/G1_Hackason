"""Strict, name-based MotionDecode parsing. No robot SDK imports here."""
from pathlib import Path
import ast
import csv
import hashlib
import json
import os
import re
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for key, value in {
    'HF_HOME': ROOT / '.cache/huggingface', 'XDG_CACHE_HOME': ROOT / '.cache',
    'PIP_CACHE_DIR': ROOT / '.cache/pip', 'TMPDIR': ROOT / '.cache/tmp',
    'MPLCONFIGDIR': ROOT / '.cache/matplotlib',
    'CUDA_CACHE_PATH': ROOT / '.cache/cuda',
}.items():
    os.environ[key] = str(value)

SDK_EXAMPLE = ROOT / 'external/g1_arm7_sdk_dds_example.py'
URDF = ROOT / 'external/g1_official_29dof.urdf'
MODEL = ROOT / 'external/g1_official_29dof.xml'

def output_path(path):
    path = Path(path).resolve()
    try:
        path.relative_to(ROOT)
    except ValueError:
        raise ValueError(f'Outputs must stay inside {ROOT}')
    path.parent.mkdir(parents=True, exist_ok=True)
    return path

def write_json(path, value):
    output_path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def text_digest(path):
    """Hash repository text canonically so Git CRLF checkout stays verifiable."""
    content=Path(path).read_bytes().replace(b'\r\n',b'\n')
    return hashlib.sha256(content).hexdigest()

def acquire_file_lock(lock):
    """Acquire a non-blocking one-byte process lock on POSIX or Windows."""
    if os.name!='nt':
        import fcntl
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return
    import msvcrt
    lock.seek(0);lock.write('\0');lock.flush();lock.seek(0)
    try:msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    except OSError as exc:raise BlockingIOError('Controller lock is held') from exc

def joint_definitions():
    """Read the actual official SDK constants, without executing the sample."""
    urdf_joints = {j.get('name'): j for j in ET.parse(URDF).getroot().findall('joint')}
    cls = next(n for n in ast.parse(SDK_EXAMPLE.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'G1JointIndex')
    result = {}
    for node in cls.body:
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Constant):
            continue
        sdk_name = node.targets[0].id
        name = re.sub(r'(?<!^)(?=[A-Z])', '_', sdk_name).lower() + '_joint'
        if name not in urdf_joints:
            continue  # SDK aliases (AnkleA/B, WaistA/B) and weight are not URDF joints.
        limit = urdf_joints[name].find('limit')
        result[name] = {'name': name, 'sdk_name': sdk_name, 'sdk_index': node.value.value,
                        'csv_column': f'dof_{name}(rad)',
                        'lower': float(limit.get('lower')), 'upper': float(limit.get('upper'))}
    result = sorted(result.values(), key=lambda x: x['sdk_index'])
    if len(result) != 29 or [x['sdk_index'] for x in result] != list(range(29)):
        raise ValueError('Official URDF / SDK mapping is not a complete 29-DOF mapping')
    return result

JOINTS = joint_definitions()
NAMES = [j['name'] for j in JOINTS]
ARMS = [j for j in JOINTS if any(s in j['name'] for s in ('shoulder', 'elbow', 'wrist'))]
ARM_NAMES = [j['name'] for j in ARMS]
ARM_INDICES = [j['sdk_index'] for j in ARMS]
ROOT_COLUMNS = [f'root_pos_{x}(m)' for x in 'xyz'] + [f'root_rot_{x}' for x in 'wxyz']

def read_table(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
        if len(set(header)) != len(header):
            raise ValueError('Duplicate CSV columns')
        rows = list(reader)
    if len(rows) < 2 or any(len(r) != len(header) for r in rows):
        raise ValueError('Need >=2 complete frames; ragged/empty CSV rejected')
    a = np.asarray(rows, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('NaN / Inf in motion')
    return header, a

def read_motion(path):
    header, a = read_table(path)
    expected = ROOT_COLUMNS + [j['csv_column'] for j in JOINTS]
    if set(header) != set(expected) or len(header) != 36:
        raise ValueError('Expected named root position (m), quaternion wxyz, 29 named joints (rad)')
    a = a[:, [header.index(n) for n in expected]]
    norms = np.linalg.norm(a[:, 3:7], axis=1)
    if np.max(np.abs(norms - 1)) > 0.002:
        raise ValueError('Root quaternion is not unit length')
    return a, header

def mapping_report(header):
    return [dict(j, csv_column_zero_based=header.index(j['csv_column'])) for j in JOINTS]

def stats(a, fps=None):
    if fps is not None and (not np.isfinite(fps) or fps <= 0):
        raise ValueError('FPS must be finite and positive')
    q = a[:, 7:]; dq = np.abs(np.diff(q, axis=0)); root = a[:, :3]
    violations = []
    per_joint = []
    for i, j in enumerate(JOINTS):
        lo, hi = float(q[:, i].min()), float(q[:, i].max())
        if lo < j['lower'] or hi > j['upper']:
            violations.append(j['name'])
        per_joint.append(dict(j, min_rad=lo, max_rad=hi, start_rad=float(q[0, i]),
                              total_variation_rad=float(dq[:, i].sum()),
                              max_step_rad=float(dq[:, i].max()),
                              max_speed_rad_s=float(dq[:, i].max()*fps) if fps else None))
    return {'frames': len(a), 'joint_count': 29, 'source_fps': None,
            'evaluated_fps': fps, 'fps_status': 'UNVERIFIED; explicit evaluation rate only' if fps else 'UNKNOWN',
            'duration_s': (len(a)-1)/fps if fps else None,
            'duration_at_30fps_s': (len(a)-1)/30, 'duration_at_120fps_s': (len(a)-1)/120,
            'root_net_displacement_m': float(np.linalg.norm(root[-1]-root[0])),
            'root_path_length_m': float(np.linalg.norm(np.diff(root,axis=0),axis=1).sum()),
            'root_max_displacement_m': float(np.linalg.norm(root-root[0],axis=1).max()),
            'root_axis_range_m': np.ptp(root,axis=0).tolist(),
            'root_max_orientation_change_rad': float((2*np.arccos(np.clip(np.abs(
                (a[:,3:7]/np.linalg.norm(a[:,3:7],axis=1)[:,None]) @ (a[0,3:7]/np.linalg.norm(a[0,3:7]))),0,1))).max()),
            'leg_total_variation_rad': float(dq[:,:12].sum()),
            'waist_total_variation_rad': float(dq[:,12:15].sum()),
            'arm_total_variation_rad': float(dq[:,ARM_INDICES].sum()),
            'max_leg_range_rad': float(np.ptp(q[:,:12],axis=0).max()),
            'max_waist_range_rad': float(np.ptp(q[:,12:15],axis=0).max()),
            'max_arm_step_rad': float(dq[:,ARM_INDICES].max()),
            'joint_limit_violations': violations, 'joints': per_joint}

def read_arms(path):
    path = Path(path)
    header, a = read_table(path)
    expected = ['time_s'] + [f'{j}(rad)' for j in ARM_NAMES]
    if header != expected:
        raise ValueError('Expected extracted arm CSV: time_s + 14 named arm columns, in SDK order')
    t, q = a[:,0], a[:,1:]
    if abs(t[0]) > 1e-9 or np.any(np.diff(t)<=0):
        raise ValueError('Arm timestamps must start at 0 and increase strictly')
    meta = json.loads(path.with_suffix('.json').read_text())
    if meta.get('csv_sha256') not in (digest(path),text_digest(path)) or meta.get('arm_names') != ARM_NAMES:
        raise ValueError('Arm artifact hash or joint names do not match sidecar')
    if meta.get('timing_mode') != 'explicit_retiming' or meta.get('schema') != 1:
        raise ValueError('Unsupported timing provenance')
    return t, q, meta

def validate_arm_pose(q, margin=0.03):
    q = np.asarray(q)
    if q.shape[-1] != 14 or not np.isfinite(q).all():
        raise ValueError('Expected 14 finite arm angles')
    lower = np.array([j['lower'] for j in ARMS]) + margin
    upper = np.array([j['upper'] for j in ARMS]) - margin
    bad = np.any((q < lower) | (q > upper), axis=0) if q.ndim == 2 else (q < lower) | (q > upper)
    if np.any(bad):
        raise ValueError('Official arm limit + margin exceeded: '+', '.join(np.array(ARM_NAMES)[bad]))

def smoothstep(u):
    return u*u*u*(10 + u*(-15 + 6*u))

def retime_arm_window(q, duration, smoothing_window=11, sample_hz=50):
    """Apply the project's explicit arm retiming to an in-memory frame window.

    Source FPS is deliberately not inferred.  ``duration`` is the authored
    output time and the quintic phase gives zero phase velocity at both ends.
    """
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or q.shape[1]!=14 or len(q)<3 or not np.isfinite(q).all():
        raise ValueError('Expected at least three finite 14-joint arm frames')
    if not np.isfinite(duration) or not 1<=duration<=3:
        raise ValueError('Authored duration must be 1..3 s')
    if not isinstance(smoothing_window,int) or smoothing_window<1 or smoothing_window%2!=1 or smoothing_window>min(121,len(q)):
        raise ValueError('Invalid smoothing window')
    if not isinstance(sample_hz,int) or sample_hz<50:
        raise ValueError('Output sample rate must be an integer >=50 Hz')
    if smoothing_window>1:
        padded=np.pad(q,((smoothing_window//2,smoothing_window//2),(0,0)),mode='edge')
        kernel=np.ones(smoothing_window)/smoothing_window
        q=np.stack([np.convolve(padded[:,i],kernel,mode='valid') for i in range(14)],axis=1)
    output_t=np.linspace(0,duration,int(round(duration*sample_hz))+1)
    phase=smoothstep(output_t/duration)
    q=np.stack([np.interp(phase,np.linspace(0,1,len(q)),q[:,i]) for i in range(14)],axis=1)
    return output_t,q

def interpolate(t, q, when):
    k = min(max(int(np.searchsorted(t, when, side='right')-1),0),len(t)-2)
    u = np.clip((when-t[k])/(t[k+1]-t[k]),0,1)
    return q[k]*(1-u)+q[k+1]*u
