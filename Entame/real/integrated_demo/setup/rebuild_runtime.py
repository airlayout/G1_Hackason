"""Build/verify an offline bundle. Never SSH, initialize DDS, or run robot code."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
C_DDS = '9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd'


def run(args, **kwargs):
    subprocess.run([str(x) for x in args], check=True, **kwargs)


def hashes(root):
    lines = []
    for path in sorted(root.rglob('*')):
        if path.relative_to(root).parts[0] == 'build':
            continue
        if path.is_file() and path.name not in ('MANIFEST.sha256', 'BUNDLE_MANIFEST.sha256'):
            lines.append(hashlib.sha256(path.read_bytes()).hexdigest() + '  ' + path.relative_to(root).as_posix())
    return '\n'.join(lines) + '\n'


def verify(root):
    for directory, name in [(root / 'code', 'MANIFEST.sha256'),
                            (root / 'deps', 'MANIFEST.sha256'), (root, 'BUNDLE_MANIFEST.sha256')]:
        for line in (directory / name).read_text().splitlines():
            if not line.strip():
                continue
            digest, relative = line.split('  ', 1)
            path = (directory / relative).resolve()
            if directory.resolve() not in path.parents:
                raise ValueError('Manifest path escapes bundle')
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError('Hash mismatch: ' + relative)
    print('BUNDLE HASHES PASS (identity of this build, not original 9/29 bundle)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--install-deps', action='store_true')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    if args.verify:
        verify(out)
        return
    if out.exists():
        raise SystemExit('Refusing to overwrite an existing bundle: ' + str(out))
    if args.install_deps and (platform.system() != 'Linux' or platform.machine() not in ('aarch64', 'arm64') or sys.version_info[:2] != (3, 8)):
        raise SystemExit('Dependency build requires Linux aarch64, CPython 3.8; use an offline build host, not a live G1 session')
    source = ROOT / 'motiondecode-test'
    # Copy complete integrated code/assets, plus required historical gates; no environments/cache.
    shutil.copytree(source, out / 'code', symlinks=True,
                    ignore=shutil.ignore_patterns('.git', '.cache', '.venv*', '__pycache__', '.pytest_cache', '*.pyc', '*.log'))
    (out / 'deps').mkdir()
    status = 'SOURCE_ONLY_NOT_RUNNABLE'
    if args.install_deps:
        scratch = out / 'build'
        run(['git', 'clone', '--no-checkout', 'https://github.com/eclipse-cyclonedds/cyclonedds.git', scratch / 'cyclonedds'])
        run(['git', '-C', scratch / 'cyclonedds', 'checkout', '--detach', C_DDS])
        actual = subprocess.check_output(['git', '-C', str(scratch / 'cyclonedds'), 'rev-parse', 'HEAD'], text=True).strip()
        if actual != C_DDS:
            raise ValueError('CycloneDDS source revision mismatch')
        native = out / 'code/external/cyclonedds'
        # This is the newly generated bundle, not the checked-in x86_64 snapshot.
        shutil.rmtree(native)
        run(['cmake', '-S', scratch / 'cyclonedds', '-B', scratch / 'cmake',
             '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_EXAMPLES=OFF', '-DBUILD_TESTING=OFF',
             '-DCMAKE_INSTALL_PREFIX=' + str(native), '-DCMAKE_INSTALL_LIBDIR=lib'])
        run(['cmake', '--build', scratch / 'cmake', '--parallel', '2'])
        run(['cmake', '--install', scratch / 'cmake'])
        env = dict(os.environ, CYCLONEDDS_HOME=str(native),
                   LD_LIBRARY_PATH=str(native / 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
        run([sys.executable, '-m', 'venv', scratch / 'build-env'])
        build_python = scratch / 'build-env/bin/python'
        run([build_python, '-m', 'pip', 'install', '-r', ROOT / 'setup/build-requirements.txt'], env=env)
        run([build_python, '-m', 'pip', 'install', '--target', out / 'deps', '--no-deps', '--no-build-isolation', '--no-binary=cyclonedds',
             '-r', ROOT / 'setup/runtime-requirements.txt'], env=env)
        # Import only; do not construct participants, readers, clients, or workers.
        code = ('import sys; sys.path[:0]=[' + repr(str(out / 'deps')) + ',' + repr(str(out / 'code/external')) + ']; '
                'import mujoco,numpy,cyclonedds,unitree_sdk2py; '
                'assert mujoco.__version__=="3.1.6"; print("IMPORT_ONLY PASS")')
        run([sys.executable, '-B', '-c', code], env=env)
        status = 'BUILT_IMPORT_CHECKED_REAL_EXECUTION_UNVERIFIED'
    info = {'code_source': '78bacb68caa620797042703444a8286223663128',
            'python': sys.version, 'architecture': platform.machine(), 'status': status,
            'cyclonedds_source': C_DDS, 'original_bundle_archived': False,
            'requirements_sha256': hashlib.sha256((ROOT / 'setup/runtime-requirements.txt').read_bytes()).hexdigest()}
    (out / 'REBUILD_INFO.json').write_text(json.dumps(info, indent=2) + '\n')
    for directory in (out / 'code', out / 'deps'):
        (directory / 'MANIFEST.sha256').write_text(hashes(directory))
    # Outer manifest includes inner manifests, matching restore script's format.
    lines = hashes(out).splitlines()
    for name in ('code/MANIFEST.sha256', 'deps/MANIFEST.sha256'):
        lines.append(hashlib.sha256((out / name).read_bytes()).hexdigest() + '  ' + name)
    (out / 'BUNDLE_MANIFEST.sha256').write_text('\n'.join(lines) + '\n')
    verify(out)


if __name__ == '__main__':
    main()
