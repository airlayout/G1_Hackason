#!/usr/bin/env python3
"""Control a connection-independent G1 recording via the user systemd manager."""
import argparse
import datetime
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path.home() / 'mapping_runs'
CURRENT = ROOT / '.g1_mapping_current'
UNIT = 'g1-mapping-record.service'
CAMERA = 'g1-mapping-camera.service'
RECORDER = Path.home() / 'mapping_tools/record_onboard_with_camera.py'


def run(*args, check=True):
    return subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check)


def active(unit):
    return run('systemctl', '--user', 'is-active', '--quiet', unit, check=False).returncode == 0


def session_path():
    if not CURRENT.exists():
        raise RuntimeError('記録セッションがありません')
    session = CURRENT.read_text().strip()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', session):
        raise RuntimeError('セッションIDが不正です')
    return ROOT / session


def main():
    p = argparse.ArgumentParser()
    p.add_argument('action', choices=['start', 'status', 'stop'])
    p.add_argument('--name', default='room')
    args = p.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    if args.action == 'start':
        if active(UNIT):
            raise RuntimeError('既に記録中です')
        if not active(CAMERA):
            raise RuntimeError('G1カメラサービスが稼働していません')
        if not re.fullmatch(r'[A-Za-z0-9_-]+', args.name):
            raise RuntimeError('nameは英数字、ハイフン、アンダースコアを使用してください')
        session = datetime.datetime.now().strftime('%Y%m%dT%H%M%S') + '_' + args.name
        dest = ROOT / session
        if dest.exists():
            raise RuntimeError('セッションが既に存在します')
        log = ROOT / (session + '.log')
        run('systemctl', '--user', 'reset-failed', UNIT, check=False)
        result = run('systemd-run', '--user', '--unit=g1-mapping-record', '--property=Type=exec',
                     '--property=StandardOutput=append:' + str(log),
                     '--property=StandardError=append:' + str(log), '/usr/bin/python3', '-u',
                     str(RECORDER), '--session-id', session, '--name', args.name, '--camera-zmq')
        CURRENT.write_text(session + '\n')
        time.sleep(3)
        if not active(UNIT):
            raise RuntimeError('記録開始に失敗しました。ログ: ' + str(log))
        print('[STARTED] ' + str(dest))
        print('ログ: ' + str(log))
    elif args.action == 'stop':
        dest = session_path()
        if active(UNIT):
            run('systemctl', '--user', 'stop', UNIT)
        state = dest / 'state.json'
        if not state.exists():
            raise RuntimeError('状態ファイルがありません: ' + str(dest))
        print(state.read_text())
        print('[OUTPUT] ' + str(dest))
    else:
        dest = session_path()
        print('[{}] {}'.format('RECORDING' if active(UNIT) else 'STOPPED', dest))
        state = dest / 'state.json'
        if state.exists():
            print(state.read_text())
        index = dest / 'raw/camera/frames.jsonl'
        if index.exists():
            with index.open() as f:
                print('camera frames:', sum(1 for _ in f))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print('[ERROR] {}'.format(error), file=sys.stderr)
        sys.exit(1)
