"""Review saved realistic results. Never runs a detector, renderer, or simulator."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import struct

import numpy as np


def timing_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        duration = row.get('duration_s')
        if duration is not None and np.isfinite(duration) and duration >= 0:
            groups[row['stage']].append(duration)
    return {stage: {'count': len(values), 'median_s': float(np.median(values)),
                    'p95_s': float(np.percentile(values, 95)), 'max_s': max(values)}
            for stage, values in sorted(groups.items())}


def summarize(root):
    results = json.loads((root/'results.json').read_text())
    trials, timings, rejections = [], [], Counter()
    for result in results:
        controller = result['controller']
        evaluation = result['evaluation']
        displacement = evaluation['bottle_displacement_m']
        rejections.update(controller.get('perception_rejections', {}))
        path = root/f'seed_{result["seed"]}'/'processing_times.json'
        if path.is_file():
            timings.extend(json.loads(path.read_text()))
        trials.append({'seed': result['seed'], 'success': result['success'],
            'phase': controller.get('phase'), 'error': controller.get('error'),
            'displacement_mm': [v*1000 for v in displacement],
            'visual_success': controller.get('visual_success'),
            'home_completed': controller.get('home_completed'), 'released': evaluation['released'],
            'home_error_rad': evaluation['home_joint_error_rad'],
            'camera_profile': result.get('camera_profile', 'legacy'),
            'bottle_initial_xy_m': result['bottle_initial_xy_m'],
            'selected_edge_pairs': dict(Counter(','.join(row['selected_edges'])
                for row in controller.get('table_diagnostics', []) if row['status'] == 'accepted'))})
    return {'trials': trials, 'successes': sum(row['success'] for row in trials),
            'trial_count': len(trials), 'charged_processing_times': timing_summary(timings),
            'perception_rejections': dict(rejections),
            'manifest': json.loads((root/'manifest.json').read_text())}


def write_report(out, report):
    (out/'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    lines = [f'# ボトルrealistic評価: {report["successes"]}/{report["trial_count"]}', '',
             '元の採点結果を集計したもの。配置・カメラ条件はreview.jsonのmanifestを参照。', '',
             '| seed | 成功 | 停止段階 | 前方/横 mm | 画像確認 | home | 解除 | 理由 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for row in report['trials']:
        dx, dy = row['displacement_mm'][:2]
        error = str(row['error'] or '').replace('|', '\\|').replace('\n', ' ')
        lines.append(f'| {row["seed"]} | {row["success"]} | {row["phase"]} | {dx:.2f}/{dy:.2f} '
                     f'| {row["visual_success"]} | {row["home_completed"]} | {row["released"]} | {error} |')
    lines.extend(['', '## 評価の経過時間に加算された処理時間（秒）', '',
                  '| 処理 | 件数 | 中央値 | 95%点 | 最大 |', '| --- | --- | --- | --- | --- |'])
    for name, row in report['charged_processing_times'].items():
        lines.append(f'| {name} | {row["count"]} | {row["median_s"]:.4f} | {row["p95_s"]:.4f} | {row["max_s"]:.4f} |')
    (out/'review.md').write_text('\n'.join(lines)+'\n')


def packet_color(packet):
    import cv2
    if len(packet) < 12 or packet[:8] != b'G1RGBD1\0':
        raise ValueError('不正なRGB-Dパケットです')
    size = struct.unpack('<I', packet[8:12])[0]
    header = json.loads(packet[12:12+size])
    color = cv2.imdecode(np.frombuffer(packet[12+size:12+size+header['color']['nbytes']], np.uint8),
                         cv2.IMREAD_COLOR)
    if color is None:
        raise ValueError('保存カラー画像をデコードできません')
    return color


def recognition_video(seed_dir, out, result):
    import cv2
    frames = []
    for path in sorted((seed_dir/'frames').glob('*.json')):
        metadata = json.loads(path.read_text())
        frames.append((metadata['captured_at_s'], path, metadata))
    if not frames:
        raise ValueError(f'保存RGB-Dがありません: {seed_dir}')
    tables = {round(row['captured_at_s'], 6): row for row in result['controller'].get('table_diagnostics', [])}
    bottles = {row['sequence']: row for row in result['controller'].get('perception_frames', [])}
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'VP80'), 8, (640, 480))
    if not writer.isOpened():
        raise RuntimeError('認識映像のエンコーダを開けません')
    index = 0
    cached = None
    try:
        for stamp in np.arange(frames[0][0], frames[-1][0]+1/8, 1/8):
            while index+1 < len(frames) and frames[index+1][0] <= stamp:
                index += 1
            if cached is None or cached[0] != index:
                captured, path, metadata = frames[index]
                color = packet_color(path.with_suffix('.rgbd').read_bytes())
                if color.shape[:2] != (480, 640):
                    raise ValueError('認識映像は640x480の記録に対応しています')
                table = tables.get(round(captured, 6), {})
                selected = table.get('selected_edges', []) if table.get('status') == 'accepted' else []
                for name, edge in table.get('edges', {}).items():
                    paint = (0, 255, 0) if name in selected else (0, 200, 255)
                    if edge['status'] == 'rejected':
                        paint = (0, 0, 255)
                    for u, v in edge.get('pixels_uv', []):
                        cv2.circle(color, (int(u), int(v)), 2, paint, -1)
                for detection in bottles.get(int(path.stem), {}).get('detections', []):
                    x0, y0, x1, y1 = map(lambda v: int(round(v)), detection['bbox'])
                    cv2.rectangle(color, (x0, y0), (x1, y1), (255, 160, 0), 2)
                age = metadata['received_at_s']-captured
                labels = [f'seed {result["seed"]} frame {path.stem} captured={captured:.3f}s delivery_age={age:.3f}s',
                          f'TABLE {table.get("status", "not logged")} {",".join(selected)}',
                          'edge candidates: green=selected yellow=valid red=rejected']
                for n, label in enumerate(labels):
                    cv2.rectangle(color, (0, n*20), (640, (n+1)*20), (0, 0, 0), -1)
                    cv2.putText(color, label, (5, 15+n*20), cv2.FONT_HERSHEY_SIMPLEX, .4,
                                (255, 255, 255), 1, cv2.LINE_AA)
                cached = index, color
            writer.write(cached[1])
    finally:
        writer.release()


def montage(root, out, results):
    import cv2
    captures, last = [], []
    try:
        for result in results:
            path = root/f'seed_{result["seed"]}'/'motion.webm'
            capture = cv2.VideoCapture(str(path))
            captures.append(capture)
            if not capture.isOpened() or abs(capture.get(cv2.CAP_PROP_FPS)-8) > .01:
                raise ValueError(f'8fpsの元映像がありません: {path}')
            last.append(None)
        columns = min(5, len(results))
        rows = math.ceil(len(results)/columns)
        writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'VP80'), 8, (columns*320, rows*200))
        if not writer.isOpened():
            raise RuntimeError('一覧映像のエンコーダを開けません')
        try:
            while True:
                active = False
                canvas = np.zeros((rows*200, columns*320, 3), np.uint8)
                for i, (capture, result) in enumerate(zip(captures, results)):
                    ok, frame = capture.read()
                    if ok:
                        active = True
                        last[i] = cv2.resize(frame, (320, 200))
                    if last[i] is None:
                        raise ValueError(f'映像にフレームがありません: seed {result["seed"]}')
                    tile = last[i].copy()
                    cv2.rectangle(tile, (0, 0), (160, 22), (0, 0, 0), -1)
                    label = f'seed {result["seed"]}: {"PASS" if result["success"] else "FAIL"}'
                    cv2.putText(tile, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                    canvas[i//columns*200:(i//columns+1)*200, i%columns*320:(i%columns+1)*320] = tile
                if not active:
                    break
                writer.write(canvas)
        finally:
            writer.release()
    finally:
        for capture in captures:
            capture.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--out', type=Path, required=True, help='新規の出力ディレクトリ')
    parser.add_argument('--videos', action='store_true', help='元映像の一覧と保存RGB-Dの認識映像も生成する')
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error('既存出力は上書きしません')
    report = summarize(args.run)
    if not report['trials']:
        parser.error('完了した試行がありません')
    args.out.mkdir(parents=True)
    write_report(args.out, report)
    if args.videos:
        results = json.loads((args.run/'results.json').read_text())
        montage(args.run, args.out/'ten_trials.webm', results)
        for result in results:
            recognition_video(args.run/f'seed_{result["seed"]}',
                              args.out/f'seed_{result["seed"]}_recognition.webm', result)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
