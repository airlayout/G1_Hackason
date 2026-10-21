"""Compare paired recorded realistic batches; do not change the default setting."""
from __future__ import annotations

import argparse
from copy import deepcopy
import html
import json
import os
from pathlib import Path
from statistics import median


TIMEOUT_SEEDS = {1002, 1010, 1013}


def entered_press(result):
    return any(p['phase'] == 'press' for p in result['phases'])


def duration(result, phase):
    return next((p.get('duration_s') for p in result['phases'] if p['phase'] == phase), None)


def compare(baseline, candidate):
    """Enforce matched inputs before reporting an adoption decision."""
    old_cfg, new_cfg = deepcopy(baseline['config']), deepcopy(candidate['config'])
    if old_cfg['wrist_servo']['enabled'] or not new_cfg['wrist_servo']['enabled']:
        raise ValueError('Expected baseline OFF and candidate ON')
    new_cfg['wrist_servo']['enabled'] = False
    if old_cfg != new_cfg:
        raise ValueError('Configurations differ beyond wrist_servo.enabled')
    old = {r['seed']: r for r in baseline['results']}
    new = {r['seed']: r for r in candidate['results']}
    if len(old) != len(baseline['results']) or len(new) != len(candidate['results']):
        raise ValueError('Duplicate seeds')
    if set(old) != set(range(1000, 1020)) or set(new) != set(old):
        raise ValueError('Both batches must contain seeds 1000..1019 exactly once')
    rows = []
    for seed in sorted(old):
        a, b = old[seed], new[seed]
        if a.get('recording_error') or b.get('recording_error'):
            raise ValueError(f'Recording/evaluation exception at {seed}')
        for key in ('target', 'conditions', 'realism', 'eval_set'):
            if a[key] != b[key]:
                raise ValueError(f'Unmatched {key} at {seed}')
        if a['eval_set'] != 'realistic':
            raise ValueError('Expected realistic evaluation')
        if a['max_unintended_force_n'] is None or b['max_unintended_force_n'] is None:
            raise ValueError('Physical contact measurements required')
        rows.append({'seed': seed, 'baseline_success': a['success'], 'candidate_success': b['success'],
                     'baseline_alignment_s': duration(a, 'align'), 'candidate_alignment_s': duration(b, 'align'),
                     'baseline_cycle_s': a['full_cycle_s'], 'candidate_cycle_s': b['full_cycle_s'],
                     'candidate_passed_alignment': entered_press(b),
                     'candidate_corrections': b['corrections'],
                     'candidate_reversals': b.get('correction_reversals', 0),
                     'baseline_error': a['error'], 'candidate_error': b['error']})
    old_success = sum(r['success'] for r in old.values())
    new_success = sum(r['success'] for r in new.values())
    recovered = [s for s in sorted(TIMEOUT_SEEDS) if entered_press(new[s]) and not entered_press(old[s])]
    unchanged_inputs = all(new[s]['max_unintended_force_n'] <= old[s]['max_unintended_force_n'] for s in old)
    press_sources_valid = all(r.get('alignment_camera') == 'wrist' and r.get('surface_camera') == 'head'
                              for r in new.values() if entered_press(r))
    checks = {'recovered_timeout_case': bool(recovered),
              'full_successes_at_least_baseline_and_one': new_success >= max(1, old_success),
              'no_increase_in_unintended_contact': unchanged_inputs,
              'press_uses_wrist_alignment_and_head_surface': press_sources_valid}
    matched_success = [s for s in old if old[s]['success'] and new[s]['success']]
    return {'trials_per_variant': len(old), 'baseline_successes': old_success,
            'candidate_successes': new_success, 'recovered_timeout_seeds': recovered,
            'baseline_physical_presses': sum(bool(r['physical_press_success']) for r in old.values()),
            'candidate_physical_presses': sum(bool(r['physical_press_success']) for r in new.values()),
            'candidate_passed_alignment': sum(entered_press(r) for r in new.values()),
            'baseline_max_unintended_force_n': max(r['max_unintended_force_n'] for r in old.values()),
            'candidate_max_unintended_force_n': max(r['max_unintended_force_n'] for r in new.values()),
            'median_cycle_s_on_shared_successes': {
                'baseline': median(old[s]['full_cycle_s'] for s in matched_success) if matched_success else None,
                'candidate': median(new[s]['full_cycle_s'] for s in matched_success) if matched_success else None},
            'checks': checks, 'eligible_for_default_on': all(checks.values()), 'trials': rows}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline', required=True, type=Path)
    ap.add_argument('--candidate', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    args = ap.parse_args()
    metadata = [json.loads((p/'metadata.json').read_text()) for p in (args.baseline, args.candidate)]
    if any('finished_at_utc' not in m for m in metadata):
        raise ValueError('Both recorded batches must have completed source-hash verification')
    for key in ('sha256', 'runner_sha256', 'packages', 'runtime'):
        if metadata[0][key] != metadata[1][key]:
            raise ValueError(f'Batch metadata differs: {key}')
    payloads = [json.loads((p/'results.json').read_text()) for p in (args.baseline, args.candidate)]
    report = compare(*payloads)
    report['baseline'] = str(args.baseline.resolve())
    report['candidate'] = str(args.candidate.resolve())
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    text = ['# 手首サーボ：realistic 20ケース比較', '',
            f"全工程成功：現行 {report['baseline_successes']}/20、新方式 {report['candidate_successes']}/20。",
            f"新方式の位置合わせ通過：{report['candidate_passed_alignment']}件。時間切れからの改善：{report['recovered_timeout_seeds']}。",
            f"標準ONの条件：{'達成' if report['eligible_for_default_on'] else '未達。既定値OFFを維持'}。", '',
            '成功率と位置合わせ通過率は別に評価する。失敗停止までの短さを成功動作の高速化とは扱わない。',
            'シミュレーション時間とPC実時間は別。現環境の手首画像・関節状態は理想的で、実機での安定性は未検証。', '',
            '| seed | 現行成功 | 新方式成功 | 新方式の位置合わせ(s) | 新方式の結果 |',
            '|---|---|---|---:|---|']
    cards = []
    for row in report['trials']:
        seed = row['seed']
        a, b = row['baseline_success'], row['candidate_success']
        reason = row['candidate_error'] or ('成功' if b else '復帰したが不点灯')
        align = row['candidate_alignment_s']
        align_text = '—' if align is None else f'{align:.2f}'
        text.append(f'| {seed} | {"○" if a else "×"} | {"○" if b else "×"} | {align_text} | {reason} |')
        videos = []
        for label, folder in [('現行 OFF', args.baseline), ('新方式 ON', args.candidate)]:
            relative = html.escape(os.path.relpath(folder/str(seed), args.out), quote=True)
            videos.append(f'<div><h3>{label}</h3><video controls preload="none" poster="{relative}/poster.jpg" src="{relative}/motion.webm"></video><p><a href="{relative}/result.json">結果・診断</a></p></div>')
        cards.append(f'<details><summary>{seed}：現行 {"成功" if a else "失敗"} / 新方式 {html.escape(reason)}</summary><section>{"".join(videos)}</section></details>')
    (args.out/'report.md').write_text('\n'.join(text)+'\n')
    (args.out/'index.html').write_text('<!doctype html><html lang="ja"><meta charset="utf-8"><title>手首サーボ比較</title><style>body{font:16px system-ui;background:#131923;color:#eee;margin:24px}details{padding:12px;border-bottom:1px solid #536}section{display:flex;flex-wrap:wrap;gap:16px}section>div{flex:1;min-width:300px}video{width:100%}a{color:#9cf}</style><h1>realistic：現行と手首サーボ</h1><p>各ケースを開くと両方式の実行映像を見られます。再生時刻はシミュレーション内の時間です。<a href="report.md">集計</a> / <a href="comparison.json">JSON</a></p>'+''.join(cards)+'</html>')
    print(json.dumps({k: v for k, v in report.items() if k != 'trials'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
