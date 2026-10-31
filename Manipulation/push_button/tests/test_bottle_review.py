"""Saved-result reporting without decoding images or creating videos."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
from review_realistic_bottle import main, summarize, timing_summary


def test_timing_summary_reports_percentiles_and_ignores_invalid_costs():
    rows = [{'stage': 'detector', 'duration_s': t} for t in [.01, .02, .03, float('nan'), -.1]]
    result = timing_summary(rows)['detector']
    assert result == pytest.approx({'count': 3, 'median_s': .02, 'p95_s': .029, 'max_s': .03})


def test_report_preserves_failure_and_configuration_without_videos(tmp_path):
    run = tmp_path/'run'
    (run/'seed_7').mkdir(parents=True)
    result = {'seed': 7, 'success': False, 'camera_profile': 'head', 'bottle_initial_xy_m': [.36, -.13],
        'controller': {'phase': 'measure', 'error': '台|不明', 'perception_rejections': {'台の二辺': 3},
                       'home_completed': False, 'visual_success': None},
        'evaluation': {'bottle_displacement_m': [0., 0., 0.], 'released': True, 'home_joint_error_rad': .002}}
    (run/'results.json').write_text(json.dumps([result]))
    (run/'manifest.json').write_text(json.dumps({'options': {'bottle_x': .36, 'bottle_y': -.13}}))
    report = summarize(run)
    assert report['successes'] == 0 and report['trial_count'] == 1
    assert report['perception_rejections'] == {'台の二辺': 3}
    out = tmp_path/'review'
    assert main([str(run), '--out', str(out)]) == 0
    assert '台\\|不明' in (out/'review.md').read_text()
    assert json.loads((out/'review.json').read_text())['trials'][0]['visual_success'] is None
    assert not list(out.glob('*.webm'))
    with pytest.raises(SystemExit):
        main([str(run), '--out', str(out)])
