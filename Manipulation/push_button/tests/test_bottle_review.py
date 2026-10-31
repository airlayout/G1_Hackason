"""Saved-result reporting without decoding images or creating videos."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
from review_realistic_bottle import main, summarize, timing_summary
from bottle_results import contact_reached, failure_category, outcome_counts


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


@pytest.mark.parametrize('updates,category', [
    ({'layout_check': {'ready': False}}, 'setup_failed'),
    ({'controller': {'error': '安定した新しい3フレームのボトル計測を取得できません'}}, 'recognition_failed'),
    ({'controller': {'phase': 'push'}}, 'contact_not_reached'),
    ({'evaluation': {'max_tip_bottle_contact_n': 2.}}, 'push_failed'),
    ({'controller': {'recovery_error': 'IK failed'}}, 'return_failed'),
    ({'controller': {'home_completed': False}, 'evaluation': {'publish_count': 10}}, 'return_failed'),
    ({'controller': {'verification_error': '画像で確認したボトルの移動量が許容範囲外です',
                     'visual_success': False}, 'evaluation': {'max_tip_bottle_contact_n': 2.}}, 'push_failed'),
    ({'controller': {'verification_error': '安定計測が不能'},
      'evaluation': {'max_tip_bottle_contact_n': 2.}}, 'recognition_failed'),
    ({'success': True}, None),
    ({'controller': {'diagnostic_completed': True}}, None),
])
def test_failure_categories_and_real_contact_are_independent(updates, category):
    result = {'success': False, 'controller': {}, 'evaluation': {'released': True}}
    for key, value in updates.items():
        if isinstance(value, dict):
            result.setdefault(key, {}).update(value)
        else:
            result[key] = value
    assert failure_category(result) == category
    assert contact_reached(result) == (result['evaluation'].get('max_tip_bottle_contact_n', 0) > .1)


def test_setup_failure_report_and_videos_work_without_simulation_artifacts(tmp_path):
    run = tmp_path/'run'
    run.mkdir()
    result = {'seed': 0, 'success': False, 'layout_check': {'ready': False},
              'bottle_initial_xy_m': [.33, -.25], 'controller': {'phase': 'setup', 'error': '画角外'},
              'evaluation': {'bottle_displacement_m': None, 'released': True, 'home_joint_error_rad': None}}
    (run/'results.json').write_text(json.dumps([result]))
    (run/'manifest.json').write_text('{}')
    out = tmp_path/'review'
    assert main([str(run), '--out', str(out), '--videos']) == 0
    report = json.loads((out/'review.json').read_text())
    assert report['trial_count'] == 1 and report['successes'] == report['contact_reached'] == 0
    assert report['failure_counts']['setup_failed'] == 1
    assert '開始条件不成立' in (out/'review.md').read_text()
    assert not list(out.glob('*.webm'))


def test_old_contact_logs_do_not_count_non_tip_collisions_as_tip_contact():
    result = {'evaluation': {'max_arm_bottle_contact_n': 2., 'max_non_tip_bottle_contact_n': 2.}}
    assert not contact_reached(result)
    result['evaluation']['max_non_tip_bottle_contact_n'] = 0.
    assert contact_reached(result)
