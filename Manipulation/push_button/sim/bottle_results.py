"""Outcome reporting shared by the evaluator and saved-result review (no simulator)."""
from collections import Counter


FAILURE_LABELS = {
    'setup_failed': '開始条件不成立',
    'recognition_failed': '認識失敗',
    'contact_not_reached': '接触未到達',
    'push_failed': '押下失敗',
    'return_failed': '復帰失敗',
}


def contact_reached(result):
    evaluation = result['evaluation']
    if 'max_tip_bottle_contact_n' in evaluation:
        return evaluation['max_tip_bottle_contact_n'] > .1
    # Older logs only separated total arm contact and non-tip contact.
    return (evaluation.get('max_arm_bottle_contact_n', 0.) > .1
            and evaluation.get('max_non_tip_bottle_contact_n', 0.) <= .1)


def failure_category(result):
    if result.get('layout_check', {}).get('ready') is False:
        return 'setup_failed'
    report, evaluation = result['controller'], result['evaluation']
    if result['success'] or report.get('diagnostic_completed'):
        return None
    moved = evaluation.get('publish_count', 0) > 0 or any(
        row['phase'] == 'approach' for row in result.get('phases', []))
    home_error = evaluation.get('home_joint_error_rad')
    if (report.get('recovery_error') or report.get('release_error')
            or evaluation.get('released') is False
            or moved and (report.get('home_completed') is False
                          or home_error is not None and home_error > .05)):
        return 'return_failed'
    error = report.get('error') or report.get('verification_error') or ''
    if report.get('visual_success') is not False and any(
            word in error for word in ('計測', '認識', '画像', '対象', '追跡', '台面', '台の')):
        return 'recognition_failed'
    return 'push_failed' if contact_reached(result) else 'contact_not_reached'


def outcome_counts(results):
    counts = Counter(failure_category(row) for row in results)
    return {'contact_reached': sum(contact_reached(row) for row in results),
            'failure_counts': {key: counts[key] for key in FAILURE_LABELS}}
