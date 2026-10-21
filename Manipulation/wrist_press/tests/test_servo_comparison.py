from copy import deepcopy

import pytest

from Manipulation.wrist_press.compare_servo import compare
from Manipulation.wrist_press.settings import load_settings


def batches():
    cfg = load_settings()
    rows = []
    for seed in range(1000, 1020):
        success = seed == 1001
        rows.append({'seed': seed, 'target': 'up', 'conditions': {'seed': seed},
                     'realism': {}, 'eval_set': 'realistic', 'success': success,
                     'physical_press_success': success, 'max_unintended_force_n': 0.,
                     'full_cycle_s': 10.3 if success else 7., 'corrections': 1,
                     'error': '' if success else 'alignment timeout',
                     'phases': [{'phase': 'align', 'duration_s': .3 if success else 2.}]
                               + ([{'phase': 'press'}] if success else []),
                     'alignment_camera': 'wrist', 'surface_camera': 'head'})
    old = {'config': cfg, 'results': rows}
    new = deepcopy(old)
    new['config']['wrist_servo']['enabled'] = True
    return old, new


def test_adoption_requires_recovered_alignment_and_at_least_one_full_success():
    old, new = batches()
    assert not compare(old, new)['eligible_for_default_on']
    new['results'][2]['phases'].append({'phase': 'press'})
    assert compare(old, new)['eligible_for_default_on']
    for payload in (old, new):
        payload['results'][1]['success'] = False
    assert not compare(old, new)['eligible_for_default_on']


def test_more_contact_or_wrong_source_blocks_adoption():
    old, new = batches()
    new['results'][2]['phases'].append({'phase': 'press'})
    new['results'][2]['max_unintended_force_n'] = .01
    assert not compare(old, new)['eligible_for_default_on']
    new['results'][2]['max_unintended_force_n'] = 0.
    new['results'][2]['alignment_camera'] = 'head'
    assert not compare(old, new)['eligible_for_default_on']


def test_comparison_refuses_unmatched_inputs_duplicate_seeds_and_missing_measurements():
    old, new = batches()
    new['config']['alignment_timeout_s'] = 6
    with pytest.raises(ValueError, match='Configurations differ'):
        compare(old, new)
    old, new = batches()
    new['results'].append(deepcopy(new['results'][0]))
    with pytest.raises(ValueError, match='Duplicate seeds'):
        compare(old, new)
    old, new = batches()
    new['results'][0]['conditions'] = {'seed': 42}
    with pytest.raises(ValueError, match='Unmatched conditions'):
        compare(old, new)
    old, new = batches()
    new['results'][0]['max_unintended_force_n'] = None
    with pytest.raises(ValueError, match='contact measurements'):
        compare(old, new)
