import time

import numpy as np

from audit_geometry import audit
from common import ROOT
from inspect_arm_sdk_publishers import PROCESS_PATTERN
from safety_monitor import OwnershipWatchdog, assess_runtime_safety


def ownership(passed=True,reason=None):
    return {'passed':passed,'reasons':[] if passed else [reason or 'conflict'],
            'external_arm_sdk_writers':[],'armsdk_writers':[],
            'arm_action':{'status':'IDLE'},'publications':{}}


def test_ownership_fault_latches_until_explicit_stopped_session_reset():
    states=[ownership(False,'external writer'),ownership(True)]
    watchdog=OwnershipWatchdog(lambda:states.pop(0) if states else ownership(True),.005).start()
    time.sleep(.03);snapshot=watchdog.snapshot();watchdog.stop()
    assert snapshot['latched_fault']
    assert not snapshot['ownership_safe']
    assert snapshot['current_cached_safe']
    assert snapshot['latched_reasons']==['external writer']
    watchdog.reset_session();watchdog.start();fresh=watchdog.wait_first();watchdog.stop()
    assert fresh['ownership_safe'] and not fresh['latched_fault']


def test_ownership_discovery_error_is_exposed_without_erasing_last_success():
    calls=0
    def sample():
        nonlocal calls
        calls+=1
        if calls==1:return ownership(True)
        raise RuntimeError('discovery jitter')
    watchdog=OwnershipWatchdog(sample,.01).start();time.sleep(.025)
    result=watchdog.snapshot();watchdog.stop()
    assert result['current_cached_safe']
    assert not result['ownership_safe']
    assert 'discovery jitter' in result['latest_check_error']
    assert result['last_successful_check_age_s'] is not None


def test_fast_cache_reads_do_not_run_discovery_synchronously():
    calls=0
    def discover():
        nonlocal calls
        calls+=1
        return ownership(True)
    watchdog=OwnershipWatchdog(discover,.02).start();watchdog.wait_first()
    for _ in range(2000):assert watchdog.snapshot()['current_cached_safe']
    watchdog.stop()
    assert calls<10


def test_hard_safety_and_existing_baseline_noise_are_separate():
    q=np.zeros(29);owner={'ownership_safe':True}
    result=assess_runtime_safety(q,q,q,{'existing':-.00302},{'existing':-.003},.001,owner)
    assert result['hard_safety']['passed']
    assert result['baseline_relative']['legacy_threshold_exceeded_pairs']==['existing']
    assert not result['baseline_relative']['calibrated_threshold_applied']
    result=assess_runtime_safety(q,q,q,{'new':-.001},{'new':.002},.001,owner)
    assert not result['hard_safety']['passed']
    assert result['hard_safety']['new_penetration_pairs']==['new']


def test_generic_ssh_unitree_destination_is_not_arm_controller_candidate():
    assert not PROCESS_PATTERN.search('ssh -fN unitree@10.42.0.76')
    assert PROCESS_PATTERN.search('python scripts/run_real_reaction.py')


def test_geometry_audit_uses_exact_runtime_pairs_and_refuses_classification(tmp_path):
    q0=ROOT/'output/hold_baseline_noise_20260913_165151/fresh_q0.json'
    result=audit(q0,tmp_path/'geometry_audit',preview=False,current_fresh=False)
    assert result['overall_classification']=='GEO-E'
    assert not result['allowlist_change']
    distances={tuple(x['pair']):x['signed_distance_mm'] for x in result['pairs']}
    np.testing.assert_allclose(distances['right_wrist_yaw_link','right_hip_pitch_link'],
                               -2.980470734706089,rtol=0,atol=1e-9)
    assert distances['right_rubber_hand','right_hip_pitch_link']< -4.26
    for pair in result['pairs']:
        for name in pair['pair']:
            selected=[x for x in pair['geometry'][name]['compiled_geometries']
                      if x['selected_by_runtime_pair_builder']]
            assert len(selected)==1 and selected[0]['mesh_name']==name
