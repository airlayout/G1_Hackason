import subprocess
import sys
import json
import numpy as np
import pytest
from common import ROOT, NAMES
from scene import make_scene, set_arms, pair_clearances, clearance_pairs, measured_pose_clearances
from measure_hold_baseline_noise import series_stats, classify, corr, observed_internal_profile, RecordingState
from robot_transport import ReadOnlyState


def test_shared_measured_FK_matches_original_runtime_code_exactly():
    snapshot=json.loads((ROOT/'tests/fixtures/blocked_pose.json').read_text())
    model,data=make_scene();pairs=clearance_pairs(model);reference=model.qpos0.copy()
    q=np.array(snapshot['all_q']);q[15]+=.00003;q[12]-=.00001
    original=reference.copy()
    for i,n in enumerate(NAMES):original[model.joint(n).qposadr[0]]=q[i]
    set_arms(model,data,q[15:],original)
    expected=pair_clearances(model,data,pairs)
    actual,timing=measured_pose_clearances(model,data,reference,pairs,q)
    assert actual==expected
    assert timing['fk_s']>=0 and timing['collision_s']>=0


def test_statistics_keep_baseline_deviation_separate_from_centered_MAD():
    s=series_stats(np.array([8.,9.,10.,11.,12.]),initial=11.)
    assert s['mean']==s['median']==10.
    assert s['MAD']==1. and s['peak_to_peak']==4.
    assert s['maximum_baseline_worsening']==3.
    assert s['p99_absolute_deviation_from_q0']==pytest.approx(2.96)


@pytest.mark.parametrize('fractions,label',[([0,0],'NOISE-C'),([.001,0],'NOISE-B'),([.05,.001],'NOISE-A')])
def test_explicit_descriptive_noise_classification(fractions,label):
    assert classify([{'fraction_worsening_over_current_tolerance':f} for f in fractions])==label


def test_constant_series_does_not_fabricate_correlation():
    assert corr([1,1,1],[1,2,3]) is None
    assert corr([1,2,3],[3,2,1])==pytest.approx(-1.)


def test_measurement_denies_sdk_publisher_and_direct_dds_datawriter():
    code='''import sys
sys.path.insert(0,'scripts')
from measure_hold_baseline_noise import forbid_writers
forbid_writers()
from unitree_sdk2py.core.channel import ChannelPublisher
from cyclonedds.pub import DataWriter
for constructor in (ChannelPublisher,DataWriter):
    try: constructor(None,None)
    except RuntimeError as e: assert 'RECEIVE ONLY' in str(e)
    else: raise AssertionError('Publisher construction was allowed')
'''
    run=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,text=True)
    assert run.returncode==0,run.stderr


def test_observation_binding_rejects_active_and_foreign_service(tmp_path):
    from datetime import datetime
    template=json.loads((ROOT/'output/arm_control_ownership_20260912_210935.json').read_text())
    template['checked_local_time']=datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %z')
    p=tmp_path/'identity_observation.json';p.write_text(json.dumps(template))
    assert observed_internal_profile(p)['hostname']=='Unitree'
    template['topics']['rt/arm_sdk']['total_sample_count']=1
    p.write_text(json.dumps(template))
    with pytest.raises(ValueError):observed_internal_profile(p)
    template['topics']['rt/arm_sdk']['total_sample_count']=0
    template['topics']['rt/arm_sdk']['publishers'][0]['source_ips']=['192.168.123.222']
    p.write_text(json.dumps(template))
    with pytest.raises(ValueError):observed_internal_profile(p)


def test_identity_binding_requires_exact_observed_participant(monkeypatch):
    record={'participant_guid':'A','hostname':'Unitree','process_name':'python3','pid':'2884',
            'source_ips':['192.168.123.161'],'related_publications':['arm'],
            'related_subscriptions':['lowstate'],'classification':'UNKNOWN EXTERNAL PARTICIPANT'}
    reader=object.__new__(RecordingState);reader.internal_profile=dict(record)
    monkeypatch.setattr(ReadOnlyState,'_endpoint_record',lambda *args:dict(record))
    assert reader._endpoint_record(None,None,None,None)['classification']=='EXPECTED ROBOT INTERNAL PARTICIPANT'
    record['participant_guid']='B'
    assert reader._endpoint_record(None,None,None,None)['classification']=='UNKNOWN EXTERNAL PARTICIPANT'
