import json
from dataclasses import replace
import pytest
from phase35.profiles import PROFILES, FINITE_PLANNER
from phase35 import codecs
from phase35.dual_rate import SharedState, Stamp, resolve, execute_resolved
from phase35.runtime import LatencyBrain
from brain.split_schemas import CurrentObservation, PlannerDecision
from brain.state import State
from latency.profiles import PROFILES as OLD
from latency.codecs import PLANNER_CLASS
from adapters.robot_mock import MockRobot


@pytest.mark.parametrize('code,side',[('A','UNKNOWN'),('B','LEFT'),('C','CENTER'),('D','RIGHT')])
def test_fast_geometry_mapping(code,side):
    obs=codecs.observation(code,'class')
    assert obs.person_visible==(code!='A') and obs.person_direction==side
    assert obs.person_distance==obs.blocking_obstacle=='UNKNOWN'
    assert PROFILES['A'].linear_patch and PROFILES['A'].budget('perception')==1


@pytest.mark.parametrize('raw',[' B','B\n','AB','X','```B```','SEARCH_LEFT','',None])
def test_invalid_finite_output_is_not_repaired(raw):
    with pytest.raises((ValueError,TypeError)):
        codecs.decision(raw,'planner_class',codecs.observation('B','class'))


@pytest.mark.parametrize('code',PLANNER_CLASS)
@pytest.mark.parametrize('distance',['NEAR','MID','FAR','UNKNOWN'])
def test_planner_class_only_binds_distance_never_changes_action(code,distance):
    obs=CurrentObservation(person_visible=True,person_count=1,person_direction='LEFT',
                           person_distance=distance,blocking_obstacle='NONE')
    before=obs.model_dump_json(); result=codecs.decision(code,'planner_class',obs)
    action,direction,target,range_=PLANNER_CLASS[code]
    assert (result.action,result.direction,result.target)==(action,direction,target)
    assert result.distance==(distance if action in {'LOOK','FOUND','APPROACH'} else range_)
    assert obs.model_dump_json()==before


def test_b_perception_prompt_budget_backend_match_reference():
    p=PROFILES['B']; ref=OLD['json_linear_uninstrumented']
    assert p.prompt('perception')==ref.prompt('perception')
    assert p.budget('perception')==ref.budget('perception')
    assert p.linear_patch==ref.linear_patch and p.dtype==ref.dtype and p.edge==ref.edge
    assert p.budget('planner')==1
    assert all(x not in FINITE_PLANNER for x in ['IMG_7121','timestamp','expected_action','ground_truth'])


@pytest.mark.parametrize('returned',['E','G','M','invalid'])
def test_disappearance_never_forces_search_left(returned):
    images=[object(),object()]; seen=[]
    class Engine:
        profile=PROFILES['C']
        index=0
        def perceive(self,image):
            assert image is images[self.index]
            raw=['B','A'][self.index]; self.index+=1
            return raw,{}
        def plan(self,obs,state):
            seen.append((obs,state))
            return ('A' if obs.person_visible else returned),{}
    brain=LatencyBrain(Engine()); brain.decide(images[0]); result=brain.decide(images[1])
    assert seen[-1][1].transition=='DISAPPEARED'
    assert seen[-1][1].last_seen_person_direction=='LEFT'
    if returned=='invalid': assert result['planner']['fallback'] and result['planner']['decision']['action']=='WAIT'
    else:
        action,side,_,_=PLANNER_CLASS[returned]
        assert result['planner']['decision']['action']==action and result['planner']['decision']['direction']==side
    assert result['state_after']['frames_since_person_seen']==1


def test_logits_argmax_selects_wrong_direction_without_policy_override():
    import torch
    from latency.engine import AllowedCodes
    scores=torch.tensor([[.1,.9,.4,.3]])
    selected=AllowedCodes([0,2,3])(torch.tensor([[1]]),scores)
    assert selected.argmax().item()==2  # Token 1 is excluded; allowed token 2 wins.
    assert selected[0,1].isneginf()


def test_stale_semantic_cannot_overwrite_geometry_or_history():
    shared=SharedState(); left=codecs.observation('B','class'); right=codecs.observation('D','class')
    shared.update_fast(left,Stamp(8,1)); shared.update_fast(right,Stamp(12,2))
    before=shared.state.snapshot
    shared.update_semantic(left,Stamp(8,1))
    assert shared.combined().person_direction=='RIGHT' and shared.state.snapshot is before
    assert not shared.update_fast(left,Stamp(8,3))
    assert not shared.update_fast(left,Stamp(12,1))
    assert shared.update_fast(left,Stamp(12,3))


def test_distance_and_obstacle_have_matching_frame_generation_requirement():
    shared=SharedState(); fast=codecs.observation('B','class')
    semantic=CurrentObservation(person_visible=True,person_count=1,person_direction='LEFT',
        person_distance='NEAR',blocking_obstacle='RIGHT')
    shared.update_fast(fast,Stamp(8,1)); shared.update_semantic(semantic,Stamp(8,1))
    assert shared.combined().person_distance=='NEAR'
    shared.update_fast(fast,Stamp(8,2))
    assert shared.combined().person_distance=='UNKNOWN'


def test_execution_binding_preserves_semantic_action_and_original_decision():
    shared=SharedState(); shared.update_fast(codecs.observation('D','class'),Stamp(12,2))
    original=PlannerDecision(action='LOOK',direction='LEFT',target='PERSON',distance='NEAR')
    robot=MockRobot(); resolved=execute_resolved(robot,original,shared)
    assert isinstance(resolved,PlannerDecision)
    assert resolved.direction=='RIGHT' and resolved.action==original.action and original.direction=='LEFT'
    assert 'LOOK RIGHT' in robot.last_output
    search=PlannerDecision(action='SEARCH',direction='LEFT',target='PERSON',distance='UNKNOWN')
    assert resolve(search,shared) is search
    shared.update_fast(codecs.observation('A','class'),Stamp(13,3))
    assert resolve(original,shared) is None  # Suppress, never invent WAIT/SEARCH.
    with pytest.raises(TypeError): execute_resolved(robot,original.model_dump(),shared)


@pytest.mark.parametrize('timestamp,generation',[(float('nan'),0),(float('inf'),0),(1,-1)])
def test_invalid_stamp(timestamp,generation):
    with pytest.raises(ValueError): Stamp(timestamp,generation)


@pytest.mark.parametrize('report_name',['B','B2'])
def test_gpu_report_proves_verified_tokens_no_image_no_gt_and_reuse(report_name):
    # Saved real-GPU evidence, available after the explicit benchmark command.
    from pathlib import Path
    path=Path(f'reports/phase35/{report_name}.json')
    if not path.exists(): pytest.skip('GPU benchmark evidence not present')
    report=json.loads(path.read_text()); ids=report['single_token_codes']
    assert all(len(v)==1 for v in ids.values()) and len({v[0] for v in ids.values()})==len(ids)
    rows=[f for r in report['runs'] for f in r['frames']]
    model_ids=set(); processor_ids=set()
    for f in rows:
        p,d=f['perception']['metrics'],f['planner']['metrics']
        assert not p['instrumented'] and d['instrumented']
        assert p['image_count']==1 and d['image_count']==0 and d['output_tokens']==1
        assert set(d['planner_input'])=={'current_observation','memory','transition'}
        assert d['planner_input']['current_observation']==f['perception']['observation']
        assert d['generated_token_ids']==ids[f['planner']['raw_response']]
        allowed=d['class_allowed_token_ids']; logits=d['class_logits']
        assert allowed[logits.index(max(logits))]==d['generated_token_ids'][0]
        assert d['model_load_count']==p['model_load_count']==1
        model_ids.update([p['model_instance_id'],d['model_instance_id']])
        processor_ids.update([p['processor_instance_id'],d['processor_instance_id']])
    assert len(model_ids)==len(processor_ids)==1


@pytest.mark.parametrize('raw,metrics',[('invalid',{}),('B',{'class_logits_finite':False})])
def test_invalid_fast_geometry_preserves_state_and_skips_planner(raw,metrics):
    class Engine:
        profile=PROFILES['C']
        def perceive(self,image): return raw,metrics.copy()
        def plan(self,*args): pytest.fail('Invalid geometry must not reach Planner')
    brain=LatencyBrain(Engine()); before=brain.state.snapshot; result=brain.decide(object())
    assert brain.state.snapshot is before and result['perception']['fallback']
    assert result['planner']['fallback'] and result['planner']['decision']['action']=='WAIT'


def test_gpu_a_evidence_combines_linear_and_one_token():
    from pathlib import Path
    report=json.loads(Path('reports/phase35/A.json').read_text())
    assert report['profile']['linear_patch'] and report['profile']['constrained']
    for run in report['runs']:
        for frame in run['frames']:
            metrics=frame['perception']['metrics']; raw=frame['perception']['raw_response']
            assert metrics['output_tokens']==1 and metrics['decode_steps']==0
            assert metrics['patch_embed_cuda_elapsed_s']>0 and metrics['vision_cuda_elapsed_s']>0
            assert metrics['generated_token_ids']==report['single_token_codes'][raw]
            ids,logits=metrics['class_allowed_token_ids'],metrics['class_logits']
            assert ids[logits.index(max(logits))]==metrics['generated_token_ids'][0]


def test_c_gpu_runner_rejects_failed_b_before_loading_model():
    import subprocess
    import sys
    result=subprocess.run([sys.executable,'benchmark_phase35.py','unused.mp4','--experiments','C'],
                          capture_output=True,text=True,check=False)
    assert result.returncode==2
    assert 'independently evaluated A and B accuracy gates PASS' in result.stderr


def test_post_run_evaluation_preserves_raw_and_rejects_action_rewrite():
    import copy
    from pathlib import Path
    from evaluate_phase35 import evaluate
    report=json.loads(Path('reports/phase35/B.json').read_text()); before=copy.deepcopy(report)
    data=[json.loads((Path('scenarios')/name).read_text()) for name in
          ['IMG_7121_phase32_perception.json','IMG_7121_phase33_nine.json','IMG_7121_phase33_three.json']]
    result=evaluate(report,*data)
    assert report==before and not result['eligible'] and result['integrity_pass']
    # Even rewriting the failed 10s result to expected SEARCH cannot earn PASS.
    frame=report['runs'][-1]['frames'][1]
    frame['planner']['decision']['action']='SEARCH'; frame['planner']['decision']['direction']='LEFT'
    frame['planner']['decision']['target']='PERSON'
    result=evaluate(report,*data)
    assert not result['eligible'] and result['serialization_errors']
