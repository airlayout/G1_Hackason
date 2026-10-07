import json
from dataclasses import replace
import pytest
from pydantic import ValidationError
from latency import codecs
from latency.profiles import Profile,PROFILES
from latency.runtime import LatencyBrain
from brain.split_prompt import PERCEPTION_SYSTEM,PERCEPTION_USER,PLANNER_SYSTEM


@pytest.mark.parametrize('raw,visible,direction',[('1|L|N|0',True,'LEFT'),('1|C|M|R',True,'CENTER'),
                                               ('0|U|U|L',False,'UNKNOWN'),('1|R|F|U',True,'RIGHT')])
def test_compact_observation_roundtrip(raw,visible,direction):
    obs=codecs.observation(raw,'compact')
    assert obs.person_visible==visible and obs.person_direction==direction
    assert codecs.observation(obs.model_dump_json(),'json')==obs


@pytest.mark.parametrize('raw',['1|L|N','1|L|N|0|X','1| L|N|0','2|L|N|0','0|L|U|0',
                               '0|U|N|0','1|Z|N|0','1|L|Q|0','1|L|N|Q','```1|L|N|0```',
                               'reply:1|L|N|0','',None])
def test_compact_rejects_without_repair(raw):
    with pytest.raises((ValueError,ValidationError)): codecs.observation(raw,'compact')


@pytest.mark.parametrize('code,action',[('L','LOOK'),('A','APPROACH'),('S','SEARCH'),('X','AVOID_LEFT'),
                                      ('Y','AVOID_RIGHT'),('F','FOUND'),('R','SURPRISE'),('W','WAIT')])
def test_compact_action_mapping(code,action):
    decision=codecs.decision(code+'|L|P|U','compact')
    assert decision.action==action
    assert codecs.decision(decision.model_dump_json(),'json')==decision


@pytest.mark.parametrize('raw',['V|L|P|U','S|L|P|U extra','S|L|P','S|UP|P|U','S|L|Q|U','S|L|P|Z'])
def test_planner_compact_strict(raw):
    with pytest.raises(ValueError): codecs.decision(raw,'compact')


@pytest.mark.parametrize('code,direction',[('A','UNKNOWN'),('B','LEFT'),('C','CENTER'),('D','RIGHT')])
def test_class_loss_of_range_and_obstacle_is_explicit(code,direction):
    obs=codecs.observation(code,'class')
    assert obs.person_direction==direction and obs.person_visible==(code!='A')
    assert obs.person_distance==obs.blocking_obstacle=='UNKNOWN'


@pytest.mark.parametrize('raw',['E','AB','B|L','B explanation',''])
def test_class_output_not_repaired(raw):
    with pytest.raises(ValueError): codecs.observation(raw,'class')


def test_json_profile_preserves_frozen_prompts():
    assert PROFILES['json'].prompt('perception')==(PERCEPTION_SYSTEM,PERCEPTION_USER)
    assert PROFILES['json'].prompt('planner')==(PLANNER_SYSTEM,None)
    for profile in PROFILES.values():
        assert all(s not in str(profile.prompt('perception'))+str(profile.prompt('planner')) for s in ['IMG_7121','timestamp','expected_action'])


@pytest.mark.parametrize('raw',['L|L|P|N','W|U|0|U','{bad}'])
def test_experimental_brain_state_truth_and_no_action_override(raw):
    token=object()
    class Engine:
        profile=Profile('test',format='compact')
        def perceive(self,image):
            assert image is token
            return '1|L|N|0',{}
        def plan(self,obs,state):
            with pytest.raises(ValidationError): obs.person_visible=False
            with pytest.raises(ValidationError): state.last_seen_person_direction='RIGHT'
            return raw,{}
    result=LatencyBrain(Engine()).decide(token)
    assert result['state_after']['last_seen_person_direction']=='LEFT'
    assert result['perception']['raw_response']=='1|L|N|0'
    assert result['planner']['decision']['action']==('LOOK' if raw.startswith('L') else 'WAIT')
    assert result['planner']['fallback']==(raw=='{bad}')


def test_invalid_compact_does_not_invent_absence_or_call_planner():
    class Engine:
        profile=Profile('test',format='compact')
        def perceive(self,image): return '0|L|N|0',{}
        def plan(self,*args): pytest.fail('Invalid Observation cannot reach Planner')
    result=LatencyBrain(Engine()).decide(object())
    assert result['state_before']==result['state_after'] and result['perception']['observation'] is None
    assert result['planner']['decision']['action']=='WAIT'


def test_percentiles_and_budgets():
    from evaluate_latency import distribution
    assert distribution([None,1,2,3])=={'count':3,'mean':2,'median':2,'p95':2.9}
    assert Profile('class',format='class').budget('perception')==1
    assert Profile('compact',format='compact').budget('planner')==8
    assert Profile('json').budget('perception')==96


def test_finite_code_selection_is_logits_not_geometry_or_labels():
    import torch
    from latency.engine import AllowedCodes
    selector=AllowedCodes([2,4,6,8])
    scores=torch.tensor([[100.,100.,1.,100.,3.,100.,7.,100.,2.,100.]])
    masked=selector(torch.zeros((1,1),dtype=torch.long),scores)
    assert masked.argmax().item()==6
    assert torch.isneginf(masked[0,[0,1,3,5,7,9]]).all()
    assert selector.last_scores.tolist()==[[1.,3.,7.,2.]]
    assert scores[0,0].item()==100.


def test_class_json_planner_preserves_current_policy_and_full_json_contract():
    profile=PROFILES['class_constrained']
    assert profile.stage_format('perception')=='class' and profile.stage_format('planner')=='json'
    assert profile.prompt('planner')==(PLANNER_SYSTEM,None)
    assert profile.budget('perception')==1 and profile.budget('planner')==96


def test_compact_evaluation_view_preserves_raw_and_rejects_false_action():
    from evaluate_latency import evaluate_run
    from adapters.robot_mock import MockRobot
    class Engine:
        profile=Profile('test',format='compact')
        def perceive(self,image): return '1|L|N|0',{'image_count':1,'model_load_count':1,'model_instance_id':1,'processor_instance_id':2}
        def plan(self,obs,state): return 'L|L|P|N',{'image_count':0,'model_load_count':1,'model_instance_id':1,'processor_instance_id':2}
    from brain.split_schemas import PlannerDecision
    frame=LatencyBrain(Engine()).decide(object()); robot=MockRobot()
    robot.execute(PlannerDecision.model_validate(frame['planner']['decision']))
    frame.update(timestamp=8,mock_robot_output=robot.last_output)
    run={'status':'pass','timestamps':[8],'frames':[frame]}
    original=json.dumps(run)
    scenario={'timestamps':[8],'checks':[dict(timestamp=8,person_visible=True,person_direction='LEFT',
              transition='UNKNOWN',allowed_actions=['LOOK'],decision_direction='LEFT')]}
    assert evaluate_run(run,Engine.profile.to_dict(),scenario)['pass']
    assert json.dumps(run)==original
    frame['planner']['decision']['action']='SEARCH'
    assert not evaluate_run(run,Engine.profile.to_dict(),scenario)['pass']


@pytest.mark.parametrize('code',list(codecs.PLANNER_CLASS))
def test_planner_class_is_fixed_serialization_of_existing_actions(code):
    decision=codecs.decision(code,'planner_class')
    assert tuple(decision.model_dump(mode='json')[key] for key in ['action','direction','target','distance'])==codecs.PLANNER_CLASS[code]


def test_nonfinite_class_logits_cannot_be_false_valid_decision():
    class Engine:
        profile=PROFILES['class_both']
        def perceive(self,image): return 'B',{'class_logits_finite':False}
        def plan(self,*args): pytest.fail('Numerical failure must skip Planner')
    result=LatencyBrain(Engine()).decide(object())
    assert result['perception']['raw_response']=='B' and result['perception']['fallback']
    assert result['state_before']==result['state_after']


def test_linear_patch_matches_conv_without_weight_copy_and_restores_on_failure():
    import torch
    from types import SimpleNamespace
    from latency.patch_linear import linear_patch_forward,patch_backend
    class Patch(torch.nn.Module):
        def __init__(self):
            super().__init__(); self.in_channels=3; self.temporal_patch_size=2; self.patch_size=4
            self.proj=torch.nn.Conv3d(3,8,kernel_size=(2,4,4),stride=(2,4,4),bias=True)
        def forward(self,pixels): return self.proj(pixels.view(-1,3,2,4,4)).view(-1,8)
    patch=Patch(); pixels=torch.randn(7,96); original=patch(pixels); weight=patch.proj.weight
    torch.testing.assert_close(original,linear_patch_forward(patch,pixels),rtol=1e-5,atol=1e-6)
    model=SimpleNamespace(model=SimpleNamespace(visual=SimpleNamespace(patch_embed=patch)))
    with pytest.raises(RuntimeError):
        with patch_backend(model,True):
            torch.testing.assert_close(original,patch(pixels),rtol=1e-5,atol=1e-6)
            assert patch.proj.weight is weight
            raise RuntimeError('Simulated generation failure')
    torch.testing.assert_close(original,patch(pixels))
    patch.proj.stride=(1,4,4)
    with pytest.raises(ValueError): linear_patch_forward(patch,pixels)


def test_expanded_kv_matches_native_gqa_and_restores_global_on_failure():
    import torch
    import torch.nn.functional as F
    from latency.attention import expanded_kv
    from transformers.integrations import sdpa_attention
    original=sdpa_attention.use_gqa_in_sdpa
    q=torch.randn(1,4,3,8); k=torch.randn(1,2,3,8); v=torch.randn(1,2,3,8)
    gqa=F.scaled_dot_product_attention(q,k,v,enable_gqa=True)
    expanded=F.scaled_dot_product_attention(q,sdpa_attention.repeat_kv(k,2),sdpa_attention.repeat_kv(v,2))
    torch.testing.assert_close(gqa,expanded,rtol=1e-5,atol=1e-6)
    with pytest.raises(RuntimeError):
        with expanded_kv(True):
            assert not sdpa_attention.use_gqa_in_sdpa(None,k)
            raise RuntimeError('Simulated generation failure')
    assert sdpa_attention.use_gqa_in_sdpa is original


def test_attention_predicate_retains_no_tensor_references():
    import weakref
    import torch
    from latency.attention import expanded_kv
    from transformers.integrations import sdpa_attention
    with expanded_kv(True):
        for _ in range(4):
            tensor=torch.randn(1,2,3,8); reference=weakref.ref(tensor)
            assert not sdpa_attention.use_gqa_in_sdpa(None,tensor)
            del tensor
            assert reference() is None


def test_failed_gpu_stage_preserves_telemetry_and_state():
    class Engine:
        profile=PROFILES['json']
        def perceive(self,image): raise RuntimeError('GPU error')
        def failure_metrics(self,stage): return {'stage':stage,'peak_allocated_gib':4.1}
        def plan(self,*args): pytest.fail('GPU failure must skip Planner')
    result=LatencyBrain(Engine()).decide(object())
    assert result['perception']['metrics']['peak_allocated_gib']==4.1
    assert result['state_before']==result['state_after'] and result['planner']['fallback']
