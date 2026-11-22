import json
import pytest
from pydantic import ValidationError
from brain.split_schemas import CurrentObservation, PlannerDecision
from brain.state import State, transition
from brain.split_brain import SplitBrain, Perception, Planner, parse_flat
from brain.split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER, planner_user
from brain.schemas import PersonTransition
from test_video import video
from run_brain import evaluate_split_video
from config import Config

OBS=dict(person_visible=True,person_count=1,person_direction='LEFT',person_distance='NEAR',blocking_obstacle='NONE')
ABSENT={**OBS,'person_visible':False,'person_count':0,'person_direction':'UNKNOWN','person_distance':'UNKNOWN'}
DEC=dict(action='LOOK',direction='LEFT',target='PERSON',distance='NEAR')

@pytest.mark.parametrize('extra',['action','person_transition','scene_summary','memory','expected_action'])
def test_observation_only_disallows_other_contracts(extra):
    with pytest.raises(ValidationError): CurrentObservation(**OBS,**{extra:'anything'})

@pytest.mark.parametrize('key',list(OBS))
def test_observation_only_requires_fields(key):
    data=OBS.copy();del data[key]
    with pytest.raises(ValidationError): CurrentObservation(**data)

@pytest.mark.parametrize('key,value',[('person_visible','false'),('person_visible',1),('person_count',True),
 ('person_count',-1),('person_count',1.5),('person_direction','UP'),('person_distance','1m'),
 ('blocking_obstacle','CLEAR')])
def test_observation_only_strict_types(key,value):
    with pytest.raises(ValidationError): CurrentObservation(**{**OBS,key:value})

@pytest.mark.parametrize('change',[{'person_count':1},{'person_direction':'LEFT'},{'person_distance':'NEAR'}])
def test_absent_observation_must_be_consistent(change):
    with pytest.raises(ValidationError): CurrentObservation(**{**ABSENT,**change})

@pytest.mark.parametrize('before,current,expected',[(False,False,'STILL_ABSENT'),(False,True,'APPEARED'),
 (True,True,'STILL_VISIBLE'),(True,False,'DISAPPEARED'),(None,False,'UNKNOWN'),(None,True,'UNKNOWN')])
def test_state_machine(before,current,expected):
    assert transition(before,current)==expected


def test_absence_retains_direction_then_reappearance_resets_state():
    state=State();state.update(CurrentObservation(**OBS));state.update(CurrentObservation(**ABSENT))
    assert state.snapshot.transition=='DISAPPEARED'
    assert state.snapshot.last_seen_person_direction=='LEFT'
    assert state.snapshot.frames_since_person_seen==1
    state.update(CurrentObservation(**ABSENT));assert state.snapshot.transition=='STILL_ABSENT'
    state.update(CurrentObservation(**{**OBS,'person_direction':'RIGHT'}))
    assert state.snapshot.transition=='APPEARED' and state.snapshot.frames_since_person_seen==0
    assert state.snapshot.last_seen_person_direction=='RIGHT'


def test_visible_unknown_updates_current_direction_without_inventing_known_one():
    state=State();state.update(CurrentObservation(**OBS))
    state.update(CurrentObservation(**{**OBS,'person_direction':'UNKNOWN'}))
    assert state.snapshot.last_seen_person_direction=='UNKNOWN'


def test_state_rejects_unvalidated_observation():
    state=State()
    with pytest.raises(TypeError): state.update(OBS)
    assert state.snapshot.person_visible_last_frame is None


def test_planner_cannot_modify_observation_or_memory():
    obs=CurrentObservation(**OBS);state=State();snapshot=state.update(obs)
    class Engine:
        def plan(self,observation,memory):
            with pytest.raises(ValidationError): observation.person_visible=False
            with pytest.raises(ValidationError): memory.last_seen_person_direction='RIGHT'
            return json.dumps(DEC),{}
    result=Planner(Engine()).decide(obs,snapshot)
    assert not result.fallback and obs.person_visible and state.snapshot==snapshot


def test_perception_receives_only_current_image_and_planner_receives_no_image():
    token=object()
    class Engine:
        def perceive(self,image):
            assert image is token
            return json.dumps(OBS),{}
        def plan(self,observation,snapshot):
            assert isinstance(observation,CurrentObservation)
            assert snapshot.transition=='UNKNOWN'
            data=json.loads(planner_user(observation,snapshot))
            assert set(data)=={'current_observation','memory','transition'}
            assert 'expected_action' not in str(data) and 'timestamp' not in str(data)
            return json.dumps(DEC),{}
    result=SplitBrain(Engine()).decide(token)
    assert not result['perception']['fallback'] and not result['planner']['fallback']


def test_perception_prompt_has_no_history_actions_or_expectations():
    assert 'last_seen' not in PERCEPTION_SYSTEM and 'SEARCH' not in PERCEPTION_SYSTEM
    assert 'IMG_7121' not in PERCEPTION_SYSTEM and 'timestamp' not in PERCEPTION_SYSTEM
    assert 'memory' not in PERCEPTION_USER.lower() and 'previous' not in PERCEPTION_USER.lower()


def test_invalid_perception_never_updates_state_or_calls_planner():
    class Engine:
        def perceive(self,image): return json.dumps({**ABSENT,'person_direction':'LEFT'}),{}
        def plan(self,*args): raise AssertionError('Planner must not see invalid Perception')
    brain=SplitBrain(Engine());result=brain.decide(object())
    assert result['perception']['fallback']
    assert result['state_before']==result['state_after']
    assert result['planner']['decision']['action']=='WAIT'
    assert result['timing']['planner_wall_s']==0


def test_invalid_planner_does_not_corrupt_perception_memory():
    class Engine:
        def perceive(self,image): return json.dumps(OBS),{}
        def plan(self,*args): return '{broken}',{}
    brain=SplitBrain(Engine());result=brain.decide(object())
    assert not result['perception']['fallback'] and result['planner']['fallback']
    assert result['planner']['decision']['action']=='WAIT'
    assert brain.state.snapshot.person_visible_last_frame is True
    assert brain.state.snapshot.last_seen_person_direction=='LEFT'


def test_valid_planner_action_is_never_overwritten():
    class Engine:
        def perceive(self,image): return json.dumps(ABSENT),{}
        def plan(self,*args): return json.dumps({**DEC,'action':'SURPRISE'}),{}
    result=SplitBrain(Engine()).decide(object())
    assert result['planner']['decision']['action']=='SURPRISE'
    assert not result['planner']['fallback']


def test_optional_confidence_and_mock_boundary(capsys):
    from adapters.robot_mock import MockRobot
    d=PlannerDecision(**DEC)
    assert d.confidence is None
    MockRobot().execute(d)
    assert 'confidence=n/a' in capsys.readouterr().out

@pytest.mark.parametrize('extra',['speed','rotation_angle','scene_summary','observation','memory'])
def test_planner_rejects_motor_values_and_other_outputs(extra):
    with pytest.raises(ValidationError): PlannerDecision(**DEC,**{extra:1})

@pytest.mark.parametrize('raw',[json.dumps(OBS)+json.dumps(OBS),'['+json.dumps(OBS)+']',
 'explanation '+json.dumps(OBS),json.dumps({'observation':OBS}),'{"person_count":0,"person_count":1}'])
def test_flat_output_rejects_ambiguous_or_nested_payload(raw):
    with pytest.raises((ValueError,ValidationError)): parse_flat(raw,CurrentObservation)

@pytest.mark.parametrize('wrapper',['{}','```json\n{}\n```'])
def test_json_fence_only_preserves_all_values(wrapper):
    assert parse_flat(wrapper.format(json.dumps(OBS)),CurrentObservation)==CurrentObservation(**OBS)


def test_runner_uses_split_and_never_supplies_labels(video,tmp_path):
    class Engine:
        def __init__(self): self.count=0
        def perceive(self,image):
            self.count+=1
            return json.dumps(OBS if self.count==1 else ABSENT),{}
        def plan(self,observation,snapshot):
            # Deliberately wrong LOOK after absence: keep it, don't force SEARCH.
            return json.dumps(DEC),{}
    result=evaluate_split_video(video,[0,1],Config(),tmp_path/'split.json',engine=Engine())
    assert result['status']=='pass'
    assert result['frames'][1]['state_after']['transition']=='DISAPPEARED'
    assert result['frames'][1]['planner']['decision']['action']=='LOOK'


def test_video_cli_defaults_to_split(video,tmp_path,monkeypatch):
    import sys,run_video,run_brain
    calls=[]
    monkeypatch.setattr(sys,'argv',['run_video.py',str(video),'--timestamps','0,1','--report',str(tmp_path/'x.json')])
    monkeypatch.setattr(run_brain,'evaluate_split_video',lambda *args: calls.append(args) or {'status':'pass'})
    monkeypatch.setattr(run_video,'evaluate_video',lambda *args,**kwargs: pytest.fail('Legacy path used by default'))
    assert run_video.main()==0 and calls



@pytest.mark.parametrize('constant',['NaN','Infinity','-Infinity'])
def test_flat_json_rejects_non_json_numeric_constants(constant):
    from brain.split_brain import bare_json
    raw='{"person_count":'+constant+'}'
    assert not bare_json(raw)
    with pytest.raises(ValueError): parse_flat(raw,CurrentObservation)


def test_split_scenario_rejects_action_overwrite():
    # This fixture is evaluator input only, never a model prompt.
    from evaluate_split_scenario import evaluate
    from brain.state import State
    state=State();obs=CurrentObservation(**OBS);before=state.snapshot.model_dump(mode='json');state.update(obs)
    frame={'timestamp':0,'perception':{'raw_response':json.dumps(OBS),'observation':OBS,'fallback':False,'metrics':{'image_count':1}},
           'planner':{'raw_response':json.dumps(DEC),'decision':{**DEC,'action':'SEARCH'},'fallback':False,'metrics':{'image_count':0}},
           'state_before':before,'state_after':state.snapshot.model_dump(mode='json')}
    result=evaluate({'status':'pass','frames':[frame]}, {'timestamps':[0],'checks':[]})
    assert any('Raw/Decision mismatch' in e for e in result['integrity_errors'])



def test_video_cli_legacy_retains_phase2_route(video,tmp_path,monkeypatch):
    import sys,run_video,run_brain
    calls=[]
    monkeypatch.setattr(sys,'argv',['run_video.py',str(video),'--legacy','--timestamps','0','--report',str(tmp_path/'legacy.json')])
    monkeypatch.setattr(run_video,'evaluate_video',lambda *args,**kwargs: calls.append((args,kwargs)) or {'status':'pass'})
    monkeypatch.setattr(run_brain,'evaluate_split_video',lambda *args,**kwargs: pytest.fail('Legacy switched to split'))
    assert run_video.main()==0
    assert not calls[0][0][2].combined_output and calls[0][0][2].max_new_tokens==96


def test_video_cli_temporal_remains_experimental(video,tmp_path,monkeypatch):
    import sys,run_video
    calls=[]
    monkeypatch.setattr(sys,'argv',['run_video.py',str(video),'--temporal','--memory','--timestamps','0','--report',str(tmp_path/'pair.json')])
    monkeypatch.setattr(run_video,'evaluate_video',lambda *args,**kwargs: calls.append((args,kwargs)) or {'status':'pass'})
    assert run_video.main()==0
    assert calls[0][0][2].combined_output and calls[0][0][2].max_new_tokens==224
    assert calls[0][1]['temporal'] and calls[0][1]['use_memory']
