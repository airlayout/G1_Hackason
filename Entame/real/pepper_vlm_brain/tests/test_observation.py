import json
import pytest
from pydantic import ValidationError
from brain.schemas import Observation, PersonTransition, Decision
from brain.decision import Brain, parse_decision
from brain.memory import Memory
from brain.prompt import observation_user_prompt, OBSERVATION_SYSTEM_PROMPT
from run_video import evaluate_video
from config import Config
from test_video import video
from test_scenario import fixture_report, SCENARIO
from evaluate_scenario import evaluate

OBS = dict(person_visible=True, person_count=1, person_direction='LEFT',
           person_distance='NEAR', blocking_obstacle='NONE', person_transition='UNKNOWN')
DEC = dict(action='LOOK', direction='RIGHT', target='PERSON', distance='MID', confidence=0.99)

@pytest.mark.parametrize('key,value', [('person_visible',1),('person_visible','false'),
 ('person_count',True),('person_count',-1),('person_count',1.5),('person_direction','UP'),
 ('person_distance','2m'),('blocking_obstacle','CLEAR'),('person_transition','GONE'),
 ('scene_summary',3),('extra',1)])
def test_observation_rejects_invalid_fields(key,value):
    with pytest.raises(ValidationError): Observation.model_validate({**OBS,key:value})

@pytest.mark.parametrize('key',list(OBS))
def test_observation_requires_every_field(key):
    payload=OBS.copy();del payload[key]
    assert parse_decision(json.dumps({'observation':payload,'decision':DEC}),combined=True).fallback

@pytest.mark.parametrize('transition',list(PersonTransition))
def test_transition_enums(transition):
    visible=transition not in {PersonTransition.DISAPPEARED,PersonTransition.STILL_ABSENT}
    data={**OBS,'person_transition':transition,'person_visible':visible,'person_count':int(visible)}
    if not visible: data.update(person_direction='UNKNOWN',person_distance='UNKNOWN')
    assert Observation.model_validate(data).person_transition==transition

@pytest.mark.parametrize('change',[{'person_count':0},{'person_visible':False},
 {'person_transition':'DISAPPEARED'}])
def test_inconsistent_observation_is_fallback(change):
    result=parse_decision(json.dumps({'observation':{**OBS,**change},'decision':DEC}),combined=True)
    assert result.fallback and result.observation is None and result.decision.action=='WAIT'


def test_decision_only_cannot_change_perceptual_memory():
    memory=Memory();memory.update(Decision(**DEC),Observation(**OBS))
    before=memory.context()
    memory.update(Decision(**{**DEC,'direction':'CENTER','target':'NONE','action':'WAIT'}))
    after=memory.context()
    assert {k:v for k,v in before.items() if k!='last_action'}=={k:v for k,v in after.items() if k!='last_action'}
    assert after['last_action']=='WAIT'


def test_observation_overrules_target_without_overwriting_action():
    memory=Memory();memory.update(Decision(**DEC),Observation(**OBS))
    absent=Observation(**{**OBS,'person_visible':False,'person_count':0,'person_direction':'UNKNOWN',
                          'person_distance':'UNKNOWN','person_transition':'DISAPPEARED'})
    memory.update(Decision(**DEC),absent)
    assert not memory.person_visible_last_frame and memory.last_seen_person_direction=='LEFT'
    assert memory.frames_since_person_seen==1 and memory.last_action=='LOOK'


def test_confidence_and_summary_never_control_memory():
    memory=Memory()
    memory.update(Decision(**{**DEC,'confidence':0}),Observation(**{**OBS,'scene_summary':'Nobody visible'}))
    assert memory.person_visible_last_frame and memory.last_seen_person_direction=='LEFT'


def test_first_frame_without_previous_is_explicit_unknown():
    class Engine:
        def generate(self,image,memory_context=None,previous_image=None):
            assert previous_image is None
            return json.dumps({'observation':OBS,'decision':DEC}),{}
    assert not Brain(Engine(),combined=True).decide(object()).fallback
    assert 'No previous image; transition UNKNOWN' in observation_user_prompt()


def test_first_frame_invented_transition_rejected():
    class Engine:
        def generate(self,*args,**kwargs):
            return json.dumps({'observation':{**OBS,'person_transition':'APPEARED'},'decision':DEC}),{}
    assert Brain(Engine(),combined=True).decide(object()).fallback


def test_runner_pairs_previous_selected_frame_and_does_not_overwrite(video,tmp_path):
    class Engine:
        def __init__(self): self.inputs=[]
        def generate(self,image,memory_context=None,previous_image=None):
            self.inputs.append((image.tobytes(),None if previous_image is None else previous_image.tobytes(),memory_context))
            obs={**OBS,'person_transition':'UNKNOWN' if previous_image is None else 'STILL_VISIBLE'}
            # LEFT observation, RIGHT decision: preserve both, not expected-label repair.
            return json.dumps({'observation':obs,'decision':DEC}),{}
    engine=Engine()
    report=evaluate_video(video,[0,2],Config(combined_output=True),tmp_path/'pair.json',engine=engine,use_memory=True,temporal=True)
    assert report['status']=='pass'
    assert engine.inputs[0][1] is None and engine.inputs[1][1]==engine.inputs[0][0]
    assert set(engine.inputs[1][2])==set(Memory().context())
    assert engine.inputs[1][2]['last_seen_person_direction']=='LEFT'
    assert report['frames'][1]['decision']==DEC
    assert report['frames'][1]['previous_selected_timestamp']==0


def test_evaluator_rejects_observation_tampering():
    report=fixture_report();report['frames'][1]['observation']['person_visible']=True
    assert evaluate(report,SCENARIO)['integrity_errors']


def test_evaluator_requires_observation_even_with_correct_action():
    report=fixture_report();report['combined_output']=False
    for frame in report['frames']: frame['raw_response']=json.dumps(frame['decision']);frame.pop('observation')
    assert not evaluate(report,{**SCENARIO,'require_observation':True})['pass']


def test_prompt_contains_no_scenario_expectations():
    prompt=OBSERVATION_SYSTEM_PROMPT+observation_user_prompt(Memory().context(),True)
    assert 'timestamp' not in prompt and 'IMG_7121' not in prompt and 'expected' not in prompt
    assert 'IMAGE 1 = previous' in prompt and 'IMAGE 2 = current' in prompt


def test_temporal_legacy_configuration_fails_before_inference(video,tmp_path):
    result=evaluate_video(video,[0],Config(),tmp_path/'bad.json',engine=object(),temporal=True)
    assert result['status']=='failed' and not result['frames']



def test_single_image_prompt_restricts_transition_without_touching_action():
    from brain.prompt import observation_system_prompt
    single=observation_system_prompt(False)
    assert 'person_transition MUST be UNKNOWN' in single
    assert 'person_transition UNKNOWN (the only permitted value' in single
    assert 'LOOK/APPROACH/SEARCH' in single
    assert 'absent->visible APPEARED' in observation_system_prompt(True)


def test_malformed_combined_decision_is_not_repaired():
    result=parse_decision(json.dumps({'observation':OBS,'decision':'LOOK'}),combined=True)
    assert result.fallback and result.observation is None and result.decision.action=='WAIT'


def test_invalid_perception_keeps_memory_and_previous_image_advances(video,tmp_path):
    class Engine:
        def __init__(self): self.images=[];self.contexts=[]
        def generate(self,image,memory_context=None,previous_image=None):
            self.images.append((image.tobytes(),None if previous_image is None else previous_image.tobytes()))
            self.contexts.append(memory_context)
            if len(self.images)==2: return '{broken}',{}
            return json.dumps({'observation':{**OBS,'person_transition':'UNKNOWN' if previous_image is None else 'STILL_VISIBLE'},'decision':DEC}),{}
    engine=Engine()
    report=evaluate_video(video,[0,1,2],Config(combined_output=True),tmp_path/'invalid.json',engine=engine,use_memory=True,temporal=True)
    assert engine.images[2][1]==engine.images[1][0]
    assert engine.contexts[2]['last_seen_person_direction']=='LEFT'
    assert engine.contexts[2]['person_visible_last_frame']
    assert report['frames'][1]['fallback'] and report['frames'][1]['decision']['action']=='WAIT'


def test_extra_missing_frame_schedule_rejected():
    report=fixture_report();report['frames'].insert(0,report['frames'][0].copy())
    assert 'Evaluated frame schedule mismatch' in evaluate(report,SCENARIO)['integrity_errors']



def test_fallback_cannot_accept_a_model_action():
    report=fixture_report();frame=report['frames'][1]
    frame.update(fallback=True,error='invalid perception',raw_response='{broken}')
    result=evaluate(report,SCENARIO)
    assert any('Fallback Action mismatch' in e for e in result['integrity_errors'])


def test_evaluator_checks_memory_against_observation_not_decision():
    report=fixture_report();report['frames'][0]['memory_after']['last_seen_person_direction']='CENTER'
    assert any('Observation/Memory mismatch' in e for e in evaluate(report,SCENARIO)['integrity_errors'])
