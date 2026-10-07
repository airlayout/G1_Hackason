import copy,hashlib,json
from pathlib import Path
import pytest
from evaluate_perception import evaluate,compare
from brain.split_prompt import PERCEPTION_SYSTEM,PERCEPTION_USER
from brain.split_schemas import PlannerDecision
from config import MODEL_2B,MODEL_4B

ANNOTATIONS={'labels':[{'timestamp':t,'person_visible':t in (3,7,8,12,13),
 'person_direction':('CENTER' if t==3 else 'LEFT') if t in (3,7,8,12,13) else 'UNKNOWN',
 'subset':'additional' if t in (3,7,9,9.5,10.25,13) else 'transition'} for t in (3,7,8,9,9.5,10,10.25,12,13)]}


def fixture_report(model=MODEL_4B):
    frames=[]
    for label in ANNOTATIONS['labels']:
        visible=label['person_visible']
        obs={'person_visible':visible,'person_count':int(visible),'person_direction':label['person_direction'],
             'person_distance':'NEAR' if visible else 'UNKNOWN','blocking_obstacle':'NONE'}
        frames.append({'timestamp':label['timestamp'],'raw_response':json.dumps(obs),'observation':obs,'fallback':False,
             'metrics':{'stage':'perception','image_count':1,'prompt_sha256':hashlib.sha256((PERCEPTION_SYSTEM+PERCEPTION_USER).encode()).hexdigest(),
                        'inference_latency_s':1,'input_tokens':300,'output_tokens':40,'image_tokens':84,
                        'peak_allocated_gib':3,'peak_reserved_gib':4}})
    return {'status':'pass','system_prompt':PERCEPTION_SYSTEM,'user_prompt':PERCEPTION_USER,'timestamps':[x['timestamp'] for x in ANNOTATIONS['labels']],
            'config':{'model_name':model,'quantize_4bit':model==MODEL_4B},'frames':frames,
            'model_load':{'loaded_in_4bit':model==MODEL_4B,'nf4_linear_layers':100 if model==MODEL_4B else 0,
                          'gpu_used':True,'model_load_peak_allocated_gib':3,'model_load_peak_reserved_gib':4}}


def test_perception_evaluator_accepts_unmodified_complete_report():
    assert evaluate(fixture_report(),ANNOTATIONS)['pass']


def test_perception_evaluator_rejects_observation_override():
    report=fixture_report();report['frames'][3]['observation']={**report['frames'][3]['observation'],'person_visible':True}
    assert evaluate(report,ANNOTATIONS)['integrity_errors']


def test_planner_gate_rejects_four_b_false_positives():
    two=fixture_report(MODEL_2B);four=fixture_report()
    for f in four['frames']:
        if not f['observation']['person_visible']:
            obs={'person_visible':True,'person_count':1,'person_direction':'LEFT','person_distance':'NEAR','blocking_obstacle':'NONE'}
            f.update(observation=obs,raw_response=json.dumps(obs))
    assert not compare(two,four,ANNOTATIONS)['planner_gate']['pass']


def test_planner_gate_requires_same_prompt_for_both_models():
    two=fixture_report(MODEL_2B);two['system_prompt']+='extra'
    assert not compare(two,fixture_report(),ANNOTATIONS)['planner_gate']['pass']


def test_invalid_absence_is_not_labeled_false_positive_or_repaired():
    two=fixture_report(MODEL_2B)
    for f in two['frames']:
        if not f['observation']['person_visible']:
            f.update(raw_response='{"person_visible":false,"person_direction":UNKNOWN}',observation=None,fallback=True)
    evaluation=evaluate(two,ANNOTATIONS)
    assert evaluation['additional_false_positives']==0
    assert evaluation['additional_invalid_absences']==3
    assert evaluation['additional_absence_correct']==0
    assert compare(two,fixture_report(),ANNOTATIONS)['planner_gate']['pass']


@pytest.mark.parametrize('confidence',[True,'0.9',-0.1,1.1,float('nan'),float('inf')])
def test_planner_optional_confidence_still_validates_numbers(confidence):
    with pytest.raises(ValueError): PlannerDecision(action='LOOK',direction='LEFT',target='PERSON',distance='NEAR',confidence=confidence)



def test_equal_absence_accuracy_does_not_authorize_planner_experiment():
    result=compare(fixture_report(MODEL_2B),fixture_report(),ANNOTATIONS)
    assert result['2b']['additional_absence_correct']==3
    assert result['4b']['additional_absence_correct']==3
    assert not result['planner_gate']['pass']
    assert not result['planner_gate']['usable_absence_improved']


def test_state_replay_rejects_invalid_or_overwritten_observation():
    from replay_state import replay
    report=fixture_report(MODEL_2B)
    report['frames'][2]['raw_response']='{"person_visible":true,"person_direction":LEFT}'
    result=replay(report,[8,10,12])
    assert result['frames'][0]['observation'] is None
    assert result['frames'][0]['state_after']['person_visible_last_frame'] is None
    assert result['frames'][1]['state_after']['transition']=='UNKNOWN'
    assert not result['planner_executed']
    assert all('decision' not in f for f in result['frames'])
