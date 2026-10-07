import json
import pytest
from pydantic import ValidationError
from adapters.robot_mock import MockRobot
from brain.split_brain import Planner, SplitBrain
from brain.split_schemas import CurrentObservation, PlannerDecision
from brain.split_prompt import planner_user
from brain.state import State
from config import Config
from run_brain import evaluate_split_video
from run_planner_replay import replay_planner
from test_video import video

OBS = dict(person_visible=True, person_count=1, person_direction='LEFT', person_distance='NEAR', blocking_obstacle='NONE')
DEC = dict(action='LOOK', direction='LEFT', target='PERSON', distance='NEAR')


@pytest.mark.parametrize('raw', [json.dumps(DEC), '{broken}', json.dumps({**DEC, 'action':'MOVE'}),
                               json.dumps({**DEC, 'direction':'UP'}), json.dumps({**DEC, 'target':'HUMAN'}),
                               json.dumps({**DEC, 'distance':'1m'}), '{"action": LOOK}',
                               json.dumps({'decision':DEC})])
def test_planner_validation_and_failure_preserve_truth(raw):
    observation = CurrentObservation(**OBS); state = State(); snapshot = state.update(observation)
    class Engine:
        def plan(self, observation_arg, snapshot_arg):
            assert observation_arg is observation and snapshot_arg is snapshot
            return raw, {}
    result = Planner(Engine()).decide(observation, snapshot)
    assert result.fallback == (raw != json.dumps(DEC))
    assert result.decision.action == ('WAIT' if result.fallback else 'LOOK')
    assert state.snapshot is snapshot and observation.model_dump(mode='json') == OBS


def test_shared_load_is_idempotent(monkeypatch):
    import brain.split_vlm as module
    calls = []
    class Model:
        def get_memory_footprint(self): return 1
    def load(engine):
        calls.append(engine); engine.model=Model(); engine.processor=object()
    monkeypatch.setattr(module.QwenVLM, 'load', load)
    monkeypatch.setattr(module.torch.cuda, 'is_available', lambda: False)
    monkeypatch.setattr(module.torch.cuda, 'max_memory_allocated', lambda: 0)
    monkeypatch.setattr(module.torch.cuda, 'max_memory_reserved', lambda: 0)
    engine=module.SplitQwenVLM(Config()); brain=SplitBrain(engine)
    engine.load(); model,processor=engine.model,engine.processor
    engine.load(); brain.perception.engine.load(); brain.planner.engine.load()
    assert len(calls)==1 and engine.model_load_count==1
    assert engine.model is model and engine.processor is processor
    assert brain.perception.engine is brain.planner.engine


def test_planner_generation_messages_contain_only_text_and_truth(monkeypatch):
    from brain.split_vlm import SplitQwenVLM
    engine=SplitQwenVLM(Config()); obs=CurrentObservation(**OBS); snapshot=State().update(obs)
    def generate(messages,stage,prompt):
        assert stage=='planner'
        assert all(c['type']=='text' for m in messages for c in m['content'])
        assert messages[-1]['content'][0]['text']==planner_user(obs,snapshot)
        assert not any(s in prompt for s in ['timestamp','expected_action','IMG_7121'])
        return json.dumps(DEC),{}
    monkeypatch.setattr(engine,'_generate',generate)
    assert engine.plan(obs,snapshot)[0]==json.dumps(DEC)


@pytest.mark.parametrize('valid',[True,False])
def test_e2e_order_and_robot_validated_boundary(video,tmp_path,monkeypatch,valid):
    events=[]
    original_update=State.update; original_execute=MockRobot.execute
    def update(state,obs):
        events.append('State'); return original_update(state,obs)
    def execute(robot,decision):
        assert isinstance(decision,PlannerDecision)
        assert decision.action==('LOOK' if valid else 'WAIT')
        events.append('Robot'); return original_execute(robot,decision)
    monkeypatch.setattr(State,'update',update); monkeypatch.setattr(MockRobot,'execute',execute)
    class Engine:
        def perceive(self,image): events.append('Perception'); return json.dumps(OBS),{}
        def plan(self,obs,state):
            events.append('Planner'); assert state.last_seen_person_direction=='LEFT'
            return (json.dumps(DEC) if valid else '{broken}'),{}
    report=evaluate_split_video(video,[0],Config(),tmp_path/'report.json',engine=Engine())
    assert events==['Perception','State','Planner','Robot']
    assert report['frames'][0]['mock_robot_output'].startswith('[MOCK ROBOT]')
    assert report['frames'][0]['state_after']['last_seen_person_direction']=='LEFT'


def test_robot_rejects_unvalidated_payload():
    with pytest.raises(TypeError): MockRobot().execute(DEC)


def test_saved_replay_no_images_and_preserves_valid_wrong_action(tmp_path):
    class Engine:
        def perceive(self,image): pytest.fail('Replay must not perform Perception')
        def plan(self,obs,state):
            assert isinstance(obs,CurrentObservation)
            return json.dumps({**DEC,'action':'SURPRISE'}),{}
    source={'frames':[{'timestamp':8,'raw_response':json.dumps(OBS),'observation':OBS,'fallback':False}]}
    result=replay_planner(source,[8],Config(),tmp_path/'report.json',Engine())
    assert result['status']=='pass' and result['frames'][0]['planner']['decision']['action']=='SURPRISE'
    source['frames'][0]['observation']={**OBS,'person_direction':'RIGHT'}
    result=replay_planner(source,[8],Config(),tmp_path/'bad.json',Engine())
    assert result['status']=='failed' and not result['frames']


@pytest.mark.parametrize('change',['raw_overwrite','reuse','mock','presence'])
def test_phase33_evaluator_rejects_false_success(tmp_path,change):
    from evaluate_phase33 import evaluate_phase33
    class Engine:
        def plan(self,obs,state):
            return json.dumps(DEC), {'image_count':0,'model_instance_id':1,'processor_instance_id':2,'model_load_count':1}
    source={'frames':[{'timestamp':8,'raw_response':json.dumps(OBS),'observation':OBS,'fallback':False}]}
    report=replay_planner(source,[8],Config(),tmp_path/'report.json',Engine())
    scenario={'timestamps':[8],'checks':[{'timestamp':8,'person_visible':True,'person_direction':'LEFT',
        'transition':'UNKNOWN','allowed_actions':['LOOK'],'decision_direction':'LEFT'}]}
    assert evaluate_phase33(report,scenario)['pass']
    frame=report['frames'][0]
    if change=='raw_overwrite': frame['planner']['decision']['action']='SEARCH'
    elif change=='reuse': frame['planner']['metrics']['model_load_count']=2
    elif change=='mock': frame['mock_robot_output']='invented'
    else: frame['perception']['observation']['person_visible']=False
    assert not evaluate_phase33(report,scenario)['pass']
