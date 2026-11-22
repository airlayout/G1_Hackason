from threading import Thread,Event
from time import perf_counter
import pytest
from hybrid.core import Frame,LatestFrameSlot,GeometryStore,EventLatch,immediate_tracking,bind_latest,RecordingRobot
from hybrid.workers import person_detection
from brain.split_schemas import PlannerDecision,CurrentObservation


def result(gen=1,side='LEFT',visible=True,capture=10.,finish=None):
    finish=capture+.02 if finish is None else finish
    return {'generation':gen,'source_timestamp':gen/30,'capture_timestamp':capture,'processing_start':capture,
        'processing_finish':finish,'observation':{'person_visible':visible,'person_count':int(visible),
        'person_direction':side if visible else 'UNKNOWN','person_distance':'UNKNOWN','blocking_obstacle':'UNKNOWN'}}


def test_slot_has_no_frame_queue_and_returns_latest_only():
    slot=LatestFrameSlot()
    for i in range(100): assert slot.publish(Frame(i,i/30,i/30,i/30,object()))
    assert slot.latest().generation==99 and slot.published==100 and slot.overwritten==99
    assert slot.latest(99) is None
    assert not slot.publish(Frame(98,3.3,3.3,3.3,object()))
    assert not hasattr(slot,'queue')


@pytest.mark.parametrize('capture,generation,now',[(9.,2,10.02),(10.01,1,10.02),(10.01,0,10.02),(10.01,2,11.)])
def test_old_generation_time_or_stale_result_rejected(capture,generation,now):
    store=GeometryStore(); store.accept_fast(result(),10.03); before=store.state.snapshot
    accepted,_=store.accept_fast(result(generation,'RIGHT',capture=capture),now)
    assert not accepted and store.state.snapshot is before and store.observation.person_direction=='LEFT'


def test_fast_only_determines_transitions_and_last_seen():
    store=GeometryStore(); store.accept_fast(result(),10.03)
    before=store.state.snapshot
    semantic=result(0,'RIGHT',capture=9.); semantic['observation']['person_distance']='NEAR'
    store.accept_semantic(semantic,store.episode)
    assert store.state.snapshot is before and store.observation.person_direction=='LEFT'
    store.accept_fast(result(2,visible=False,capture=10.1),10.12)
    assert store.state.snapshot.transition=='DISAPPEARED' and store.state.snapshot.last_seen_person_direction=='LEFT'
    store.accept_fast(result(3,'RIGHT',capture=10.2),10.22)
    assert store.state.snapshot.transition=='APPEARED' and store.state.snapshot.last_seen_person_direction=='RIGHT'


def test_semantic_cache_respects_episode_and_ttl_without_geometry_overwrite():
    store=GeometryStore(semantic_ttl=1.)
    store.accept_fast(result(),10.02)
    semantic=result(); semantic['observation']['person_distance']='NEAR'; semantic['observation']['blocking_obstacle']='NONE'
    store.accept_semantic(semantic,store.episode)
    assert store.combined(10.1).person_distance=='NEAR'
    assert store.combined(11.1).person_distance=='UNKNOWN'
    store.accept_fast(result(2,visible=False,capture=10.2),10.22)
    store.accept_fast(result(3,capture=10.3),10.33)
    assert store.combined(10.4).person_distance=='UNKNOWN'


@pytest.mark.parametrize('side',['LEFT','CENTER','RIGHT'])
def test_only_look_is_generated_by_immediate_layer(side):
    store=GeometryStore(); store.accept_fast(result(side=side),10.03)
    decision=immediate_tracking(store,10.03)
    assert decision.action=='LOOK' and decision.direction==side and decision.target=='PERSON'
    assert immediate_tracking(store,11.) is None
    store.accept_fast(result(2,visible=False,capture=11.1),11.12)
    assert immediate_tracking(store,11.12) is None  # Never synthesize SEARCH.


@pytest.mark.parametrize('action',['LOOK','FOUND'])
def test_binding_uses_latest_direction_and_keeps_original(action):
    store=GeometryStore(); store.accept_fast(result(side='RIGHT'),10.03)
    original=PlannerDecision(action=action,direction='LEFT',target='PERSON',distance='NEAR')
    resolved=bind_latest(original,store,10.03)
    assert resolved.direction=='RIGHT' and original.direction=='LEFT' and resolved.action==original.action
    assert resolved.distance==original.distance


def test_search_memory_binding_preserved_no_rule_replacement():
    store=GeometryStore(); store.accept_fast(result(side='RIGHT'),10.03)
    search=PlannerDecision(action='SEARCH',direction='LEFT',target='PERSON',distance='UNKNOWN')
    assert bind_latest(search,store,10.03) is search
    robot=RecordingRobot(); robot.execute(search)
    assert 'SEARCH LEFT' in robot.last_output
    with pytest.raises(TypeError): robot.execute(search.model_dump())


def test_event_latch_debounces_and_coalesces_without_queue():
    store=GeometryStore(); store.accept_fast(result(),10.03); latch=EventLatch()
    args=(store.observation,store.state.snapshot,store.result,store.episode)
    assert latch.offer('APPEARED',*args,10.03)
    assert not latch.offer('APPEARED',*args,10.04)
    assert latch.offer('DISAPPEARED',*args,10.1)
    assert latch.coalesced==1 and latch.take()['kind']=='DISAPPEARED' and latch.take() is None
    assert not hasattr(latch,'queue')


def test_still_visible_does_not_schedule_planner_every_frame():
    store=GeometryStore(); latch=EventLatch(); calls=0
    for i in range(50):
        store.accept_fast(result(i+1,capture=10+i/30),10+i/30+.03)
        if store.state.snapshot.transition in ['UNKNOWN','APPEARED','DISAPPEARED']:
            calls+=int(latch.offer('INITIAL',store.observation,store.state.snapshot,store.result,store.episode,10+i/30+.03))
    assert calls==1


def test_mock_fast_updates_continue_while_slow_worker_is_busy():
    busy=Event(); stop=Event(); busy.set(); slot=LatestFrameSlot()
    # Fast producer never acquires a slow-model lock or awaits slow completion.
    def produce():
        for i in range(200): slot.publish(Frame(i,i/30,i/30,i/30,None))
        stop.set()
    thread=Thread(target=produce); thread.start(); assert stop.wait(1); thread.join()
    assert busy.is_set() and slot.latest().generation==199


@pytest.mark.parametrize('center,side',[(.3999,'LEFT'),(.4,'CENTER'),(.6,'CENTER'),(.6001,'RIGHT')])
def test_direction_boundaries_fixed_inclusive(center,side):
    x=center*100
    output=person_detection([[x-1,0,x+1,10,.9,0]],100)
    assert output['observation']['person_direction']==side


def test_multiple_people_generic_target_policy_and_count():
    output=person_detection([[0,0,20,20,.8,0],[60,0,100,40,.6,0],[0,0,100,100,.99,46]],100)
    assert output['observation']['person_count']==2 and output['bbox_center_x']==.8
    assert output['observation']['person_direction']=='RIGHT'


def test_gt_paths_and_expected_actions_absent_from_runtime():
    from pathlib import Path
    paths=list(Path('hybrid').glob('*.py'))+[Path('run_hybrid_video.py'),Path('benchmark_fast_sources.py')]
    text='\n'.join(p.read_text() for p in paths)
    for forbidden in ['phase36_events.json','phase32_perception.json','expected_action','ground_truth','IMG_7121_phase33']:
        assert forbidden not in text


@pytest.mark.parametrize('field,value',[('capture_timestamp',float('nan')),('processing_start',9.),
    ('processing_finish',9.9),('generation',True),('generation',-1),('capture_timestamp',11.)])
def test_invalid_result_metadata_never_updates_state(field,value):
    store=GeometryStore(); data=result(); data[field]=value
    accepted,_=store.accept_fast(data,10.03)
    assert not accepted and store.observation is None


def test_event_snapshot_does_not_rewind_live_state_or_force_action():
    store=GeometryStore(); store.accept_fast(result(),10.03)
    store.accept_fast(result(2,visible=False,capture=10.1),10.12)
    latch=EventLatch(); latch.offer('DISAPPEARED',store.observation,store.state.snapshot,store.result,store.episode,10.12)
    store.accept_fast(result(3,visible=False,capture=10.2),10.22)
    event=latch.take()
    assert event['snapshot']['frames_since_person_seen']==1 and event['snapshot']['transition']=='DISAPPEARED'
    assert store.state.snapshot.frames_since_person_seen==2 and store.state.snapshot.transition=='STILL_ABSENT'
    wrong_model=PlannerDecision(action='SEARCH',direction='RIGHT',target='PERSON',distance='UNKNOWN')
    assert bind_latest(wrong_model,store,10.22) is wrong_model


def test_worker_one_inflight_and_cancel_event_without_frame_queue():
    from hybrid.process import WorkerProcess
    from threading import Event
    from types import SimpleNamespace
    worker=WorkerProcess.__new__(WorkerProcess); worker.busy=True; worker.cancel=Event()
    assert worker.cancel_obsolete() and worker.cancel.is_set()
    with pytest.raises(RuntimeError): worker.submit({'kind':'planner'})
    worker.busy=False
    assert not worker.cancel_obsolete()


def test_global_confidence_variant_is_not_timestamp_or_action_rule():
    row=[[0,0,10,10,.15,0]]
    assert not person_detection(row,100,.25)['observation']['person_visible']
    assert person_detection(row,100,.1)['observation']['person_visible']
    assert person_detection([[0,0,10,10,.9,.1]],100,.1)['observation']['person_count']==0


def test_real_gpu_fast_updates_overlap_slow_model_and_rope_cache_is_restored():
    import json
    from pathlib import Path
    path=Path('reports/phase36/realtime_cooperative/fast_vlm_planner_cooperative_0.json')
    if not path.exists(): pytest.skip('Cooperative GPU benchmark not present')
    report=json.loads(path.read_text()); pushes=[r for r in report['fast_results'] if r.get('cooperative_fast_update')]
    assert pushes and all(r['slow_rope_cache_restored'] for r in pushes)
    assert report['fast_worker']['pid']==report['slow_worker']['pid']
    metrics=[r['metrics'] for r in pushes+report['slow_results']]
    assert len({m['model_instance_id'] for m in metrics})==1 and all(m['model_load_count']==1 for m in metrics)
    for row in pushes:
        assert any(s['processing_start']<=row['processing_start']<=s['processing_finish'] for s in report['slow_results'])


def test_cancelled_raw_output_never_reaches_robot():
    import json
    from pathlib import Path
    report=json.loads(Path('reports/phase36/realtime_preempt/hybrid_0.json').read_text())
    cancelled=[r for r in report['slow_results'] if r.get('cancelled')]
    assert cancelled
    for row in cancelled:
        assert not any(c.get('raw_response')==row['raw_response'] for c in report['commands'] if c['channel']=='vlm_semantic')
    for row in report['slow_results']:
        if row['kind']=='planner' and not row.get('cancelled'):
            assert row['metrics']['image_count']==0
            assert set(row['planner_input'])=={'current_observation','memory','transition'}


def test_new_episode_survives_debounce_and_pending_events_settle_without_queue():
    store=GeometryStore(); store.accept_fast(result(),10.03); latch=EventLatch()
    args=(store.observation,store.state.snapshot,store.result)
    assert latch.offer('APPEARED',*args,1,10.03)
    assert latch.take(10.08) is None and latch.pending is not None
    # Same event kind, new visibility episode: don't discard the latest event.
    assert latch.offer('APPEARED',*args,3,10.10)
    assert latch.take(10.19) is None
    assert latch.take(10.21)['episode']==3
    assert latch.pending is None and latch.coalesced==1


def test_semantic_refresh_cannot_erase_pending_disappearance_event():
    store=GeometryStore(); store.accept_fast(result(1,visible=False),10.03); latch=EventLatch()
    args=(store.observation,store.state.snapshot,store.result,store.episode)
    assert latch.offer('DISAPPEARED',*args,10.03)
    assert not latch.offer('SEMANTIC_CHANGED',*args,10.05)
    assert not latch.offer('EXPIRED',*args,10.06)
    assert latch.take(10.14)['kind']=='DISAPPEARED'


def test_mock_geometry_and_commands_update_while_slow_completion_is_unavailable():
    busy=Event(); busy.set(); store=GeometryStore(); commands=[]
    def fast_worker():
        for i in range(100):
            now=10+i/30+.03
            store.accept_fast(result(i+1,'RIGHT' if i%2 else 'LEFT',capture=10+i/30),now)
            commands.append(immediate_tracking(store,now))
    thread=Thread(target=fast_worker); thread.start(); thread.join(timeout=1)
    assert not thread.is_alive() and busy.is_set() and store.generation==100
    assert len(commands)==100 and all(c.action=='LOOK' for c in commands)
    assert commands[-1].direction=='RIGHT'
