"""Paced video, latest-frame wins, no GT inputs; stress cases distinct from event runtime."""
import argparse
import json
from pathlib import Path
from time import perf_counter,sleep
from hybrid.core import LatestFrameSlot,GeometryStore,EventLatch,RecordingRobot,bind_latest,immediate_tracking
from hybrid.video import VideoProducer
from hybrid.process import WorkerProcess
from brain.split_schemas import PlannerDecision,CurrentObservation
from brain.state import State
from run_video import write_report

CASES={
 'yolo_alone':('yolo','none'), 'fast_vlm_alone':('qwen','none'),
 'planner_alone':('none','planner'), 'yolo_planner':('yolo','planner'),
 'fast_vlm_planner':('qwen','planner'), 'yolo_semantic':('yolo','semantic'),
 'fast_vlm_semantic':('qwen','semantic'), 'hybrid':('yolo','events'),
 'fast_vlm_planner_cooperative':('qwen','planner'), 'fast_vlm_semantic_cooperative':('qwen','semantic')}


def frame_request(frame,kind):
    return {'kind':kind,'generation':frame.generation,'source_timestamp':frame.source_timestamp,
            'capture_timestamp':frame.capture_timestamp,'bgr':frame.bgr}


def run_case(video,case,model_path,output,device='0',confidence=.25):
    source,load=CASES[case]; fast=slow=None; workers=[]
    cooperative=case.endswith('_cooperative')
    report={'case':case,'status':'initializing','fast_source':source,'slow_load':load,
            'yolo_confidence':confidence,
            'fast_results':[],'slow_results':[],'events':[],'commands':[],'resources':[],
            'queue_capacity':0,'latest_frame_slot_capacity':1,'max_in_flight_per_worker':1,
            'shared_qwen_negative_control':source=='qwen' and load!='none' and not cooperative,
            'cooperative_single_weight':cooperative,
            'cancellation_enabled':load=='events',
            'note':'Qwen negative control uses one safe serialized GPU owner; YOLO and Qwen use independent processes.'}
    write_report(output,report)
    try:
        if source!='none':
            fast=WorkerProcess(source,model_path,device,cooperative,confidence); workers.append(fast); report['fast_worker']=fast.ready()
        if load!='none':
            slow=fast if source=='qwen' else WorkerProcess('qwen')
            if slow is not fast: workers.append(slow); report['slow_worker']=slow.ready()
            else: report['slow_worker']=report['fast_worker']
        import psutil
        cpus={w.process.pid:psutil.Process(w.process.pid) for w in workers}
        cpus[psutil.Process().pid]=psutil.Process()
        for p in cpus.values(): p.cpu_percent()
        try:
            import pynvml
            pynvml.nvmlInit(); gpu=pynvml.nvmlDeviceGetHandleByIndex(0)
        except Exception: gpu=None
        slot=LatestFrameSlot(); producer=VideoProducer(video,slot)
        if cooperative: slot.sink=fast.publish_latest
        store=GeometryStore(); latch=EventLatch(); robot=RecordingRobot()
        lastfast=-1; next_shared='fast'; last_semantic=0.; last_plan=0.; last_transition=0.
        start=perf_counter(); sample=0.; persisted=0.; latest_semantic_payload=None
        producer.start()
        while producer.origin is None and not producer.done.is_set(): sleep(.001)
        origin=producer.origin; report['origin_perf_counter']=origin
        report['status']='running'
        while not producer.done.is_set() or any(w.busy for w in workers):
            now=perf_counter(); active=not producer.done.is_set()
            for w in workers:
                result=w.poll()
                if result is None: continue
                req=w.request
                if result['kind']=='fast':
                    lastfast=max(lastfast,result['generation'])
                    result['slow_busy_at_dispatch']=result.get('cooperative_fast_update',False) or req.get('slow_busy',False)
                    before=store.state.snapshot.model_dump(mode='json')
                    accepted,reason=store.accept_fast(result,perf_counter())
                    result.update(accepted=accepted,rejection_reason=reason,state_before=before,
                        state_after=store.state.snapshot.model_dump(mode='json'),episode=store.episode,
                        state_update_timestamp=perf_counter())
                    report['fast_results'].append(result)
                    if accepted:
                        transition=store.state.snapshot.transition.value
                        if transition in {'APPEARED','DISAPPEARED','UNKNOWN'}:
                            event='INITIAL' if transition=='UNKNOWN' else transition
                            last_transition=now
                            if load=='events' and slow is not None and slow.busy and slow.request.get('episode')!=store.episode:
                                slow.cancel_obsolete()
                                report['events'].append({'type':'cancel_obsolete_slow','timestamp':now,
                                    'new_episode':store.episode,'old_request_generation':slow.request['generation']})
                            if load=='events' and latch.offer(event,store.combined(now),store.state.snapshot,result,store.episode,now):
                                report['events'].append({**latch.pending,'type':'planner_trigger'})
                        # A geometry-only channel. Does not modify the stored/model Planner Decision.
                        command=immediate_tracking(store,now)
                        if command is not None:
                            robot.execute(command); ready=perf_counter()
                            report['commands'].append({'channel':'immediate_geometry','generation':result['generation'],
                                'source_timestamp':result['source_timestamp'],'capture_timestamp':result['capture_timestamp'],
                                'ready_timestamp':ready,'event_to_ready_age_s':ready-result['capture_timestamp'],
                                'decision':command.model_dump(mode='json',exclude_none=True),'mock_robot_output':robot.last_output})
                else:
                    result['request_context']={k:v for k,v in req.items() if k!='bgr'}
                    report['slow_results'].append(result)
                    if result.get('cancelled'):
                        report['events'].append({'type':'slow_cancelled','timestamp':now,
                            'kind':result['kind'],'generation':result['generation']})
                        continue  # Partial raw output is retained, never repaired or sent to Robot.
                    if result['kind']=='semantic':
                        before=store.state.snapshot.model_dump(mode='json')
                        store.accept_semantic(result,req.get('episode',store.episode))
                        payload=store.combined(now).model_dump(mode='json') if store.observation else None
                        if load=='events' and payload is not None and payload!=latest_semantic_payload:
                            if latch.offer('SEMANTIC_CHANGED',store.combined(now),store.state.snapshot,
                                           store.result,store.episode,now):
                                report['events'].append({**latch.pending,'type':'planner_trigger'})
                        latest_semantic_payload=payload
                        assert before==store.state.snapshot.model_dump(mode='json')
                    elif load=='events':
                        original=PlannerDecision.model_validate(result['decision'])
                        # Reject obsolete intents, never relabel their capture time as completion time.
                        valid=(req.get('episode')==store.episode and now-req['capture_timestamp']<=4.)
                        resolved=bind_latest(original,store,now) if valid else None
                        result['intent_accepted']=valid; result['resolved_final_decision']=resolved.model_dump(mode='json',exclude_none=True) if resolved else None
                        if resolved is not None:
                            robot.execute(resolved); ready=perf_counter(); last_plan=ready
                            report['commands'].append({'channel':'vlm_semantic','trigger':req.get('event_kind'),
                                'generation':req['generation'],'source_timestamp':req['source_timestamp'],
                                'capture_timestamp':req['capture_timestamp'],'ready_timestamp':ready,
                                'decision':resolved.model_dump(mode='json',exclude_none=True),'raw_decision':result['decision'],
                                'raw_response':result['raw_response'],'mock_robot_output':robot.last_output})
            frame=slot.latest(lastfast)
            shared=(fast is not None and fast is slow)
            # Stress comparison intentionally exposes safe shared-Qwen blocking.
            if active and fast is not None and not fast.busy and frame is not None and (not shared or next_shared=='fast'):
                request=frame_request(frame,'fast'); request['slow_busy']=slow.busy if slow and slow is not fast else False
                fast.submit(request); lastfast=frame.generation
                if shared: next_shared='slow'
            if active and slow is not None and not slow.busy and (not shared or next_shared=='slow'):
                latest=slot.latest(); request=None
                if load=='events' and store.observation is not None:
                    if not latch.pending and last_plan and now-last_plan>4:
                        if latch.offer('EXPIRED',store.combined(now),store.state.snapshot,store.result,store.episode,now):
                            report['events'].append({**latch.pending,'type':'planner_trigger'})
                    event=latch.take(now)
                    if event:
                        request={**event,'kind':'planner','event_kind':event['kind']}
                    elif (not latch.pending and latest is not None and last_plan and now-last_plan>1.5
                          and now-last_semantic>8 and now-last_transition>1.5):
                        request={**frame_request(latest,'semantic'),'episode':store.episode}
                        last_semantic=now
                elif load=='semantic' and latest is not None:
                    request={**frame_request(latest,'semantic'),'episode':store.episode}
                elif load=='planner' and latest is not None:
                    if store.observation is not None:
                        obs,snapshot=store.combined(now),store.state.snapshot
                    else:
                        # Generic load fixture, no labels, no video-specific expected action.
                        obs=CurrentObservation(person_visible=True,person_count=1,person_direction='LEFT',
                            person_distance='NEAR',blocking_obstacle='NONE'); snapshot=State().update(obs)
                    request={'kind':'planner','generation':latest.generation,'source_timestamp':latest.source_timestamp,
                        'capture_timestamp':latest.capture_timestamp,'observation':obs.model_dump(mode='json'),
                        'snapshot':snapshot.model_dump(mode='json'),'workload':'continuous stress, not per-camera product planning'}
                if request:
                    slow.submit(request)
                    report['events'].append({'type':'slow_dispatch','kind':request['kind'],'timestamp':now,
                        'generation':request['generation'],'capture_timestamp':request['capture_timestamp']})
                    if shared: next_shared='fast'
            if now-sample>=.5:
                record={'timestamp':now,'cpu_process_percent':{str(pid):p.cpu_percent() for pid,p in cpus.items()}}
                if gpu is not None:
                    try:
                        record['gpu_utilization_percent']=pynvml.nvmlDeviceGetUtilizationRates(gpu).gpu
                        record['gpu_used_gib']=pynvml.nvmlDeviceGetMemoryInfo(gpu).used/2**30
                    except Exception as exc: record['gpu_telemetry_error']=str(exc)
                report['resources'].append(record); sample=now
            if now-persisted>=1:
                write_report(output,report); persisted=now
            sleep(.001)
        producer.join(timeout=2)
        report.update(status='complete',video=producer.metadata,producer_error=producer.error,
            producer_frames=slot.published,latest_slot_overwrites=slot.overwritten,slot_capacity=1,
            final_state=store.state.snapshot.model_dump(mode='json'),event_coalesced=latch.coalesced,
            worker_counts=[{'pid':w.process.pid,'sent':w.sent,'completed':w.completed} for w in workers])
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally:
        if 'producer' in locals() and not producer.done.is_set(): producer.stop.set(); producer.join(timeout=3)
        for w in workers: w.close()
        write_report(output,report)
    print(f"{case}: {report['status']} fast={len(report['fast_results'])} slow={len(report['slow_results'])}",flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('video',type=Path)
    parser.add_argument('--cases',default='hybrid'); parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--model',type=Path,default=Path('.cache/phase36/yolo11n.pt'))
    parser.add_argument('--device',default='0'); parser.add_argument('--output-dir',type=Path,default=Path('reports/phase36/realtime'))
    parser.add_argument('--confidence',type=float,default=.1)
    args=parser.parse_args()
    if not 0<args.confidence<=1: parser.error('Confidence must be in (0,1]')
    for case in args.cases.split(','):
        for i in range(args.repeats): run_case(args.video,case,args.model,args.output_dir/f'{case}_{i}.json',args.device,args.confidence)


if __name__=='__main__': main()
