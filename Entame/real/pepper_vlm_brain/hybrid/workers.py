"""One Pipe request in flight. Qwen has one GPU owner; YOLO has its own worker."""
import os
from pathlib import Path
from time import perf_counter
import traceback


def person_detection(rows,width,confidence=.25):
    import math
    from brain.split_schemas import CurrentObservation
    valid=[tuple(float(x) for x in row) for row in rows if len(row)==6
        and all(math.isfinite(float(x)) for x in row) and row[5]==0 and row[4]>=confidence
        and row[2]>row[0] and row[3]>row[1]]
    # Largest visible bbox, then confidence, then x/y coordinates: generic, stateless.
    valid.sort(key=lambda r:(-(r[2]-r[0])*(r[3]-r[1]),-r[4],r[0],r[1]))
    selected=valid[0] if valid else None
    center=(selected[0]+selected[2])/2/width if selected else None
    side=('LEFT' if center<.4 else 'RIGHT' if center>.6 else 'CENTER') if selected else 'UNKNOWN'
    obs=CurrentObservation(person_visible=bool(valid),person_count=len(valid),person_direction=side,
        person_distance='UNKNOWN',blocking_obstacle='UNKNOWN')
    return {'observation':obs.model_dump(mode='json'),'bbox':list(selected[:4]) if selected else None,
        'bbox_center_x':center,'confidence':selected[4] if selected else None,'detections':[list(r) for r in valid]}


def worker(connection,backend,model_path=None,device='0',cancel=None,latest=None,confidence=.25):
    os.environ['HF_HUB_OFFLINE']='1'; os.environ['YOLO_AUTOINSTALL']='false'
    os.environ['YOLO_CONFIG_DIR']=str(Path('.cache/phase36/yolo').resolve())
    Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True,exist_ok=True)
    os.environ['ULTRALYTICS_HUB_DISABLE']='1'
    try:
        import torch
        torch.set_num_threads(4 if backend=='qwen' else 2)
        if backend=='yolo':
            from ultralytics import YOLO,settings
            settings.update({'sync':False})
            import numpy as np
            model=YOLO(str(model_path))
            kwargs=dict(device=device,classes=[0],imgsz=640,conf=confidence,iou=.7,verbose=False,save=False,
                        half=False,augment=False,max_det=100)
            for _ in range(3): model.predict(np.zeros((512,910,3),dtype=np.uint8),**kwargs)
            info={'backend':'YOLO11n','device':device,'model':str(model_path),'precision':'FP32',
                  'imgsz':640,'confidence':confidence,'target_selection':'largest bbox area; confidence; x/y tie break'}
        else:
            from phase35.engine import FastQwen
            from phase35.profiles import PROFILES
            from latency.profiles import PROFILES as STABLE
            from brain.split_schemas import CurrentObservation
            from brain.state import State
            from PIL import Image
            engine=FastQwen(PROFILES['A']); engine.load()
            active_kind=None; in_fast_callback=False; last_fast_generation=-1; last_fast_start=float('-inf')
            def yield_fast_at_token_boundary():
                nonlocal in_fast_callback,last_fast_generation,last_fast_start
                if latest is None or in_fast_callback or active_kind not in {'planner','semantic'}:
                    return
                if perf_counter()-last_fast_start<.12: return
                import numpy as np
                import cv2
                from phase35.codecs import observation
                with latest['lock']:
                    gen,source,cap,decoded,h,w=latest['metadata'][:]
                    if gen<=last_fast_generation: return
                    bgr=np.frombuffer(latest['pixels'],dtype=np.uint8)[:int(h*w*3)].copy().reshape(int(h),int(w),3)
                start=perf_counter(); last_fast_start=start
                profile=engine.profile
                # Separate generation KV caches are local to generate. This one mutable
                # model-level RoPE cache must also be restored before slow decoding resumes.
                rope=engine.model.model.rope_deltas
                in_fast_callback=True
                try:
                    engine.configure(PROFILES['A'])
                    raw,metrics=engine.perceive(Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)))
                    if metrics.get('class_logits_finite') is False: raise ValueError('Nonfinite cooperative class logits')
                    obs=observation(raw,'class'); finish=perf_counter()
                    result={'kind':'fast','generation':int(gen),'source_timestamp':source,'capture_timestamp':cap,
                        'processing_start':start,'processing_finish':finish,'inference_finished_timestamp':finish,
                        'inference_latency_s':finish-start,'result_age_s':finish-cap,'raw_response':raw,
                        'observation':obs.model_dump(mode='json'),'metrics':metrics,
                        'peak_allocated_gib':metrics['peak_allocated_gib'],'peak_reserved_gib':metrics['peak_reserved_gib']}
                finally:
                    engine.configure(profile); engine.model.model.rope_deltas=rope; in_fast_callback=False
                result['slow_rope_cache_restored']=engine.model.model.rope_deltas is rope
                last_fast_generation=int(gen)
                connection.send({'type':'fast_update','result':result})
            # A single owner generates; stopping is only a cancellation signal,
            # never an action/class rule. Original stable generate remains unchanged.
            if cancel is not None:
                from transformers import StoppingCriteria,StoppingCriteriaList
                class CancelAtTokenBoundary(StoppingCriteria):
                    def __call__(self,input_ids,scores,**kwargs):
                        if cancel.is_set(): return True
                        yield_fast_at_token_boundary()
                        return False
                original_generate=engine.model.generate
                def cancellable_generate(*args,**kwargs):
                    kwargs['stopping_criteria']=StoppingCriteriaList([CancelAtTokenBoundary()])
                    return original_generate(*args,**kwargs)
                engine.model.generate=cancellable_generate
            engine.perceive(Image.new('RGB',(910,512)))
            engine.configure(STABLE['json_linear_uninstrumented'])
            obs=CurrentObservation(person_visible=True,person_count=1,person_direction='LEFT',
                person_distance='NEAR',blocking_obstacle='NONE')
            engine.plan(obs,State().update(obs))
            info={'backend':'shared-one-Qwen2B','model_load':engine.last_metrics,
                  'cooperative_token_boundary_fast_updates':latest is not None}
        connection.send({'type':'ready','info':info,'pid':os.getpid()})
        while True:
            request=connection.recv()
            if request is None: break
            start=perf_counter(); kind=request['kind']
            if backend=='qwen': active_kind=kind
            result={'kind':kind,'generation':request['generation'],'source_timestamp':request['source_timestamp'],
                'capture_timestamp':request['capture_timestamp'],'processing_start':start}
            if backend=='yolo':
                predictions=model.predict(request['bgr'],**kwargs)[0]
                result.update(person_detection(predictions.boxes.data.cpu().tolist(),request['bgr'].shape[1],confidence))
                result['raw_model_boxes']=predictions.boxes.data.cpu().tolist()
                result['metrics']={'ultralytics_speed_ms':predictions.speed}
            elif kind in {'fast','semantic'}:
                from PIL import Image
                import cv2
                from brain.split_brain import parse_flat
                from brain.split_schemas import CurrentObservation
                from phase35.codecs import observation
                engine.configure(PROFILES['A'] if kind=='fast' else STABLE['json_linear_uninstrumented'])
                raw,metrics=engine.perceive(Image.fromarray(cv2.cvtColor(request['bgr'],cv2.COLOR_BGR2RGB)))
                if cancel is not None and cancel.is_set():
                    finish=perf_counter()
                    result.update(cancelled=True,raw_response=raw,metrics=metrics,processing_finish=finish,
                        inference_latency_s=finish-start,result_age_s=finish-request['capture_timestamp'])
                    connection.send({'type':'result','result':result}); continue
                if metrics.get('class_logits_finite') is False: raise ValueError('Nonfinite fast class logits')
                obs=observation(raw,'class') if kind=='fast' else parse_flat(raw,CurrentObservation)
                result.update(raw_response=raw,observation=obs.model_dump(mode='json'),metrics=metrics)
            elif kind=='planner':
                from brain.split_brain import parse_flat
                from brain.split_schemas import CurrentObservation,PlannerDecision
                from brain.state import StateSnapshot
                engine.configure(STABLE['json_linear_uninstrumented'])
                obs=CurrentObservation.model_validate(request['observation'])
                snapshot=StateSnapshot.model_validate(request['snapshot'])
                raw,metrics=engine.plan(obs,snapshot)
                if cancel is not None and cancel.is_set():
                    finish=perf_counter()
                    result.update(cancelled=True,raw_response=raw,metrics=metrics,processing_finish=finish,
                        inference_latency_s=finish-start,result_age_s=finish-request['capture_timestamp'])
                    connection.send({'type':'result','result':result}); continue
                result.update(raw_response=raw,decision=parse_flat(raw,PlannerDecision).model_dump(mode='json',exclude_none=True),
                    metrics=metrics,planner_input=metrics['planner_input'])
            else: raise ValueError('Unknown worker request')
            if device!='cpu':
                torch.cuda.synchronize()
                result['peak_allocated_gib']=torch.cuda.max_memory_allocated()/2**30
                result['peak_reserved_gib']=torch.cuda.max_memory_reserved()/2**30
            finish=perf_counter()
            result.update(processing_finish=finish,inference_finished_timestamp=finish,
                inference_latency_s=finish-start,result_age_s=finish-request['capture_timestamp'])
            connection.send({'type':'result','result':result})
            if backend=='qwen' and kind=='fast': last_fast_generation=max(last_fast_generation,result['generation'])
    except EOFError: pass
    except Exception as exc:
        connection.send({'type':'error','error':f'{type(exc).__name__}: {exc}','traceback':traceback.format_exc()})
    finally: connection.close()
