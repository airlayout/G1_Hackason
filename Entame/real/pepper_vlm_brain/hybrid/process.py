import multiprocessing as mp
from time import perf_counter
from .workers import worker


class WorkerProcess:
    def __init__(self,backend,model_path=None,device='0',cooperative=False,confidence=.25):
        context=mp.get_context('spawn'); parent,child=context.Pipe()
        self.cancel=context.Event()
        self.latest=({'pixels':context.RawArray('B',8*1024*1024),'metadata':context.RawArray('d',6),'lock':context.Lock()}
                     if cooperative else None)
        if self.latest is not None: self.latest['metadata'][0]=-1
        self.pipe=parent; self.process=context.Process(target=worker,args=(child,backend,model_path,device,self.cancel,self.latest,confidence),daemon=True)
        self.busy=False; self.request=None; self.sent=0; self.completed=0
        self.process.start(); child.close()

    def publish_latest(self,frame):
        if self.latest is None: return
        import numpy as np
        pixels=frame.bgr.ravel()
        if pixels.size>len(self.latest['pixels']): raise ValueError('Frame exceeds shared latest-slot capacity')
        with self.latest['lock']:
            np.frombuffer(self.latest['pixels'],dtype=np.uint8)[:pixels.size]=pixels
            self.latest['metadata'][:]=[frame.generation,frame.source_timestamp,frame.capture_timestamp,
                frame.decode_finished_timestamp,frame.bgr.shape[0],frame.bgr.shape[1]]

    def ready(self,timeout=100):
        if not self.pipe.poll(timeout): raise RuntimeError('Worker initialization timeout')
        message=self.pipe.recv()
        if message['type']!='ready': raise RuntimeError(str(message))
        self.info=message; return message

    def submit(self,request):
        if self.busy: raise RuntimeError('Only one in-flight request; no queue')
        self.cancel.clear()
        request={**request,'dispatch_timestamp':perf_counter()}
        self.pipe.send(request); self.request=request; self.busy=True; self.sent+=1

    def cancel_obsolete(self):
        if self.busy: self.cancel.set(); return True
        return False

    def poll(self):
        if not self.busy or not self.pipe.poll(): return None
        message=self.pipe.recv()
        pushed=message['type']=='fast_update'
        if not pushed: self.busy=False; self.completed+=1
        if message['type']=='error': raise RuntimeError(str(message))
        result=message['result']; result['dispatch_timestamp']=(result['processing_start'] if pushed else self.request['dispatch_timestamp'])
        if pushed: result['cooperative_fast_update']=True
        result['received_timestamp']=perf_counter(); return result

    def close(self):
        if self.process.is_alive():
            try: self.pipe.send(None)
            except (EOFError,BrokenPipeError): pass
            self.process.join(timeout=8)
        if self.process.is_alive(): self.process.terminate(); self.process.join(timeout=5)
        self.pipe.close()
