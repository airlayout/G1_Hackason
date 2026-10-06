import cv2
from threading import Thread, Event
from time import perf_counter
from .core import Frame


class VideoProducer(Thread):
    def __init__(self,path,slot):
        super().__init__(daemon=True); self.path=str(path); self.slot=slot
        self.stop=Event(); self.done=Event(); self.origin=None; self.error=None; self.metadata={}

    def run(self):
        cap=cv2.VideoCapture(self.path)
        try:
            if not cap.isOpened(): raise ValueError('Video cannot be opened')
            fps=cap.get(cv2.CAP_PROP_FPS); count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.metadata={'fps':fps,'frame_count':count,'duration_s':count/fps}
            self.origin=perf_counter(); index=0
            while index<count and not self.stop.is_set():
                due=self.origin+index/fps
                self.stop.wait(max(0,due-perf_counter()))
                if self.stop.is_set(): break
                # Skip decode backlog too, never replay missed camera frames.
                latest=min(count-1,max(index,int((perf_counter()-self.origin)*fps)))
                if latest!=index: cap.set(cv2.CAP_PROP_POS_FRAMES,latest)
                index=latest; ok,bgr=cap.read()
                if not ok: raise ValueError(f'Video decode failed at {index}')
                bgr.setflags(write=False)
                self.slot.publish(Frame(index,index/fps,self.origin+index/fps,perf_counter(),bgr))
                index+=1
            self.stop.wait(max(0,self.origin+count/fps-perf_counter()))
        except Exception as exc: self.error=f'{type(exc).__name__}: {exc}'
        finally: cap.release(); self.done.set()
