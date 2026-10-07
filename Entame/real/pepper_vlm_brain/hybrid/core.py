from dataclasses import dataclass
from threading import Lock
from time import perf_counter
import io
from contextlib import redirect_stdout
import math
from brain.split_schemas import CurrentObservation, PlannerDecision
from brain.state import State
from adapters.robot_mock import MockRobot


@dataclass(frozen=True)
class Frame:
    generation: int
    source_timestamp: float
    capture_timestamp: float
    decode_finished_timestamp: float
    bgr: object


class LatestFrameSlot:
    """One immutable frame reference. No FIFO and no history of image buffers."""
    def __init__(self):
        self._lock=Lock(); self._frame=None; self.published=0; self.overwritten=0
        self.sink=None

    def publish(self,frame):
        if not isinstance(frame,Frame): raise TypeError('Frame required')
        with self._lock:
            if self._frame is not None:
                if frame.generation<=self._frame.generation: return False
                self.overwritten+=1
            self._frame=frame; self.published+=1
            if self.sink is not None: self.sink(frame)
            return True

    def latest(self,after=-1):
        with self._lock:
            return self._frame if self._frame is not None and self._frame.generation>after else None


class GeometryStore:
    """Fast-only State history. Semantic metadata lives in a separate cache."""
    def __init__(self,max_fast_age=.5,semantic_ttl=3.0):
        self.state=State(); self.observation=None; self.result=None
        self.generation=-1; self.capture_timestamp=float('-inf'); self.episode=0
        self.semantic=None; self.semantic_generation=-1; self.semantic_episode=None
        self.max_fast_age=max_fast_age; self.semantic_ttl=semantic_ttl

    def accept_fast(self,result,now=None):
        now=perf_counter() if now is None else now
        if (type(result['generation']) is not int or result['generation']<0 or
            not all(math.isfinite(result[k]) for k in ['capture_timestamp','source_timestamp','processing_start','processing_finish']) or
            result['processing_start']<result['capture_timestamp'] or result['processing_finish']<result['processing_start'] or
            result['capture_timestamp']>now or result['processing_finish']>now):
            return False,'invalid source clock/generation'
        if (result['generation']<=self.generation or result['capture_timestamp']<=self.capture_timestamp):
            return False,'old generation/source time'
        if now-result['capture_timestamp']>self.max_fast_age:
            return False,'stale fast frame'
        obs=CurrentObservation.model_validate(result['observation'])
        previous=self.observation.person_visible if self.observation else None
        if previous is None or previous!=obs.person_visible: self.episode+=1
        self.observation=obs; self.result=result; self.generation=result['generation']
        self.capture_timestamp=result['capture_timestamp']; self.state.update(obs)
        return True,None

    def accept_semantic(self,result,episode):
        obs=CurrentObservation.model_validate(result['observation'])
        if result['generation']<=self.semantic_generation: return False
        self.semantic={**result,'observation':obs.model_dump(mode='json')}
        self.semantic_generation=result['generation']; self.semantic_episode=episode
        return True

    def combined(self,now=None):
        if self.observation is None: raise ValueError('Fast geometry required')
        now=perf_counter() if now is None else now
        data=self.observation.model_dump(mode='json')
        if self.semantic and 0<=now-self.semantic['capture_timestamp']<=self.semantic_ttl:
            semantic=self.semantic['observation']
            data['blocking_obstacle']=semantic['blocking_obstacle']
            if (data['person_visible'] and self.semantic_episode==self.episode and semantic['person_visible']
                    and semantic['person_direction']==data['person_direction']):
                data['person_distance']=semantic['person_distance']
        return CurrentObservation.model_validate(data)


class EventLatch:
    """One coalescing event snapshot, never a frame/event FIFO."""
    def __init__(self,debounce=.25):
        self.pending=None; self.last={}; self.debounce=debounce; self.coalesced=0

    def offer(self,kind,observation,snapshot,result,episode,now):
        if kind not in {'INITIAL','APPEARED','DISAPPEARED','SEMANTIC_CHANGED','EXPIRED','COMPLETED'}:
            raise ValueError('Unknown generic event')
        if (self.pending is not None and self.pending['kind'] in {'INITIAL','APPEARED','DISAPPEARED'}
                and kind not in {'INITIAL','APPEARED','DISAPPEARED'} and episode<=self.pending['episode']):
            return False  # Never lose the most recent visibility event to lower-priority refresh work.
        key=(kind,episode)
        if now-self.last.get(key,float('-inf'))<self.debounce: return False
        self.last[key]=now
        if self.pending is not None: self.coalesced+=1
        self.pending={'kind':kind,'observation':observation.model_dump(mode='json'),
            'snapshot':snapshot.model_dump(mode='json'),'generation':result['generation'],
            'source_timestamp':result['source_timestamp'],'capture_timestamp':result['capture_timestamp'],
            'episode':episode,'event_received_timestamp':now}
        return True

    def take(self,now=None,settle_seconds=.1):
        if (now is not None and self.pending is not None and self.pending['kind'] in {'APPEARED','DISAPPEARED'}
                and now-self.pending['event_received_timestamp']<settle_seconds):
            return None
        value=self.pending; self.pending=None; return value


def bind_latest(decision,store,now=None):
    if not isinstance(decision,PlannerDecision): raise TypeError('Validated Decision required')
    now=perf_counter() if now is None else now
    obs=store.observation
    fresh=(obs is not None and now-store.capture_timestamp<=store.max_fast_age)
    if decision.action.value in {'LOOK','FOUND'} and decision.target.value=='PERSON':
        if not fresh or not obs.person_visible or obs.person_direction.value=='UNKNOWN': return None
        data=decision.model_dump(mode='json',exclude_none=True); data['direction']=obs.person_direction.value
        return PlannerDecision.model_validate(data)
    if decision.action.value=='APPROACH' and (not fresh or not obs.person_visible or obs.person_direction.value!='CENTER'):
        return None
    return decision  # SEARCH preserves the model-selected memory direction.


def immediate_tracking(store,now=None):
    """Only LOOK geometry. No SEARCH/FOUND/APPROACH/AVOID decision rules."""
    now=perf_counter() if now is None else now
    obs=store.observation
    if obs is None or now-store.capture_timestamp>store.max_fast_age or not obs.person_visible or obs.person_direction.value=='UNKNOWN':
        return None
    return PlannerDecision(action='LOOK',direction=obs.person_direction,target='PERSON',distance='UNKNOWN')


class RecordingRobot(MockRobot):
    def execute(self,decision):
        # Preserve the existing validated Mock contract without per-frame console I/O.
        with redirect_stdout(io.StringIO()): super().execute(decision)
