"""Experimental serializers around the same immutable contracts and State."""
from time import perf_counter
from brain.split_brain import PerceptionResult, PlannerResult
from brain.state import State
from . import codecs


class LatencyBrain:
    def __init__(self,engine): self.engine=engine; self.state=State()

    def decide(self,image):
        start=perf_counter(); before=self.state.snapshot; p=PerceptionResult(); d=PlannerResult()
        pstart=perf_counter()
        try:
            p.raw_response,p.metrics=self.engine.perceive(image)
            if p.metrics.get('class_logits_finite') is False: raise ValueError('Nonfinite scene logits')
            validation=perf_counter()
            try: p.observation=codecs.observation(p.raw_response,self.engine.profile.stage_format('perception'))
            finally: p.metrics['validation_parsing_s']=perf_counter()-validation
        except Exception as exc:
            p.fallback=True; p.error=f'{type(exc).__name__}: {exc}'
            if not p.metrics and hasattr(self.engine,'failure_metrics'): p.metrics=self.engine.failure_metrics('perception')
        ptime=perf_counter()-pstart; sstart=perf_counter()
        after=self.state.update(p.observation) if not p.fallback else before
        stime=perf_counter()-sstart; dstart=perf_counter()
        if not p.fallback:
            try:
                d.raw_response,d.metrics=self.engine.plan(p.observation,after)
                if d.metrics.get('class_logits_finite') is False: raise ValueError('Nonfinite Planner logits')
                validation=perf_counter()
                try: d.decision=codecs.decision(d.raw_response,self.engine.profile.stage_format('planner'))
                finally: d.metrics['validation_parsing_s']=perf_counter()-validation
            except Exception as exc:
                d.fallback=True; d.error=f'{type(exc).__name__}: {exc}'
                if not d.metrics and hasattr(self.engine,'failure_metrics'): d.metrics=self.engine.failure_metrics('planner')
        else: d.fallback=True; d.error='Planner skipped: invalid Perception'
        dtime=perf_counter()-dstart if not p.fallback else 0.0
        return {'perception':p.to_dict(),'planner':d.to_dict(),'state_before':before.model_dump(mode='json'),
                'state_after':after.model_dump(mode='json'),'transition':after.transition.value,
                'timing':{'perception_wall_s':ptime,'state_update_s':stime,'planner_wall_s':dtime,'total_brain_s':perf_counter()-start}}
