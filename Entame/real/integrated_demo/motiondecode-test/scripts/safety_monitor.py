"""Independent ownership watchdog and separated runtime safety assessment."""
from copy import deepcopy
import threading
import time

import numpy as np

from common import JOINTS, NAMES


def _endpoint_fingerprint(report):
    publications=report.get('publications',{})
    rows=[]
    for topic,endpoints in publications.items():
        for endpoint in endpoints:
            rows.append((topic,endpoint.get('endpoint_guid'),endpoint.get('participant_guid'),
                         endpoint.get('classification')))
    return tuple(sorted(rows))


class OwnershipWatchdog:
    """Poll DDS ownership off the fast loop and latch any unsafe result.

    A successful check means discovery completed, regardless of whether the
    resulting ownership state was safe. A conflict remains latched until an
    explicit new session reset while the worker is stopped.
    """
    def __init__(self,snapshot_fn,period_s=.1,clock=time.monotonic):
        if not np.isfinite(period_s) or period_s<=0:
            raise ValueError('Ownership watchdog period must be finite and positive')
        self.snapshot_fn=snapshot_fn;self.period_s=float(period_s);self.clock=clock
        self.lock=threading.Lock();self.stop_event=threading.Event();self.thread=None
        self.discovery_lock=threading.Lock();self.synchronous_refresh_lock=threading.Lock()
        self.reset_session()

    def reset_session(self):
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('Stop ownership watchdog before starting a new session')
        with self.lock:
            self.current=None;self.last_check_started=None;self.last_check_time=None
            self.last_success_time=None;self.last_error=None;self.latched_fault=False
            self.latched_reasons=[];self.latched_at=None;self.records=[]
            self.change_events=[];self.previous_fingerprint=None

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('Ownership watchdog is already running')
        self.stop_event.clear()
        self.thread=threading.Thread(target=self._run,name='ownership-watchdog',daemon=True)
        self.thread.start()
        return self

    def _run(self):
        previous_start=None
        while not self.stop_event.is_set():
            started=self.clock()
            with self.discovery_lock:
                self._perform_check(started,previous_start)
            previous_start=started
            self.stop_event.wait(max(0.,self.period_s-(self.clock()-started)))

    def _perform_check(self,started=None,previous_start=None):
        started=self.clock() if started is None else started
        error=None;report=None
        try:report=self.snapshot_fn()
        except Exception as exc:error=f'{type(exc).__name__}: {exc}'
        ended=self.clock()
        with self.lock:
            record={'check_started_monotonic_s':started,
                    'check_completed_monotonic_s':ended,
                    'check_duration_s':ended-started,
                    'check_interval_s':None if previous_start is None else started-previous_start,
                    'successful':report is not None,'error':error}
            self.last_check_started=started;self.last_check_time=ended;self.last_error=error
            if report is not None:
                self.current=deepcopy(report);self.last_success_time=ended
                record.update(ownership_passed=bool(report.get('passed')),
                              external_writer_count=len(report.get('external_arm_sdk_writers',[])),
                              armsdk_writer_count=len(report.get('armsdk_writers',[])),
                              arm_action_state=report.get('arm_action',{}).get('status'))
                fingerprint=_endpoint_fingerprint(report)
                if self.previous_fingerprint is not None and fingerprint!=self.previous_fingerprint:
                    self.change_events.append({'monotonic_s':ended,
                                               'previous':self.previous_fingerprint,
                                               'current':fingerprint})
                self.previous_fingerprint=fingerprint
                if not report.get('passed') and not self.latched_fault:
                    self.latched_fault=True;self.latched_at=ended
                    self.latched_reasons=list(report.get('reasons',[])) or ['ownership unsafe']
            self.records.append(record)

    @staticmethod
    def _refresh_allowed(snapshot):
        return (snapshot['watchdog_running'] and snapshot['current_cached_safe']
                and snapshot['freshness_expired'] and not snapshot['latched_fault']
                and snapshot['latest_check_error'] is None)

    def fresh_snapshot(self,max_age_s,timeout_s=2.):
        """Return fresh ownership, synchronously refreshing only stale SAFE state.

        Unsafe, errored, latched, or stopped watchdog state remains fail-closed.
        A timed-out refresh may finish later and update the cache, but never
        authorizes the caller that observed the timeout.
        """
        if not np.isfinite(timeout_s) or timeout_s<=0:
            raise ValueError('Ownership refresh timeout must be finite and positive')
        initial=self.snapshot(max_age_s=max_age_s)
        if initial['ownership_safe'] or not self._refresh_allowed(initial):return initial
        completed=threading.Event();result={}
        def refresh():
            with self.synchronous_refresh_lock:
                current=self.snapshot(max_age_s=max_age_s)
                if not current['ownership_safe'] and self._refresh_allowed(current):
                    with self.discovery_lock:self._perform_check()
                    current=self.snapshot(max_age_s=max_age_s)
                result['snapshot']=current
            completed.set()
        threading.Thread(target=refresh,name='ownership-synchronous-refresh',daemon=True).start()
        if completed.wait(timeout_s):return result['snapshot']
        timed_out=self.snapshot(max_age_s=max_age_s)
        timed_out['ownership_safe']=False
        timed_out['reasons']=list(timed_out['reasons'])+[
            'fresh synchronous ownership check timed out']
        timed_out['synchronous_refresh_timed_out']=True
        return timed_out

    def wait_first(self,timeout_s=2.):
        deadline=self.clock()+timeout_s
        while self.clock()<deadline:
            with self.lock:
                ready=self.last_check_time is not None
            if ready:return self.snapshot()
            time.sleep(.005)
        raise RuntimeError('Ownership watchdog produced no check before timeout')

    def snapshot(self,max_age_s=None):
        now=self.clock()
        with self.lock:
            current=deepcopy(self.current);last_check=self.last_check_time
            last_success=self.last_success_time;error=self.last_error
            latched=self.latched_fault;reasons=list(self.latched_reasons)
            latched_at=self.latched_at
            running=self.thread is not None and self.thread.is_alive()
            events=deepcopy(self.change_events)
        success_age=None if last_success is None else now-last_success
        check_age=None if last_check is None else now-last_check
        expired=bool(max_age_s is not None and (success_age is None or success_age>max_age_s))
        current_safe=bool(current is not None and current.get('passed'))
        safe=current_safe and not latched and error is None and running and not expired
        why=[]
        if not running:why.append('ownership watchdog is not running')
        if current is None:why.append('no completed ownership discovery')
        if error:why.append('latest ownership discovery failed: '+error)
        if latched:why.extend('latched: '+value for value in reasons)
        if expired:why.append('last successful ownership check exceeds caller policy')
        return {'ownership_safe':safe,'current_cached_safe':current_safe,
                'last_check_age_s':check_age,'last_successful_check_age_s':success_age,
                'latest_check_error':error,'latched_fault':latched,
                'latched_reasons':reasons,'latched_at_monotonic_s':latched_at,
                'freshness_policy_s':max_age_s,'freshness_expired':expired,
                'watchdog_running':running,'reasons':why,'current':current,
                'participant_writer_change_events':events}

    def timing_records(self):
        with self.lock:return deepcopy(self.records)

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=max(2.,self.period_s*3))


def joint_limit_violations(q,controlled=range(12,29),margin=.03):
    q=np.asarray(q,dtype=float)
    if q.shape!=(29,) or not np.isfinite(q).all():return ['invalid 29-joint state']
    return [NAMES[i] for i in controlled
            if q[i]<JOINTS[i]['lower']+margin or q[i]>JOINTS[i]['upper']-margin]


def assess_collision_clearances(distances,baseline,clearance_m=.015,
                                baseline_tolerance_m=1e-6):
    new_penetration=[key for key,value in distances.items()
                     if baseline[key]>=0 and value<0]
    new_dangerous=[key for key,value in distances.items()
                   if baseline[key]>=clearance_m and value<clearance_m]
    existing={key:{'class':'existing_penetration' if baseline[key]<0 else 'existing_near_pair',
                   'baseline_m':baseline[key],'measured_m':value,
                   'delta_from_baseline_m':value-baseline[key],
                   'legacy_0_001mm_exceeded':value<baseline[key]-baseline_tolerance_m}
              for key,value in distances.items() if baseline[key]<clearance_m}
    baseline_exceeded=[key for key,value in existing.items()
                       if value['legacy_0_001mm_exceeded']]
    return {
        'new_penetration_pairs':new_penetration,
        'new_dangerous_pairs':new_dangerous,
        'baseline_relative':{'existing_pairs':existing,
                             'legacy_threshold_m':baseline_tolerance_m,
                             'legacy_threshold_exceeded_pairs':baseline_exceeded,
                             'calibrated_threshold_applied':False,
                             'stop_policy_changed':False},
    }


def assess_runtime_safety(q,q0,target,distances,baseline,lowstate_age_s,ownership,
                          clearance_m=.015,baseline_tolerance_m=1e-6,
                          tracking_limit_rad=.10,lowstate_stale_s=.15):
    """Return independent hard-safety and baseline-relative observations.

    ``baseline_tolerance_m`` remains the legacy 0.001 mm comparison for
    visibility only. Calibration candidates never alter or authorize a runtime
    threshold in this function.
    """
    q=np.asarray(q,dtype=float);q0=np.asarray(q0,dtype=float);target=np.asarray(target,dtype=float)
    limits=joint_limit_violations(q)
    collision=assess_collision_clearances(
        distances,baseline,clearance_m,baseline_tolerance_m)
    new_penetration=collision['new_penetration_pairs']
    new_dangerous=collision['new_dangerous_pairs']
    tracking=float(np.max(abs(q[12:]-target[12:])))
    hard_reasons=[]
    if lowstate_age_s>lowstate_stale_s:hard_reasons.append('LowState stale')
    if not ownership.get('ownership_safe'):hard_reasons.append('ownership unsafe')
    if limits:hard_reasons.append('joint limit violation')
    if new_penetration:hard_reasons.append('new penetration')
    if new_dangerous:hard_reasons.append('new dangerous clearance pair')
    if tracking>tracking_limit_rad:hard_reasons.append('large tracking error')
    return {
        'hard_safety':{'passed':not hard_reasons,'reasons':hard_reasons,
                       'new_penetration_pairs':new_penetration,
                       'new_dangerous_pairs':new_dangerous,
                       'joint_limit_violations':limits,
                       'tracking_error_rad':tracking,
                       'lowstate_age_s':float(lowstate_age_s),
                       'ownership_safe':bool(ownership.get('ownership_safe'))},
        'baseline_relative':collision['baseline_relative'],
        'q0_deviation_rad':float(np.max(abs(q-q0))),
    }
