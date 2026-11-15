"""Observe the button pair and correct the stance while the arm is idle."""

from copy import copy
from dataclasses import asdict, replace
from types import SimpleNamespace
import time

import numpy as np


class ButtonApproach:
    def __init__(self, runtime, detector, button_set, interface, *, execute):
        if not {'up', 'down'}.issubset(detector.class_names):
            raise ValueError('脚補正には▲と▼を識別するYOLOの重みが必要です')
        load_config = runtime.module('config').load_config
        self.approach = runtime.module('approach')
        self.buttons = runtime.module('elevator_buttons')
        loco = runtime.module('loco')
        dds = runtime.module('dds')
        settings = load_config('approach.yaml')
        self.cfg = settings['approach']
        elevator = load_config('elevator.yaml')
        geometry = elevator['geometry']
        geometry['button_set'] = button_set
        # Keep detecting the pair while its yaw is outside the press planner's range.
        geometry['normal_max_tilt_deg'] = settings['face']['wall']['max_yaw_deg']
        self.general_min_height = geometry['wheelchair_max_z_m'] if button_set == 'general' else None
        press = elevator['press']
        self.stand = dict(press.get('by_button_set', {}).get(button_set, {}).get('stand', press['stand']))
        self.stand.setdefault('center_side_m', 0.)
        self.target_dist = self.cfg.get('target_dist_m')
        if self.target_dist is None:
            lo, hi = self.stand['min_dist_m'], self.stand['max_dist_m']
            self.target_dist = self.stand.get('target_dist_m', (lo + hi) / 2.)
        self.tracking = elevator['tracking']

        def detect(rgb):
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            detections = (d for d in detector.detect(bgr) if d.class_name in self.buttons.ARROWS)
            return self.buttons.yolo_candidates(rgb, detections)

        self.finder = self.buttons.ButtonFinder.from_config(
            load_config('robot.yaml'), elevator, detector=SimpleNamespace(detect=detect))
        dds.init_dds(0, interface)
        reader = dds.LowStateReader(0, interface)
        reader.open()
        reader.wait(10.)
        self.still_cfg = settings['loco']['still']
        self.stillness = loco.LegStillness(reader, self.still_cfg)
        # This is the private configuration of the confirmed approach session.
        settings['loco']['axes']['backward']['enabled'] = True
        settings['loco']['max_duration_s'] = float('inf')
        for axis in settings['loco']['axes'].values():
            axis['max_duration_s'] = float('inf')
        backend = loco.SdkLocoBackend() if execute else None
        self.mover = loco.LocoMover(backend, self.stillness, settings['loco'], go=execute)
        self.before_reverse = None
        self.reverse_checked = False

    def wait_still(self):
        self.stillness.reset()
        deadline = time.monotonic() + self.still_cfg['timeout_s']
        while time.monotonic() < deadline:
            if self.stillness.is_still():
                return
            time.sleep(self.still_cfg['poll_s'])
        raise RuntimeError('観測前の脚の停止を確認できません')

    def measure(self, camera, configuration, history):
        tracker = self.buttons.ButtonTracker(self.tracking)
        capture_after = time.time()
        deadline = time.monotonic() + 30.
        reason = '複数フレームで同じ組が安定しません'
        for _ in range(self.cfg['max_frames']):
            frame, stamp = camera.read_after(capture_after, deadline)
            config = configuration()
            try:
                q, _ = history.at(stamp, config, max_distance_s=.1)
            except ValueError as exc:
                tracker.reset()
                reason = str(exc)
                continue
            intr = frame.intrinsics
            matrix = np.array(((intr.fx, 0., intr.cx), (0., intr.fy, intr.cy), (0., 0., 1.)))
            try:
                call = self.finder.find(
                    frame.color_bgr[:, :, ::-1], frame.depth_m(), matrix, q[12:15])
                if self.general_min_height is not None and call.up.center[2] <= self.general_min_height:
                    raise self.buttons.ButtonNotFound('一般用の高さにある▲▼を確認できません')
            except self.buttons.ButtonNotFound as exc:
                tracker.reset()
                reason = str(exc)
                continue
            stable = tracker.add(call)
            if stable is not None:
                return self.approach.Measurement.from_call(stable, q)
        raise RuntimeError(f'同じ組の▲▼を確認できません: {reason}')

    def required_correction(self, measured):
        distance = measured.distance_m
        if distance < self.cfg['min_safe_dist_m']:
            raise RuntimeError(f'柱まで{distance * 100:.1f}cm。歩行の中止境界より近すぎます')
        if self.before_reverse is not None and not self.reverse_checked:
            if distance <= self.before_reverse or distance < self.stand['min_dist_m']:
                raise RuntimeError('後退後に柱との距離を確保できません')
            self.reverse_checked = True
        if distance < self.stand['min_dist_m']:
            return self.reverse(distance), '推奨距離の下限より近い'
        if measured.call.normal[0] < .9:
            return self.turn_to_panel(measured)
        return None

    def turn_to_panel(self, measured):
        decision = self.approach.decide(measured, None, {}, {'right': self.stand}, self.cfg, self.mover)
        if decision.pulse is None or not decision.pulse.axis.startswith('turn_'):
            raise RuntimeError('押下計画の範囲外のパネル角度を旋回で補正できません')
        return decision.pulse, decision.reason

    def reverse(self, distance):
        if self.before_reverse is not None:
            raise RuntimeError('自動後退は1回までです')
        return self.mover.plan('backward', self.target_dist - distance)

    def correction(self, measured, reason):
        decision = self.approach.decide(
            measured, None, {'right': reason}, {'right': self.stand}, self.cfg, self.mover)
        pulse = decision.pulse
        if pulse is not None:
            if pulse.axis not in ('left', 'right'):
                return pulse, decision.reason
            error = abs(measured.side_m - self.stand['center_side_m'])
            worst_step = abs(self.mover.side_step(pulse)) * (1. + self.cfg['step_uncertainty'])
            if worst_step < 2. * error:
                return pulse, decision.reason
            reason = '横移動の最小刻みで横ずれが悪化する可能性があります'
            decision = self.approach.decide(
                measured, None, {'right': reason}, {'right': self.stand},
                {**self.cfg, 'side_tol_m': float('inf')}, self.mover)
            if decision.pulse is not None:
                return decision.pulse, decision.reason
        if measured.distance_m < self.target_dist:
            return self.reverse(measured.distance_m), f'{decision.reason}。後退して測り直す'
        raise RuntimeError(decision.reason)

    def walk(self, measured, pulse, row):
        if pulse.axis == 'backward':
            self.before_reverse = measured.distance_m
        self.stillness.reset()
        result = self.mover.pulse(pulse)
        row.update(stopped=result.stopped, settle_s=result.settle_s)
        if not result.stopped:
            raise RuntimeError(result.note)

    def plan_corrections(self, measured, *, initial=None):
        """Plan the stance corrections from one observation using estimated motion."""
        normal = np.asarray(measured.call.normal, dtype=float).copy()
        if normal.shape != (3,) or not np.isfinite(normal).all() or abs(normal[0]) < 1e-6:
            raise RuntimeError('柱の向きを使って補正を計画できません')
        midpoint = (np.asarray(measured.call.up.center) + np.asarray(measured.call.down.center)) / 2
        point = np.array([(measured.distance_m - normal[1] * measured.side_m - normal[2] * midpoint[2]) / normal[0],
                          measured.side_m, midpoint[2]], dtype=float)
        predicted = measured
        planner = copy(self.mover)
        pulses = []
        axes_used = set()
        for index in range(4):
            if predicted.distance_m < self.cfg['min_safe_dist_m']:
                raise RuntimeError('計画した補正後の位置が柱に近すぎます')
            if index == 0 and initial is not None:
                pulse, reason = initial
            elif predicted.distance_m < self.stand['min_dist_m']:
                if self.before_reverse is not None or 'backward' in axes_used:
                    raise RuntimeError('後退後の距離を確認できません')
                pulse = planner.plan('backward', self.target_dist - predicted.distance_m)
                reason = '推奨距離の下限より近い'
            else:
                decision = self.approach.decide(predicted, None, {}, {'right': self.stand},
                                                {**self.cfg, 'target_dist_m': self.target_dist}, planner)
                pulse, reason = decision.pulse, decision.reason
            if pulse is None or pulse.axis in axes_used:
                break
            planner.check(pulse)
            axes_used.add(pulse.axis)
            pulses.append((pulse, reason))
            planner.n_pulses += 1
            if pulse.axis == 'forward':
                planner.forward_total_m += planner.cfg['axes']['forward']['rate'] * pulse.duration_s
            planner.side_total_m += planner.side_step(pulse)
            amount = planner.cfg['axes'][pulse.axis]['rate'] * pulse.duration_s
            if pulse.axis.startswith('turn_'):
                angle = amount if pulse.axis == 'turn_left' else -amount
                c, s = np.cos(angle), np.sin(angle)
                rotation = np.array(((c, s, 0.), (-s, c, 0.), (0., 0., 1.)))
                point, normal = rotation @ point, rotation @ normal
            elif pulse.axis == 'forward':
                point[0] -= amount
            elif pulse.axis == 'backward':
                point[0] += amount
            else:
                point[1] -= amount if pulse.axis == 'left' else -amount
            predicted = replace(predicted, distance_m=float(point @ normal), side_m=float(point[1]),
                                yaw_rad=float(np.arctan2(normal[1], normal[0])),
                                call=SimpleNamespace(normal=normal))
        return pulses


def plan_at_stops(approach, measure, search, move, *, steps, budget_s):
    """Return the actual accepted PlanResult from the last stationary observation."""
    with approach.mover:
        for index in range(approach.cfg['max_steps']):
            row = {'step': index + 1}
            steps.append(row)
            measured = measure()
            row.update(distance_m=measured.distance_m, side_m=measured.side_m,
                       yaw_deg=float(np.degrees(measured.yaw_rad)))
            print(f'[approach] {measured.summary()}', flush=True)
            approach.required_correction(measured)
            corrections = approach.plan_corrections(measured)
            if not corrections:
                if budget_s <= 0:
                    raise RuntimeError('押下軌道の合計計画時間を超えました')
                plan = search(measured, budget_s)
                if plan is None:
                    row.update(decision='turn', reason='本押しのパネル角度が計画範囲外')
                    corrections = approach.plan_corrections(measured, initial=approach.turn_to_panel(measured))
                else:
                    status = plan.report['search_status']
                    budget_s -= plan.report['total_wall_s']
                    row.update(search_status=status, planning_s=plan.report['total_wall_s'],
                               candidates_attempted=plan.report['candidates_attempted'])
                    if budget_s <= 0 or status == 'budget_exhausted':
                        raise RuntimeError('押下軌道の合計計画時間を超えました')
                    if status == 'accepted' and plan.candidate is not None:
                        row['decision'] = 'press'
                        print('[approach] 本押しの軌道が合格しました', flush=True)
                        return plan
                    if status != 'candidates_exhausted':
                        raise RuntimeError('押下軌道の探索結果が不正です')
                    reasons = [entry['reason'] for entry in plan.report['candidates'] if 'reason' in entry]
                    reason = ' / '.join(dict.fromkeys(reasons)) or plan.report['reason']
                    row['rejections'] = reasons
                    corrections = approach.plan_corrections(measured, initial=approach.correction(measured, reason))
            row.update(decision='move', pulses=[dict(pulse=asdict(pulse), reason=reason)
                                                for pulse, reason in corrections])
            for pulse, reason in corrections:
                print(f'[approach] {reason}。動作: {pulse.summary()}', flush=True)
            if not approach.mover.go:
                return None
            if index + 1 == approach.cfg['max_steps']:
                break
            move(measured, corrections, row)
        raise RuntimeError('押下可能な位置を確認する前に観測回数の上限に達しました')
