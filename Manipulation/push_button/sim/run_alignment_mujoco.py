#!/usr/bin/env python3
"""到着ずれを模擬し、1回のRGB-D再計測→FK追従→物理押下を検証する。"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from alignment import (AlignmentController, CorrectionCadence, CorrectionTiming,
                       ContinuousNormalController, JointCommandFilter, NormalMotionController,
                       SingleCorrectionTiming, TargetTracker, add_correction_arguments,
                       align_fixed_target, normal_completion_tolerance, press_compensation_allowance,
                       press_overtravel_allowance, run_continuous_normal)
from kinematics import ArmKinematics
from trajectory import DEFAULT_BUTTON_STROKE_M, interpolate
from vision import CameraIntrinsics, estimate_button_target, validate_reachable_target
from run_mujoco import (BUTTON_HALF_DEPTH, BUTTON_VERTICAL_SPACING, DEFAULT_TIP_OFFSET,
                        TIP_RADIUS, build_model)
from run_vision_mujoco import oracle_bbox


class Experiment:
    def __init__(self, args):
        self.args = args
        self.timing = CorrectionTiming.from_args(args)
        self.single_timing = SingleCorrectionTiming.from_args(args)
        self.single = getattr(args, "alignment_mode", "single") == "single"
        self.smooth = self.single and getattr(args, "normal_motion", "smooth") == "smooth"
        self.continuous_result = None
        self.measurement_count = 0
        self.fk_alignment_log = []
        self.model, self.stand = build_model(
            Path(args.model).resolve(), 0.36, -0.25, 1.0, args.stroke,
            DEFAULT_TIP_OFFSET, args.fixed_base, rgbd_camera=True,
            button_direction=args.button_direction, panel_offset=tuple(args.offset),
            panel_yaw_deg=args.yaw_deg,
            rgbd_position=f"-0.10 -0.42 {1.45 if args.button_direction == 'up' else 1.31}",
            rgbd_fovy=45.0, elevator_front=getattr(args, "elevator_front", False))
        self.data = mujoco.MjData(self.model)
        if args.fixed_base:
            self.data.qpos[:29] = self.stand
        else:
            self.data.qpos[:7] = (0, 0, 0.79, 1, 0, 0, 0)
            self.data.qpos[7:36] = self.stand
        self.data.ctrl[:] = self.stand
        mujoco.mj_forward(self.model, self.data)
        self.kin = ArmKinematics(self.model)
        self.kin.update(self.data.qpos)
        self.home = self.data.qpos[self.kin.qadr].copy()
        # セグメンテーションIDや深度を境界で混ぜない。GPUのMSAAによって
        # 別のボタンの境界画素が対象IDになることがある。鑑賞用の描画設定は戻す。
        samples = self.model.vis.quality.offsamples
        self.model.vis.quality.offsamples = 0
        try:
            self.sensor = mujoco.Renderer(self.model, height=480, width=640)
        finally:
            self.model.vis.quality.offsamples = samples
        video_out = getattr(args, "video_out", None)
        self.demo = mujoco.Renderer(self.model, height=480, width=640) if args.gif_out or video_out else None
        self.video_writer = None
        self.video_path = Path(video_out) if video_out else None
        self.video_poster_saved = False
        self.video_frame_count = 0
        if self.video_path is not None:
            import cv2
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            self.video_writer = cv2.VideoWriter(str(self.video_path), cv2.VideoWriter_fourcc(*"VP80"),
                                               args.gif_fps, (960, 540))
            if not self.video_writer.isOpened():
                self.video_writer.release()
                self.demo.close()
                self.sensor.close()
                raise ValueError("WebM動画の出力を開けません。OpenCVのVP8 encoderが必要です")
        self.detector = None
        if args.weights:
            import torch
            torch.set_num_threads(1)
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "Perception"))
            from common.detector.yolo_detector import YoloDetector
            self.detector = YoloDetector(args.weights, classes=["button"],
                                         confidence_threshold=0.25, device="cpu")
        self.tracker = TargetTracker()
        self.controller = AlignmentController(clearance=args.clearance)
        self.frames = []
        self.next_frame = 0.0
        self.phase = "settle"
        self.last_error = None
        self.last_rgb = None
        self.last_bbox = None
        self.observation = None
        self.log = []
        self.normal_log = []
        self.normal_trace = []
        self.next_normal_sample = 0.0
        self.normal_phase_times = {}
        self.stroke_adr = self.model.joint(f"button_slide_{args.button_direction}").qposadr[0]
        other = "down" if args.button_direction == "up" else "up"
        self.other_adr = self.model.joint(f"button_slide_{other}").qposadr[0]
        self.face_id = self.model.geom(f"button_face_{args.button_direction}").id
        self.max_stroke = self.max_other = self.max_drift = self.max_tilt = 0.0
        self.base_drift_origin = np.zeros(2)
        self.min_height = float(self.data.body("pelvis").xpos[2])

    def before_approach(self):
        """既定はボタン前から開始。歩行を接続する実験はこのフックを使う。"""

    def step_physics(self):
        mujoco.mj_step(self.model, self.data)

    def track_base_motion(self):
        if not self.args.fixed_base:
            self.max_drift = max(self.max_drift, float(np.linalg.norm(
                self.data.qpos[:2] - self.base_drift_origin)))

    def decorate_video_frame(self, canvas):
        """歩行を接続する実験用の追加表示。"""

    def control_status(self):
        return "Fixed pelvis" if self.args.fixed_base else "Free standing / position servos"

    def tick(self):
        self.step_physics()
        self.max_stroke = max(self.max_stroke, float(self.data.qpos[self.stroke_adr]))
        self.max_other = max(self.max_other, float(self.data.qpos[self.other_adr]))
        self.min_height = min(self.min_height, float(self.data.body("pelvis").xpos[2]))
        self.track_base_motion()
        tilt = math.degrees(math.acos(float(np.clip(self.data.body("pelvis").xmat[8], -1, 1))))
        self.max_tilt = max(self.max_tilt, tilt)
        if self.max_stroke >= 0.9 * self.args.stroke:
            self.model.geom_rgba[self.face_id] = (0.95, 0.81, 0.49, 1)
        if self.min_height < 0.65 or tilt > 20 or self.max_drift > 0.10:
            raise ValueError("模擬機体の立位が不安定です")
        for contact in self.data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if contact.dist < -0.001 and pair.intersection(self.kin.arm_geoms):
                if pair == {self.kin.tip_id, self.face_id} and self.phase in ("press", "contact", "retract"):
                    continue
                raise ValueError(f"実行中に意図しない腕の接触があります: {sorted(pair)}")
        if self.demo is not None and self.data.time >= self.next_frame:
            self.capture_frame()
            self.next_frame += 1 / self.args.gif_fps
        if self.phase in ("contact", "press") and self.data.time >= self.next_normal_sample:
            self.normal_trace.append({"time_s": float(self.data.time), "phase": self.phase,
                                      "tip_xyz_m": self.data.site_xpos[self.kin.site_id].tolist(),
                                      "joint_command_rad": self.data.ctrl[self.kin.actuator].tolist()})
            self.next_normal_sample = float(self.data.time) + 0.02

    def hold(self, duration):
        for _ in range(math.ceil(duration / self.model.opt.timestep)):
            self.tick()

    def capture_frame(self):
        from PIL import Image, ImageDraw, ImageFont
        self.demo.update_scene(self.data, camera="demo_camera")
        frame = Image.fromarray(self.demo.render().copy())
        canvas = Image.new("RGB", (960, 540), (16, 21, 28))
        canvas.paste(frame, (0, 60))
        draw = ImageDraw.Draw(canvas)
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        font = ImageFont.truetype(font_path, 17)
        small = ImageFont.truetype(font_path, 14)
        dx, dy, dz = self.args.offset
        title = getattr(self.args, "demo_title", "G1 / RGB-D alignment")
        draw.text((15, 8), f"{title} / {self.args.button_direction.upper()} button", font=font, fill="white")
        draw.text((15, 33), f"Arrival offset: X {dx*1000:+.0f} / Y {dy*1000:+.0f} / Z {dz*1000:+.0f} mm   yaw {self.args.yaw_deg:+.0f} deg", font=small, fill="#a8c8ef")
        draw.text((655, 70), "RGB-D / last detection", font=font, fill="white")
        if self.last_rgb is not None:
            view = Image.fromarray(self.last_rgb)
            overlay = ImageDraw.Draw(view)
            if self.last_bbox is not None:
                overlay.rectangle(self.last_bbox, outline="#49ff85", width=3)
            canvas.paste(view.resize((320, 240)), (640, 102))
        lines = [f"Phase: {self.phase}", f"Time: {self.data.time:.1f} s",
                 "Normal motion: continuous" if self.smooth else "Normal motion: stepped",
                 f"Remeasurements: {max(0, self.measurement_count-1)}",
                 "Tip error: --" if self.last_error is None else f"Tip error: {self.last_error*1000:.1f} mm",
                 f"Button stroke: {self.max_stroke*1000:.1f} / {self.args.stroke*1000:.1f} mm",
                 "Detector: YOLO" if self.detector else "Detector: segmentation (test)",
                 self.control_status()]
        for i, line in enumerate(lines):
            draw.text((650, 355 + i * 24), line, font=small,
                      fill="#49ff85" if "error" in line else "white")
        self.decorate_video_frame(canvas)
        if self.video_writer is not None:
            import cv2
            self.video_writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
            self.video_frame_count += 1
            if not self.video_poster_saved and self.phase == "press" and self.max_stroke >= 0.9 * self.args.stroke:
                canvas.save(self.video_path.with_suffix(".png"))
                self.video_poster_saved = True
        if self.args.gif_out:
            self.frames.append(canvas.quantize(colors=192))

    def observe(self):
        self.measurement_count += 1
        self.tracker.begin_measurement()
        for _ in range(20):
            self.hold(1 / 15)
            self.sensor.update_scene(self.data, camera="button_rgbd")
            rgb = self.sensor.render().copy() if self.detector is not None or self.demo is not None else None
            self.sensor.enable_depth_rendering()
            depth = self.sensor.render().copy()
            self.sensor.disable_depth_rendering()
            if self.detector is None:
                self.sensor.enable_segmentation_rendering()
                segmentation = self.sensor.render().copy()
                self.sensor.disable_segmentation_rendering()
                boxes = [oracle_bbox(segmentation, self.face_id)]
            else:
                boxes = [d.bbox for d in self.detector.detect(rgb[:, :, ::-1].copy())]
            camera = self.model.camera("button_rgbd").id
            transform = np.eye(4)
            transform[:3, :3] = self.data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1])
            transform[:3, 3] = self.data.cam_xpos[camera]
            focal = 480 / (2 * math.tan(math.radians(self.model.cam_fovy[camera]) / 2))
            intr = CameraIntrinsics(focal, focal, 320, 240)
            candidates = []
            for box in boxes:
                try:
                    target = estimate_button_target(depth, box, intr, transform,
                                                     expected_face_xyz=self.tracker.previous)
                    validate_reachable_target(target)
                    if self.tracker.initial is None:
                        expected_z = 1.0 if self.args.button_direction == "up" else 1.0 - BUTTON_VERTICAL_SPACING
                        if abs(target.face_xyz[2] - expected_z) >= 0.07:
                            continue
                    elif np.linalg.norm(np.array(target.face_xyz) - self.tracker.previous) > self.tracker.association_m:
                        continue
                except ValueError:
                    continue
                candidates.append(target)
                self.last_bbox = box
            self.last_rgb = rgb
            if not candidates:
                self.tracker.samples.clear()
                continue
            observation = self.tracker.update(candidates, float(self.data.time))
            if observation is not None:
                self.observation = observation
                return observation
        raise ValueError("RGB-D計測が安定しません")

    def move(self, xyz, normal, duration, allow_contact=False, settle_s=0.1, cadence=None):
        self.kin.update(self.data.qpos)
        solution = self.kin.solve(np.asarray(xyz), np.asarray(normal))
        return self.move_joints(solution.joint_angles, duration, allow_contact, settle_s, cadence)

    def move_joints(self, goal, duration, allow_contact=False, settle_s=0.1, cadence=None):
        self.kin.update(self.data.qpos)
        face = None if self.observation is None else np.array(self.observation.target.face_xyz)
        normal = None if self.observation is None else np.array(self.observation.target.press_direction)
        self.kin.validate_path(goal, face, normal, allow_contact, {self.face_id})
        start = self.data.qpos[self.kin.qadr].copy()
        duration = max(duration, float(np.max(np.abs(goal - start))) * 1.5 / 0.8)
        started_at = cadence.start() if cadence is not None else float(self.data.time)
        for q in interpolate(tuple(start), tuple(goal), duration, 1 / self.model.opt.timestep):
            self.data.ctrl[self.kin.actuator] = q
            self.tick()
        self.hold(settle_s)
        return started_at

    def move_normal(self, face, normal, depth, bias, allow_contact):
        phase_started_at = float(self.data.time)
        self.kin.update(self.data.qpos)
        start_depth = float((self.kin.tip_position()-face) @ normal + TIP_RADIUS)
        command = face + (start_depth - TIP_RADIUS) * normal + bias
        servo = NormalMotionController(face, normal, TIP_RADIUS, depth, command, single=self.single)
        cadence = None if self.single else CorrectionCadence(self.timing, lambda: float(self.data.time), self.hold)
        while True:
            if cadence is not None:
                cadence.wait_for_next()
            elif self.data.time - phase_started_at >= self.single_timing.phase_timeout_s:
                raise ValueError("法線方向の追従が制限時間内に収束しません")
            self.kin.update(self.data.qpos)
            goal = servo.next_goal(self.kin.tip_position())
            if goal is None:
                break
            started_at = self.move(goal, normal,
                                   self.single_timing.feedback_move_s if self.single else self.timing.move_s,
                                   allow_contact,
                                   self.single_timing.feedback_settle_s if self.single else self.timing.settle_s, cadence)
            self.normal_log.append({"phase": self.phase, "motion_started_s": started_at,
                                    "tip_goal_xyz_m": goal.tolist()})
        self.normal_phase_times[self.phase] = float(self.data.time) - phase_started_at

    def move_smooth(self, face, normal, bias):
        self.kin.update(self.data.qpos)
        start_depth = float((self.kin.tip_position() - face) @ normal + TIP_RADIUS)
        servo = ContinuousNormalController(face, normal, TIP_RADIUS, self.args.stroke,
                                            face + (start_depth-TIP_RADIUS)*normal + bias)
        joint_filter = JointCommandFilter(self.data.ctrl[self.kin.actuator], servo.settings)

        def read_tip():
            self.kin.update(self.data.qpos)
            return self.kin.tip_position()

        def send_goal(xyz, dt, phase):
            self.phase = phase
            solution = self.kin.solve(xyz, normal, position_tolerance_m=0.00002)
            goal = joint_filter.next_goal(solution.joint_angles, dt)
            self.kin.validate_path(goal, face, normal, phase == "press", {self.face_id})
            self.data.ctrl[self.kin.actuator] = goal
            return goal

        self.continuous_result = run_continuous_normal(
            servo, lambda: float(self.data.time), self.hold, read_tip, send_goal,
            self.single_timing.phase_timeout_s, self.normal_log.append)
        self.normal_phase_times["continuous_approach_and_press"] = self.continuous_result["duration_s"]

    def run(self):
        error = None
        final_alignment = None
        truth_error = None
        initial_error = None
        alignment_duration = None
        try:
            self.before_approach()
            self.hold(0.5)
            self.phase = "re-detect"
            first = self.observe()
            height = 1.0 if self.args.button_direction == "up" else 1.0 - BUTTON_VERTICAL_SPACING
            nominal_face = np.array([0.36 - BUTTON_HALF_DEPTH, -0.25, height])
            nominal_wait = nominal_face - (TIP_RADIUS + 0.09) * np.array([1, 0, 0])
            self.phase = "approach"
            # 既定の1回再計測では初回のRGB-Dで到着ずれを吸収して5cm手前へ運ぶ。
            wait_point = (np.array(first.target.face_xyz)
                          - (TIP_RADIUS + self.args.clearance) * np.array(first.target.press_direction)) if self.single else nominal_wait
            if self.single:
                transit = np.array(first.target.face_xyz) - (TIP_RADIUS + 0.09) * np.array(first.target.press_direction)
                self.move(transit, np.array(first.target.press_direction), 3.0, settle_s=0.1)
            self.move(wait_point, np.array(first.target.press_direction), 1.0 if self.single else 3.0,
                      settle_s=self.single_timing.capture_settle_s if self.single else self.timing.settle_s)
            if any(self.args.drift_after_approach):
                panel = self.model.body("elevator_panel_frame").id
                self.model.body_pos[panel] += np.array(self.args.drift_after_approach)
                mujoco.mj_forward(self.model, self.data)
            self.phase = "align"
            if self.single:
                observation = self.observe()

                def read_tip():
                    self.kin.update(self.data.qpos)
                    return self.kin.tip_position()

                self.controller, result = align_fixed_target(
                    observation, TIP_RADIUS, self.args.clearance, self.single_timing,
                    lambda: float(self.data.time), self.hold, read_tip,
                    lambda goal, duration, settle: self.move(
                        goal, np.array(observation.target.press_direction), duration, settle_s=settle))
                initial_error, final_alignment = result["initial_error_m"], result["final_error_m"]
                self.last_error = final_alignment
                alignment_duration = result["duration_s"]
                self.fk_alignment_log = result["fk_feedback_steps"]
                self.log.append({"sequence": observation.sequence, "time_s": observation.captured_at,
                                 "captured_at_s": observation.captured_at, "motion_started_s": None,
                                 "error_m": initial_error, "final_error_m": final_alignment,
                                 "measured_face_xyz_m": list(observation.target.face_xyz)})
                truth = self.data.geom_xpos[self.face_id] - BUTTON_HALF_DEPTH * self.data.geom_xmat[self.face_id].reshape(3, 3)[:, 2]
                truth_error = float(np.linalg.norm(np.array(observation.target.face_xyz) - truth))
            cadence = CorrectionCadence(self.timing, lambda: float(self.data.time), self.hold) if not self.single else None
            while not self.single:
                cadence.wait_for_next()
                observation = self.observe()
                self.kin.update(self.data.qpos)
                step = self.controller.next_step(observation, self.kin.tip_position(), float(self.data.time))
                if initial_error is None:
                    initial_error = step.error_m
                self.last_error = step.error_m
                entry = {"sequence": observation.sequence, "time_s": float(self.data.time),
                         "captured_at_s": observation.captured_at, "motion_started_s": None,
                         "error_m": step.error_m, "lateral_error_m": step.lateral_error_m,
                         "normal_error_m": step.normal_error_m,
                         "measured_face_xyz_m": list(observation.target.face_xyz)}
                self.log.append(entry)
                if step.converged:
                    cadence.start()
                    final_alignment = step.error_m
                    alignment_duration = float(self.data.time) - cadence.phase_started_at
                    # 正解座標は評価にのみ使い、制御器には渡さない。
                    truth = self.data.geom_xpos[self.face_id] - BUTTON_HALF_DEPTH * self.data.geom_xmat[self.face_id].reshape(3, 3)[:, 2]
                    truth_error = float(np.linalg.norm(np.array(observation.target.face_xyz) - truth))
                    break
                if step.error_m > self.controller.tolerance_m:
                    entry["motion_started_s"] = self.move(
                        step.tip_goal, np.array(observation.target.press_direction), self.timing.move_s,
                        settle_s=self.timing.settle_s, cadence=cadence)
                else:
                    cadence.start()
            target = self.observation.target
            face, normal = np.array(target.face_xyz), np.array(target.press_direction)
            bias = self.controller.command_xyz - (face - (TIP_RADIUS + self.args.clearance) * normal)
            bias -= (bias @ normal) * normal
            if self.smooth:
                self.move_smooth(face, normal, bias)
            else:
                self.phase = "contact"
                self.move_normal(face, normal, -0.003, bias, False)
                self.phase = "press"
                self.move_normal(face, normal, self.args.stroke, bias, True)
            self.hold(0.3)
            self.phase = "retract"
            self.move(face - (TIP_RADIUS + self.args.clearance) * normal + bias, normal, 2.0, True)
            self.phase = "home"
            self.move_joints(self.home, 3.0)
            self.phase = "done"
            self.hold(0.4)
        except (ValueError, RuntimeError) as exc:
            error = str(exc)
        finally:
            self.sensor.close()
            if self.demo is not None:
                try:
                    self.capture_frame()
                finally:
                    self.demo.close()
                    if self.video_writer is not None:
                        self.video_writer.release()
                if self.frames:
                    path = Path(self.args.gif_out)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    # GIFは10ms単位。8fpsの125msを毎回120msへ切り捨てない。
                    durations = [10 * (round((i+1)*100/self.args.gif_fps)
                                       - round(i*100/self.args.gif_fps))
                                 for i in range(len(self.frames))]
                    self.frames[0].save(path, save_all=True, append_images=self.frames[1:],
                                        duration=durations, loop=0)
                    self.frames[len(self.frames)//2].convert("RGB").save(path.with_suffix(".png"))
        starts = {"alignment_decision": [s["time_s"] for s in self.log],
                  "alignment_motion": [s["motion_started_s"] for s in self.log
                                       if s["motion_started_s"] is not None]}
        starts.update({phase: [s["motion_started_s"] for s in self.normal_log if s["phase"] == phase]
                       for phase in ("contact", "press")})
        minimum_intervals = {phase: float(np.min(np.diff(times))) if len(times) > 1 else None
                             for phase, times in starts.items()}
        timing_valid = ((self.measurement_count == 2 and len(self.log) == 1) if self.single else
                        all(value is None or value >= self.timing.interval_s - 1e-8
                            for value in minimum_intervals.values()))
        success = (error is None and timing_valid and final_alignment is not None
                   and final_alignment <= self.controller.tolerance_m
                   and 0.9 * self.args.stroke <= self.max_stroke <= 1.1 * self.args.stroke
                   and self.max_other < 0.1 * self.args.stroke)
        return {"success": success, "error": error, "button_direction": self.args.button_direction,
                "offset_m": list(self.args.offset), "yaw_deg": self.args.yaw_deg,
                "stroke_m": self.args.stroke,
                "press_limits": {"overtravel_allowance_m": press_overtravel_allowance(self.args.stroke),
                                 "compensation_allowance_m": press_compensation_allowance(self.args.stroke)},
                "drift_after_approach_m": list(self.args.drift_after_approach),
                "detector": "yolo" if self.detector else "segmentation-oracle",
                "alignment_mode": "single" if self.single else "iterative",
                "normal_motion": "smooth" if self.smooth else "stepped",
                "normal_motion_trace": self.normal_trace,
                "continuous_normal_result": self.continuous_result,
                "measurement_count": self.measurement_count,
                "remeasurement_count": max(0, self.measurement_count-1),
                "single_correction_timing": vars(self.single_timing),
                "fk_alignment_steps": self.fk_alignment_log,
                "correction_timing": vars(self.timing), "timing_valid": timing_valid,
                "press_completion_tolerance_m": normal_completion_tolerance(self.args.stroke, self.single),
                "minimum_correction_intervals_s": minimum_intervals,
                "elapsed_time_s": float(self.data.time), "alignment_duration_s": alignment_duration,
                "video_frame_count": self.video_frame_count, "video_fps": self.args.gif_fps,
                "normal_phase_duration_s": self.normal_phase_times, "normal_steps": self.normal_log,
                "base_fixed": self.args.fixed_base, "initial_alignment_error_m": initial_error,
                "final_alignment_error_m": final_alignment, "measurement_truth_error_m": truth_error,
                "max_stroke_m": self.max_stroke, "max_other_stroke_m": self.max_other,
                "max_base_drift_m": self.max_drift, "min_pelvis_height_m": self.min_height,
                "max_tilt_deg": self.max_tilt, "alignment_steps": self.log}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--weights", help="指定時は実際のYOLO推論を使う。省略時は検出器を切り分ける")
    parser.add_argument("--offset", type=float, nargs=3, default=[0.03, -0.03, 0.0], metavar=("X", "Y", "Z"))
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--drift-after-approach", type=float, nargs=3, default=[0, 0, 0],
                        help="待機姿勢に移動した後に追加するパネルの移動 [m]")
    parser.add_argument("--button-direction", choices=("up", "down"), default="up")
    parser.add_argument("--stroke", type=float, default=DEFAULT_BUTTON_STROKE_M,
                        help="押込量 [m] (既定: 0.0015)")
    parser.add_argument("--clearance", type=float, default=0.05)
    parser.add_argument("--fixed-base", action="store_true")
    parser.add_argument("--gif-out")
    parser.add_argument("--video-out", help="VP8/WebM動画の保存先 (.webm)。OpenCVが必要")
    parser.add_argument("--gif-fps", "--video-fps", type=int, default=10)
    parser.add_argument("--report-out", required=True)
    parser.add_argument("--suite", action="store_true", help="上下ボタン・前後左右±1/3/5cm・向き±5度")
    add_correction_arguments(parser)
    args = parser.parse_args()
    try:
        CorrectionTiming.from_args(args)
        SingleCorrectionTiming.from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.video_out and Path(args.video_out).suffix.lower() != ".webm":
        parser.error("--video-out は .webm で指定してください")
    if (not np.all(np.isfinite([*args.offset, *args.drift_after_approach, args.yaw_deg, args.stroke, args.clearance]))
            or max(abs(v) for v in args.offset) > 0.05 or abs(args.yaw_deg) > 5
            or max(abs(v) for v in args.drift_after_approach) > 0.01
            or not 0.0015 <= args.stroke <= 0.015 or not 0.02 <= args.clearance <= 0.05
            or not 1 <= args.gif_fps <= 30):
        parser.error("ずれは±5cm、向きは±5度、strokeは1.5～15mm、clearanceは2～5cmで指定してください")
    results = []
    cases = [(args.button_direction, args.offset, args.yaw_deg, args.drift_after_approach)]
    if args.suite:
        shifts = [([0, 0, 0], 0)]
        for axis in (0, 1):
            for distance in (-0.05, -0.03, -0.01, 0.01, 0.03, 0.05):
                offset = [0, 0, 0]
                offset[axis] = distance
                shifts.append((offset, 0))
        shifts += [([0, 0, 0], -5), ([0, 0, 0], 5), ([0.03, -0.03, 0], 3)]
        cases = [(button, offset, yaw, [0, 0, 0]) for button in ("up", "down") for offset, yaw in shifts]
        cases += [(button, [0.03, -0.03, 0], 0, [0, 0.01, 0.01]) for button in ("up", "down")]
    for button, offset, yaw, drift in cases:
        case = SimpleNamespace(**vars(args))
        case.button_direction, case.offset, case.yaw_deg = button, offset, yaw
        case.drift_after_approach = drift
        if args.suite:
            case.gif_out = None
            case.video_out = None
        result = Experiment(case).run()
        results.append(result)
        print(f"{'PASS' if result['success'] else 'FAIL'} {button} offset={offset} yaw={yaw} "
              f"stroke={result['max_stroke_m']*1000:.1f} mm align={result['final_alignment_error_m']} "
              f"time={result['elapsed_time_s']:.1f} s timing={result['timing_valid']} "
              f"error={result['error']}", flush=True)
        report = {"schema": "g1-button-alignment-test-v1", "mujoco_version": mujoco.__version__,
                  "passed": sum(r['success'] for r in results), "total": len(results), "results": results}
        path = Path(args.report_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if all(r["success"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
