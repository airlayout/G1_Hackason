#!/usr/bin/env python3
"""MuJoCoで経由点に沿って歩き、静止を確認して1回再計測＋ボタン押下を行う。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from run_alignment_mujoco import Experiment
from alignment import add_correction_arguments
from trajectory import DEFAULT_BUTTON_STROKE_M, validate_button_stroke
from walking import (DEFAULT_ASSETS, WalkingPolicy, Waypoint, support_margin,
                     velocity_to_waypoint, wrap_angle, yaw_of)


def load_route(path=None):
    raw = json.loads(Path(path).read_text()) if path else [dict(x_m=-0.6, y_m=0), dict(x_m=0, y_m=0)]
    if not isinstance(raw, list) or not 1 <= len(raw) <= 30 or not all(isinstance(p, dict) for p in raw):
        raise ValueError("経由点は座標オブジェクト1〜30個の配列が必要です")
    route = [Waypoint(**p) for p in raw]
    goal = route[-1]
    if abs(goal.x_m) > 0.05 or abs(goal.y_m) > 0.05 or abs(goal.yaw_deg) > 5:
        raise ValueError("最後の経由点はボタン前の原点±5cm・正面±5度に設定してください")
    return route


class WalkAndPressExperiment(Experiment):
    def __init__(self, args):
        self.route = load_route(args.route_json)
        super().__init__(args)
        self.walking = False
        try:
            self.walker = WalkingPolicy(self.model, self.data, args.walk_assets)
        except Exception:
            self.sensor.close()
            if self.demo is not None:
                self.demo.close()
            if self.video_writer is not None:
                self.video_writer.release()
            raise
        self.attach_rgbd_camera()
        x, y, yaw_deg = args.start_pose
        angle = math.radians(yaw_deg)/2
        # 初期配置の1回だけ。歩行・引き継ぎ・押下中に胴体の状態を書き換えない。
        self.data.qpos[:7] = (x, y, 0.793, math.cos(angle), 0, 0, math.sin(angle))
        self.data.qpos[self.walker.qadr] = self.walker.defaults
        mujoco.mj_forward(self.model, self.data)
        self.walking = True
        self.standing_targets = None
        self.standing_yaw = None
        self.initial_xy = self.data.qpos[:2].copy()
        self.navigation_trace = []
        self.navigation_result = None
        self.navigation_completed = False
        self.handoff_support = None
        self.max_standing_yaw_correction = 0.0
        self.arrival_pose = None
        self.pose_at_press = None
        self.phase_events = []
        self.current_waypoint = 0
        self.close_camera_pos = self.model.cam_pos[self.model.camera("demo_camera").id].copy()
        self.close_camera_fovy = float(self.model.cam_fovy[self.model.camera("demo_camera").id])
        self.model.cam_pos[self.model.camera("demo_camera").id] = (-2.6, -2.0, 2.0)
        self.model.cam_fovy[self.model.camera("demo_camera").id] = 75

    def attach_rgbd_camera(self):
        """仮想RGB-Dを胴体に固定する。ずれたボタンを自動追尾するカメラにはしない。"""
        camera = self.model.camera("button_rgbd").id
        body = self.model.body("torso_link").id
        position = self.model.cam_pos[camera].copy()
        nominal_face = np.array([0.36, -0.25, 1.0 if self.args.button_direction == "up" else 0.86])
        z_axis = position-nominal_face
        z_axis /= np.linalg.norm(z_axis)
        x_axis = np.cross([0, 0, 1], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        rotation = np.column_stack((x_axis, np.cross(z_axis, x_axis), z_axis))
        body_rotation = self.data.xmat[body].reshape(3, 3)
        self.model.cam_bodyid[camera] = body
        self.model.cam_mode[camera] = mujoco.mjtCamLight.mjCAMLIGHT_FIXED
        self.model.cam_pos[camera] = body_rotation.T@(position-self.data.xpos[body])
        mujoco.mju_mat2Quat(self.model.cam_quat[camera], (body_rotation.T@rotation).reshape(-1))

    def step_physics(self):
        if self.walking:
            self.walker.step()
        else:
            if self.standing_targets is not None:
                # 接地した脚の股関節yawで静止時の向きを保持する。胴体への外力は使わない。
                target = self.standing_targets.copy()
                error = wrap_angle(yaw_of(self.data.qpos[3:7])-self.standing_yaw)
                correction = float(np.clip(0.8*error, -0.10, 0.10))
                self.max_standing_yaw_correction = max(self.max_standing_yaw_correction, abs(correction))
                target[[2, 8]] += correction
                self.data.ctrl[self.walker.actuator] = target
            super().step_physics()

    def track_base_motion(self):
        # 自由歩行の移動距離と、静止後の押下中のドリフトを別々に記録する。
        if not self.walking and self.navigation_completed:
            super().track_base_motion()

    def navigation_sample(self):
        if self.navigation_trace and self.data.time-self.navigation_trace[-1]["time_s"] < 0.02-1e-9:
            return
        self.navigation_trace.append({"time_s": float(self.data.time), "phase": self.phase,
            "waypoint_index": self.current_waypoint, "pelvis_xyz_m": self.data.qpos[:3].tolist(),
            "yaw_deg": math.degrees(yaw_of(self.data.qpos[3:7])),
            "command_m_s_rad_s": self.walker.command.tolist(),
            "base_speed_m_s": float(np.linalg.norm(self.data.qvel[:3])),
            "base_angular_speed_rad_s": float(np.linalg.norm(self.data.qvel[3:6]))})

    def event(self, phase):
        self.phase = phase
        self.phase_events.append({"phase": phase, "time_s": float(self.data.time)})

    def support_state(self):
        """両足接地と支持面内の重心を、MuJoCoの接触状態で確認する。"""
        floor = self.model.geom("floor").id
        feet = {self.model.body(f"{side}_ankle_roll_link").id: side for side in ("left", "right")}
        loads = {"left": 0.0, "right": 0.0}
        points = []
        for index, contact in enumerate(self.data.contact):
            pair = {int(contact.geom1), int(contact.geom2)}
            if floor not in pair:
                continue
            geom = next(g for g in pair if g != floor)
            side = feet.get(int(self.model.geom_bodyid[geom]))
            if side is None:
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            loads[side] += float(max(0, force[0]))
            if force[0] > 1:
                points.append(contact.pos[:2].copy())
        com = self.data.subtree_com[self.model.body("pelvis").id, :2]
        margin = support_margin(points, com)
        margin = None if margin is None else float(margin)
        return {"foot_load_n": loads, "com_support_margin_m": margin,
                "double_support": min(loads.values()) >= 20 and margin is not None and margin >= 0.005}

    def double_support(self):
        return self.support_state()["double_support"]

    def control_status(self):
        return "Legs: RL torque / arms: position" if self.walking else "Legs: stance hold / arms: position"

    def decorate_video_frame(self, canvas):
        from PIL import ImageDraw, ImageFont
        draw = ImageDraw.Draw(canvas)
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
        draw.rectangle((640, 70, 959, 96), fill=(16, 21, 28))
        draw.text((650, 72), "Route / simulated odometry" if self.last_rgb is None else "RGB-D / body-mounted camera",
                  font=font, fill="white")
        if self.last_rgb is not None:
            return
        draw.rectangle((650, 102, 949, 335), fill=(30, 39, 48))
        points = [self.initial_xy, *[[p.x_m, p.y_m] for p in self.route]]
        low, high = np.min(points, axis=0)-0.15, np.max(points, axis=0)+0.15
        scale = min(275/(high[0]-low[0]), 195/(high[1]-low[1]))

        def pixel(xy):
            p = (np.asarray(xy)-low)*scale
            return (660+float(p[0]), 325-float(p[1]))

        draw.line([pixel(p) for p in points], fill="#a8c8ef", width=2)
        if len(self.navigation_trace) > 1:
            draw.line([pixel(t["pelvis_xyz_m"][:2]) for t in self.navigation_trace], fill="#49ff85", width=2)
        for index, point in enumerate(self.route):
            x, y = pixel([point.x_m, point.y_m])
            draw.ellipse((x-4, y-4, x+4, y+4), fill="#a8c8ef")
            draw.text((x-5, y-22), str(index+1), font=font, fill="white")
        x, y = pixel(self.data.qpos[:2])
        draw.ellipse((x-5, y-5, x+5, y+5), fill="#49ff85")
        draw.text((657, 107), "Blue: route   Green: robot", font=font, fill="white")

    def before_approach(self):
        self.event("walk-settle")
        self.walker.set_command([0, 0, 0])
        self.hold(0.8)
        started = float(self.data.time)
        for index, goal in enumerate(self.route):
            self.current_waypoint = index
            self.event(f"walk {index+1}/{len(self.route)}")
            while True:
                yaw = yaw_of(self.data.qpos[3:7])
                distance = np.linalg.norm(self.data.qpos[:2]-[goal.x_m, goal.y_m])
                if distance <= 0.025 and abs(wrap_angle(math.radians(goal.yaw_deg)-yaw)) <= math.radians(3):
                    break
                if self.data.time-started >= self.args.navigation_timeout:
                    raise ValueError("歩行が制限時間内に経由点へ到着しません")
                self.walker.set_command(velocity_to_waypoint(self.data.qpos[:2], yaw, goal))
                self.hold(0.02)
                self.navigation_sample()
        self.arrival_pose = self.data.qpos[:7].copy()
        self.event("stop-walking")
        self.walker.set_command([0, 0, 0])
        stop_started = float(self.data.time)
        while True:
            self.hold(0.002)
            self.navigation_sample()
            if (self.double_support() and np.linalg.norm(self.data.qvel[:3]) <= 0.20
                    and np.linalg.norm(self.data.qvel[3:6]) <= 0.9):
                break
            if self.data.time-stop_started >= 5:
                raise ValueError("歩行停止後に両足支持へ引き継げません")
        self.handoff_support = self.support_state()
        self.walker.restore_position_control()
        self.standing_targets = self.data.ctrl[self.walker.actuator].copy()
        self.standing_yaw = yaw_of(self.data.qpos[3:7])
        self.walking = False
        self.event("settle-standing")
        camera_start = self.model.cam_pos[self.model.camera("demo_camera").id].copy()
        for i in range(601):
            blend = min(1, i/600)
            self.model.cam_pos[self.model.camera("demo_camera").id] = (1-blend)*camera_start+blend*self.close_camera_pos
            self.model.cam_fovy[self.model.camera("demo_camera").id] = (1-blend)*75+blend*self.close_camera_fovy
            self.tick()
        settled = 0.0
        settling_start = float(self.data.time)
        while settled < 0.5:
            if self.data.time-settling_start >= 5:
                raise ValueError("歩行停止後の機体が静止しません")
            self.hold(0.02)
            self.navigation_sample()
            speed = np.linalg.norm(self.data.qvel[:3])
            angular_speed = np.linalg.norm(self.data.qvel[3:6])
            leg_speed = np.max(np.abs(self.data.qvel[self.walker.dadr]))
            settled = (settled+0.02 if speed <= 0.02 and angular_speed <= 0.05
                       and leg_speed <= 0.05 and self.double_support() else 0.0)
        goal = self.route[-1]
        arrival_error = float(np.linalg.norm(self.data.qpos[:2]-[goal.x_m, goal.y_m]))
        yaw_error = abs(wrap_angle(math.radians(goal.yaw_deg)-yaw_of(self.data.qpos[3:7])))
        if arrival_error > 0.08 or yaw_error > math.radians(5):
            raise ValueError("歩行停止後の姿勢がボタン前の許容範囲外です")
        self.navigation_completed = True
        self.pose_at_press = self.data.qpos[:7].copy()
        self.base_drift_origin = self.data.qpos[:2].copy()
        self.navigation_result = {"success": True, "arrival_error_m": arrival_error,
            "arrival_yaw_error_deg": math.degrees(yaw_error),
            "walk_and_stop_duration_s": float(self.data.time),
            "travel_distance_m": float(np.linalg.norm(self.data.qpos[:2]-self.initial_xy)),
            "base_speed_before_press_m_s": float(np.linalg.norm(self.data.qvel[:3])),
            "base_angular_speed_before_press_rad_s": float(np.linalg.norm(self.data.qvel[3:6])),
            "leg_speed_before_press_rad_s": float(np.max(np.abs(self.data.qvel[self.walker.dadr]))),
            "stationary_window_s": settled, "support_before_press": self.support_state(),
            "peak_policy_action": self.walker.peak_action}
        self.event("navigation-complete")
        print(f"WALK PASS: {self.navigation_result}", flush=True)

    def run(self):
        result = super().run()
        result.update({"mission": "walk-stop-align-press", "navigation": self.navigation_result,
            "navigation_completed": self.navigation_completed,
            "navigation_trace": self.navigation_trace, "phase_events": self.phase_events,
            "start_pose_m_m_deg": self.args.start_pose, "route": [vars(p) for p in self.route],
            "arrival_freejoint_qpos": None if self.arrival_pose is None else self.arrival_pose.tolist(),
            "press_start_freejoint_qpos": None if self.pose_at_press is None else self.pose_at_press.tolist(),
            "walking_policy_assets": str(Path(self.args.walk_assets).resolve()),
            "odometry_source": "mujoco-ground-truth", "body_teleport_after_initialization": False,
            "external_support": False, "nav2_executed": False})
        result["handoff_support"] = self.handoff_support
        result["max_standing_yaw_correction_rad"] = self.max_standing_yaw_correction
        result["success"] = result["success"] and self.navigation_completed
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--walk-assets", default=str(DEFAULT_ASSETS))
    parser.add_argument("--start-pose", type=float, nargs=3, default=[-1.2, 0, 0], metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--route-json", help='経由点の配列。各点は{"x_m":..., "y_m":..., "yaw_deg":...}')
    parser.add_argument("--navigation-timeout", type=float, default=40)
    parser.add_argument("--button-direction", choices=("up", "down"), default="up")
    parser.add_argument("--stroke", type=float, default=DEFAULT_BUTTON_STROKE_M)
    parser.add_argument("--offset", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--yaw-deg", type=float, default=0)
    parser.add_argument("--drift-after-approach", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--weights")
    parser.add_argument("--clearance", type=float, default=0.05)
    parser.add_argument("--video-out")
    parser.add_argument("--video-fps", dest="gif_fps", type=int, default=12)
    parser.add_argument("--report-out", required=True)
    add_correction_arguments(parser)
    args = parser.parse_args()
    args.fixed_base, args.gif_out, args.elevator_front = False, None, True
    args.demo_title = "G1 / walk + button press"
    try:
        validate_button_stroke(args.stroke)
        if (not np.all(np.isfinite([*args.start_pose, *args.offset, *args.drift_after_approach,
                                   args.yaw_deg, args.clearance, args.navigation_timeout]))
                or not 1 <= args.gif_fps <= 30 or not 0.02 <= args.clearance <= 0.05
                or not 1 <= args.navigation_timeout <= 120
                or max(abs(v) for v in args.offset) > 0.05 or abs(args.yaw_deg) > 5
                or max(abs(v) for v in args.drift_after_approach) > 0.01
                or (args.video_out and Path(args.video_out).suffix.lower() != ".webm")):
            raise ValueError("位置・時間・描画または押下パラメータが許容範囲外です")
        result = WalkAndPressExperiment(args).run()
    except (ValueError, OSError, TypeError, KeyError, RuntimeError) as exc:
        parser.error(str(exc))
    path = Path(args.report_out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"schema": "g1-walk-and-press-test-v1", "mujoco_version": mujoco.__version__,
                               "result": result}, indent=2, allow_nan=False)+"\n")
    print(f"{'PASS' if result['success'] else 'FAIL'} mission: stroke={result['max_stroke_m']*1000:.3f}mm "
          f"time={result['elapsed_time_s']:.2f}s error={result['error']}", flush=True)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
