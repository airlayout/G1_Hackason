"""採点環境へのアダプタ。画像の3D推定と連続押下は既存処理、FK/IKは公式URDF。

MuJoCo / Isaac の状態や試行の正解にはアクセスしない。機体設定のみ J1-gen と共用する。
"""

from __future__ import annotations

from collections import deque

import numpy as np

from alignment import ContinuousNormalController, JointCommandFilter
from vision import CameraIntrinsics, estimate_button_target

RIGHT = slice(22, 29)
UPPER = slice(12, 29)
TIP_RADIUS = 0.008
STROKE = 0.0025
CLEARANCE = 0.05
RAISE_CLEARANCE = 0.18
RAISE_SIDE_OFFSET = 0.12
RAISE_MIN_HEIGHT = 0.24  # pelvis基準。低いボタンでも手首を折りすぎない持ち上げ高さ。


def validate_yada_target(target):
    """一般用ボタンの作業範囲（pelvis座標）。従来の実機作業範囲は変更しない。"""
    p, n = np.asarray(target.face_xyz), np.asarray(target.press_direction)
    if (not np.all(np.isfinite(p)) or not np.all(np.isfinite(n))
            or np.any(p < [0.19, -0.15, 0.10]) or np.any(p > [0.49, 0.15, 0.37])
            or not np.isclose(np.linalg.norm(n), 1.0) or n[0] < 0.9):
        raise ValueError("一般用ボタンの作業範囲または押下方向が不正です")


def select_target(candidates, direction, previous=None):
    """正解座標を使わず、画像から求めた一般用の上下ペアから選ぶ。"""
    if direction not in ("up", "down"):
        raise ValueError("対象は up / down で指定してください")
    valid = []
    for target in candidates:
        try:
            validate_yada_target(target)
        except ValueError:
            continue
        valid.append(target)
    if previous is not None:
        near = [v for v in valid if np.linalg.norm(np.asarray(v.face_xyz) - previous) <= 0.025]
        if len(near) != 1:
            raise ValueError("再計測で同じボタンを一意に追跡できません")
        return near[0]
    valid.sort(key=lambda t: -t.face_xyz[2])
    if len(valid) != 2:
        raise ValueError("一般用の上下ボタンを一意に検出できません")
    top, bottom = (np.asarray(t.face_xyz) for t in valid)
    if not 0.05 <= top[2] - bottom[2] <= 0.12 or np.linalg.norm(top[:2] - bottom[:2]) > 0.035:
        raise ValueError("一般用の上下ボタンの並びを確認できません")
    return valid[0 if direction == "up" else 1]


def color_boxes(rgb):
    """YOLO重みを指定しない場合の画像検出。局所背景より明るい灰色の丸を探す。"""
    from scipy import ndimage

    rgb = np.asarray(rgb, dtype=float)
    gray = rgb.mean(axis=2)
    background = ndimage.uniform_filter(gray, size=121)
    spots = (gray - background > 18) & (np.ptp(rgb, axis=2) < 40)
    spots = ndimage.binary_fill_holes(spots)
    labels, _ = ndimage.label(spots)
    boxes = []
    for i, window in enumerate(ndimage.find_objects(labels), 1):
        if window is None:
            continue
        component = labels[window] == i
        area = int(component.sum())
        h, w = component.shape
        if 40 <= area <= 20000 and area >= component.size * 0.5 and 0.5 <= w / h <= 2:
            y, x = window
            boxes.append((x.start, y.start, x.stop, y.stop))
    return boxes


def finger_axis(normal):
    """胸の前のボタンへ、指を右下から向ける。手先の移動は面の法線方向。"""
    left = np.cross([0, 0, 1], normal)
    left /= np.linalg.norm(left)
    up = np.cross(normal, left)
    a, b = np.deg2rad([35, 20])
    return np.cos(b) * (np.cos(a) * normal + np.sin(a) * left) + np.sin(b) * up


def raised_approach_point(face, normal):
    """壁から離れ、体の右側でボタンの高さへ腕を上げる経由点。"""
    right = np.cross(normal, [0, 0, 1])
    right /= np.linalg.norm(right)
    point = face - (TIP_RADIUS + RAISE_CLEARANCE) * normal + RAISE_SIDE_OFFSET * right
    point[2] = max(point[2], RAISE_MIN_HEIGHT)
    return point


class YadaButtonController:
    def __init__(self, weights=None):
        from common.j1gen_bridge import j1gen, j1gen_config

        robot = j1gen_config("robot.yaml")
        ik_cfg = dict(j1gen_config("press.yaml")["ik"])
        # 50Hzの微小移動を解ける精度。実機のMuJoCo運動学設定には触らない。
        ik_cfg.update(tol_pos_m=0.00002, damping=0.015, max_iter=350)
        self.kin = j1gen("kinematics").ArmKinematics(robot, "right", ik_cfg)
        self.camera_tf = j1gen("camera_geometry").HeadCameraTransform(robot)
        self.detector = None
        if weights:
            from ultralytics import YOLO

            self.detector = YOLO(weights)
        else:
            # DDSの動作開始後に初回importで制御を止めない。
            from scipy import ndimage  # noqa: F401

    def reset(self, task):
        if task.target not in ("up", "down") or not 0 < task.control_dt <= 0.1:
            raise ValueError("採点タスクの対象または制御周期が不正です")
        self.task = task
        self.phase, self.phase_t = "look", 0.0
        self.samples = deque(maxlen=3)
        self.last_image_t = -np.inf
        self.capture_after = -np.inf
        self.previous = None
        self.measurement_count = 0
        self.q_home = self.q_cmd = self.q_goal = None
        self.last_t = None
        self.axis = self.finger = self.face = None
        self.servo = self.joint_filter = None
        self.last_measurement_error = ""
        self.kp = np.asarray(task.upper_kp, dtype=float)
        self.detection = "yolo" if self.detector is not None else "color"

    def _measure(self, obs):
        if not np.isfinite(obs.image_t) or not 0 <= obs.t - obs.image_t <= 0.5:
            raise ValueError("RGB-D画像が古すぎる、または撮影時刻が不正です")
        if obs.image_t <= max(self.last_image_t, self.capture_after):
            return None
        self.last_image_t = obs.image_t
        if self.detector is None:
            boxes = color_boxes(obs.rgb)
        else:
            # 既存の重みはBGR画像で使う。API観測はRGBなので明示的に変換する。
            result = self.detector.predict(obs.rgb[:, :, ::-1].copy(), verbose=False)[0]
            boxes = [tuple(b.xyxy[0].cpu().numpy()) for b in result.boxes
                     if self.detector.names[int(b.cls[0])] == "button"]
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = self.camera_tf.pelvis_from_optical(obs.q[12:15])
        intr = CameraIntrinsics(obs.K[0, 0], obs.K[1, 1], obs.K[0, 2], obs.K[1, 2])
        candidates = []
        for bbox in boxes:
            try:
                candidates.append(estimate_button_target(obs.depth, bbox, intr, transform,
                                                        expected_face_xyz=self.previous))
            except ValueError:
                continue
        try:
            target = select_target(candidates, self.task.target, self.previous)
        except ValueError as exc:
            self.last_measurement_error = f"{exc}（検出枠 {len(boxes)} 個、有効な3D候補 {len(candidates)} 個）"
            self.samples.clear()
            return None
        self.samples.append(target)
        if len(self.samples) < 3:
            return None
        positions = np.array([t.face_xyz for t in self.samples])
        normals = np.array([t.press_direction for t in self.samples])
        face, normal = np.median(positions, axis=0), np.median(normals, axis=0)
        normal /= np.linalg.norm(normal)
        if (np.max(np.linalg.norm(positions - face, axis=1)) > 0.004
                or np.min(normals @ normal) < np.cos(np.deg2rad(5))):
            self.last_measurement_error = "3フレームの位置または法線のばらつきが大きすぎます"
            return None
        self.previous = face.copy()
        self.measurement_count += 1
        return face, normal

    def _solve(self, xyz, seed=None, initial=False):
        seed = self.q_cmd.copy() if seed is None else seed.copy()
        if initial:
            seed[RIGHT] = [-1.0, -0.5, 0.3, 0.3, 0, 0, 0]
            seed = self.kin.ik(xyz, seed).q
        result = self.kin.ik(xyz, seed, self.finger)
        if not result.success:
            raise ValueError(f"採点用のIKが収束しません: {result.reason}")
        return result.q

    def _move_to(self, xyz, obs, duration, next_phase, initial=False):
        self.move_start = self.q_cmd.copy()
        self.q_goal = self._solve(xyz, initial=initial)
        self.move_duration = max(duration, 1.5 * np.max(np.abs(self.q_goal[RIGHT] - self.move_start[RIGHT])) / 0.8)
        self.move_next = next_phase
        self._enter("move", obs.t)

    def _enter(self, phase, t):
        self.phase, self.phase_t = phase, t

    def _gravity_offset(self, obs):
        # IMUの重力方向を使う。採点上のPDは腕の重力を補償しない。
        pin = self.kin._pin
        quat = np.asarray(obs.imu_quat, dtype=float)
        if quat.shape != (4,) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 0.5:
            raise ValueError("IMU姿勢が不正です")
        w, x, y, z = quat / np.linalg.norm(quat)
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        self.kin.model.gravity.linear = rotation.T @ np.array([0, 0, -9.81])
        tau = pin.computeGeneralizedGravity(self.kin.model, self.kin.data, self.q_cmd)
        return self.q_cmd[UPPER] + tau[UPPER] / np.where(self.kp > 0, self.kp, np.inf)

    def act(self, obs):
        if (np.asarray(obs.q).shape != (29,) or not np.all(np.isfinite(obs.q))
                or not np.isfinite(obs.t)):
            raise ValueError("採点観測の関節角または時刻が不正です")
        dt = self.task.control_dt if self.last_t is None else obs.t - self.last_t
        if dt <= 0 or (self.phase == "press" and dt > 0.1):
            raise ValueError("連続押下の状態更新が100ms以上途絶えた、または時刻が逆行しました")
        self.last_t = obs.t
        if self.q_home is None:
            self.q_home = obs.q.copy()
            self.q_cmd = obs.q.copy()
        elapsed = obs.t - self.phase_t
        tip = self.kin.fk_pos(obs.q)

        if self.phase in ("look", "remeasure"):
            target = self._measure(obs)
            if target is None:
                if elapsed > 2.0:
                    raise ValueError(f"新しい3フレームから安定したボタン位置を計測できません: {self.last_measurement_error}")
            else:
                self.face, self.axis = target
                self.finger = finger_axis(self.axis)
                goal = self.face - (TIP_RADIUS + CLEARANCE) * self.axis
                if self.phase == "remeasure" and np.linalg.norm(goal - tip) > 0.08:
                    raise ValueError("再計測後の位置補正が8cmを超えました")
                if self.phase == "look":
                    self._move_to(raised_approach_point(self.face, self.axis), obs, 2.0,
                                  "approach", initial=True)
                else:
                    self._move_to(goal, obs, 1.0, "align")
        elif self.phase == "move":
            s = min(1.0, elapsed / self.move_duration)
            s = s*s*(3-2*s)
            self.q_cmd[RIGHT] = (1-s)*self.move_start[RIGHT] + s*self.q_goal[RIGHT]
            if elapsed >= self.move_duration:
                self._enter(self.move_next, obs.t)
        elif self.phase == "approach":
            self._move_to(self.face - (TIP_RADIUS + CLEARANCE) * self.axis, obs, 2.0, "settle")
        elif self.phase == "settle":
            if elapsed >= 0.3:
                self.samples.clear()
                # 静止待ち以前に撮った画像は再計測に使わない。
                self.capture_after = obs.t
                self._enter("remeasure", obs.t)
        elif self.phase == "align":
            goal = self.face - (TIP_RADIUS + CLEARANCE) * self.axis
            error = goal - tip
            if np.linalg.norm(error) <= 0.003:
                self.servo = ContinuousNormalController(self.face, self.axis, TIP_RADIUS, STROKE, tip)
                self.joint_filter = JointCommandFilter(self.q_cmd[RIGHT])
                self._enter("press", obs.t)
            elif elapsed > 5.0 or np.linalg.norm(error) > 0.01:
                raise ValueError("再計測後の手先位置合わせが収束しません")
            else:
                goal = goal + error * min(1.0, 0.005 / np.linalg.norm(error))
                self.q_cmd = self._solve(goal)
        elif self.phase == "press":
            if elapsed > 20.0:
                raise ValueError("連続押下が制限時間内に収束しません")
            goal = self.servo.next_goal(tip, dt)
            if goal is None:
                self._move_to(self.face - (TIP_RADIUS + CLEARANCE)*self.axis, obs, 2.0, "home")
            else:
                solved = self._solve(goal)
                self.q_cmd[RIGHT] = self.joint_filter.next_goal(solved[RIGHT], dt)
        elif self.phase == "home":
            self.move_start = self.q_cmd.copy()
            self.q_goal = self.q_home.copy()
            self.move_duration, self.move_next = 4.0, "done"
            self._enter("move", obs.t)
        return self._gravity_offset(obs), self.phase == "done"
