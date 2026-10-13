"""採点環境へのアダプタ。画像から位置を推定し、J1-genを参考に押下する。

MuJoCo / Isaac の状態や試行の正解にはアクセスしない。機体設定のみ J1-gen と共用する。
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace

import numpy as np

from vision import CameraIntrinsics, estimate_button_target
from yada_press import (ButtonLampCheck, MAX_COMMAND_DEPTH_M, MAX_RETRIES,
                        PRESS_COMMAND_DEPTH_M, PressCommandController, PressJointCommandFilter,
                        RETRY_EXTRA_DEPTH_M)

RIGHT = slice(22, 29)
UPPER = slice(12, 29)
TIP_RADIUS = 0.008
CLEARANCE = 0.05
RETRACT_CLEARANCE = 0.18  # 点灯確認後は壁から十分離れてから腕を初期姿勢へ戻す。
RAISE_CLEARANCE = 0.18
RAISE_SIDE_OFFSET = 0.12
RAISE_MIN_HEIGHT = 0.24  # pelvis基準。低いボタンでも手首を折りすぎない持ち上げ高さ。
# 公開40配置で上下を検出できた最小の比較解像度。重み内の512を継承しない。
YOLO_IMAGE_SIZE = 1024
POSE_HISTORY_S = 0.75  # 画像の許容遅延500msを含む姿勢履歴。


def imu_rotation(imu_quat):
    """IMUのwxyz姿勢を、world_from_pelvisの回転にする。"""
    quat = np.asarray(imu_quat, dtype=float)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 0.5:
        raise ValueError("IMU姿勢が不正です")
    w, x, y, z = quat / np.linalg.norm(quat)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class PoseHistory:
    """関節角とIMUを保存し、画像の撮影時刻へ補間する。並進は推定しない。"""

    def __init__(self):
        self.samples = deque()

    def add(self, t, q_waist, imu_quat):
        q = np.asarray(q_waist, dtype=float)
        quat = np.asarray(imu_quat, dtype=float)
        if not np.isfinite(t) or q.shape != (3,) or not np.all(np.isfinite(q)):
            raise ValueError("姿勢履歴の時刻または腰の関節角が不正です")
        imu_rotation(quat)  # 形状・有限値・ノルムを検査する。
        if self.samples and t <= self.samples[-1][0]:
            raise ValueError("姿勢履歴の時刻が逆行または重複しました")
        self.samples.append((t, q.copy(), quat / np.linalg.norm(quat)))
        while len(self.samples) > 2 and self.samples[1][0] < t - POSE_HISTORY_S:
            self.samples.popleft()

    def at(self, t):
        if (not np.isfinite(t) or not self.samples
                or t < self.samples[0][0] or t > self.samples[-1][0]):
            raise ValueError("撮影時刻に対応する姿勢履歴がありません")
        for i, (stamp, q, quat) in enumerate(self.samples):
            if t == stamp:
                return q.copy(), quat.copy()
            if stamp > t:
                old_t, old_q, old_quat = self.samples[i - 1]
                if stamp - old_t > 0.1 + 1e-9:
                    raise ValueError("撮影時刻を挟む姿勢更新が100msを超えて途絶えています")
                s = (t - old_t) / (stamp - old_t)
                dot = float(old_quat @ quat)
                if dot < 0:
                    quat, dot = -quat, -dot
                dot = float(np.clip(dot, -1, 1))
                if dot > 0.9995:
                    interpolated = (1 - s) * old_quat + s * quat
                else:
                    angle = np.arccos(dot)
                    interpolated = (np.sin((1-s)*angle)*old_quat + np.sin(s*angle)*quat) / np.sin(angle)
                return (1 - s) * old_q + s * q, interpolated / np.linalg.norm(interpolated)
        raise ValueError("撮影時刻に対応する姿勢履歴がありません")


def correct_vertical_panel(face, normal, camera_origin, imu_quat):
    """鉛直パネルの法線から、重力方向に現れるカメラの傾きを補正する。

    法線だけでなくカメラ原点からの位置ベクトルも同じ回転で補正する。
    カメラの並進ずれや重力軸まわりの回転は、この拘束だけでは求まらない。
    """
    up = imu_rotation(imu_quat).T @ np.array([0.0, 0.0, 1.0])
    tilt = float(np.asarray(normal) @ up)
    if abs(tilt) > np.sin(np.deg2rad(5)):
        raise ValueError("パネルの鉛直補正が5度を超えました")
    corrected_normal = normal - tilt * up
    corrected_normal /= np.linalg.norm(corrected_normal)
    x, y, z = np.cross(normal, corrected_normal)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    rotation = np.eye(3) + skew + skew @ skew / (1 + normal @ corrected_normal)
    corrected_face = camera_origin + rotation @ (face - camera_origin)
    return corrected_face, corrected_normal


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
        self.pose_history = PoseHistory()
        self.last_image_t = -np.inf
        self.capture_after = -np.inf
        self.previous = None
        self.measurement_count = 0
        self.q_home = self.q_cmd = self.q_goal = None
        self.last_t = None
        self.axis = self.finger = self.face = None
        self.face_fixed = self.axis_fixed = self.finger_fixed = None
        self.servo = self.joint_filter = None
        self.target_bbox = self.lamp_check = None
        self.retry_count = 0
        self.press_depth_m = PRESS_COMMAND_DEPTH_M
        self.lamp_confirmed = False
        self.last_measurement_error = ""
        self.kp = np.asarray(task.upper_kp, dtype=float)
        self.detection = "yolo" if self.detector is not None else "color"

    def _measure(self, obs):
        if not np.isfinite(obs.image_t) or not 0 <= obs.t - obs.image_t <= 0.5:
            raise ValueError("RGB-D画像が古すぎる、または撮影時刻が不正です")
        if obs.image_t <= max(self.last_image_t, self.capture_after):
            return None
        self.last_image_t = obs.image_t
        try:
            q_capture, imu_capture = self.pose_history.at(obs.image_t)
        except ValueError as exc:
            self.last_measurement_error = str(exc)
            self.samples.clear()
            return None
        capture_rotation = imu_rotation(imu_capture)
        # previousは補正前の点を、IMUで向きを固定した基準に保存している。
        previous_capture = None if self.previous is None else capture_rotation.T @ self.previous
        if self.detector is None:
            boxes = color_boxes(obs.rgb)
        else:
            # 既存の重みはBGR画像で使う。API観測はRGBなので明示的に変換する。
            result = self.detector.predict(obs.rgb[:, :, ::-1].copy(),
                                           imgsz=YOLO_IMAGE_SIZE, verbose=False)[0]
            boxes = [tuple(b.xyxy[0].cpu().numpy()) for b in result.boxes
                     if self.detector.names[int(b.cls[0])] == "button"]
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = self.camera_tf.pelvis_from_optical(q_capture)
        intr = CameraIntrinsics(obs.K[0, 0], obs.K[1, 1], obs.K[0, 2], obs.K[1, 2])
        candidates, candidate_boxes = [], {}
        for bbox in boxes:
            try:
                candidate = estimate_button_target(obs.depth, bbox, intr, transform,
                                                   expected_face_xyz=previous_capture)
                candidates.append(candidate)
                candidate_boxes[id(candidate)] = bbox
            except ValueError:
                continue
        try:
            target = select_target(candidates, self.task.target, previous_capture)
        except ValueError as exc:
            self.last_measurement_error = f"{exc}（検出枠 {len(boxes)} 個、有効な3D候補 {len(candidates)} 個）"
            self.samples.clear()
            return None
        raw_face = np.asarray(target.face_xyz)
        corrected_face, corrected_normal = correct_vertical_panel(
            raw_face, np.asarray(target.press_direction), transform[:3, 3], imu_capture)
        validate_yada_target(replace(target, face_xyz=tuple(corrected_face),
                                    press_direction=tuple(corrected_normal)))
        # 異なる撮影時刻の3フレームを、機体の向きが変わらない基準にそろえる。
        self.samples.append((capture_rotation @ raw_face, capture_rotation @ corrected_face,
                             capture_rotation @ corrected_normal))
        if len(self.samples) < 3:
            return None
        raw_positions = np.array([s[0] for s in self.samples])
        positions = np.array([s[1] for s in self.samples])
        normals = np.array([s[2] for s in self.samples])
        face, normal = np.median(positions, axis=0), np.median(normals, axis=0)
        normal /= np.linalg.norm(normal)
        if (np.max(np.linalg.norm(positions - face, axis=1)) > 0.004
                or np.min(normals @ normal) < np.cos(np.deg2rad(5))):
            self.last_measurement_error = "3フレームの位置または法線のばらつきが大きすぎます"
            return None
        current_rotation = imu_rotation(obs.imu_quat)
        face, normal = current_rotation.T @ face, current_rotation.T @ normal
        validate_yada_target(replace(target, face_xyz=tuple(face), press_direction=tuple(normal)))
        # 深度選択には補正前の点を、次の画像の撮影姿勢へ戻して渡す。
        self.previous = np.median(raw_positions, axis=0)
        self.target_bbox = candidate_boxes[id(target)]
        self.measurement_count += 1
        return face, normal

    def _solve(self, xyz, seed=None, initial=False, align_finger=True):
        seed = self.q_cmd.copy() if seed is None else seed.copy()
        # 初回だけ、肘の曲げ方を変えた種姿勢も試す。連続押下では現在の解を維持する。
        for elbow in ([0.3, 0.7, 1.2] if initial else [None]):
            candidate = seed.copy()
            if initial:
                candidate[RIGHT] = [-1.0, -0.5, 0.3, elbow, 0, 0, 0]
                candidate = self.kin.ik(xyz, candidate).q
            result = self.kin.ik(xyz, candidate, self.finger if align_finger else None)
            if result.success:
                return result.q
        if not align_finger:
            # 向きを自由にした後退で肘の解がリミットに張り付く場合は、
            # 手首の種姿勢を少し変えて、同じ位置へ届く別の解を探す。
            for wrist_yaw in (0.3, -0.3):
                candidate = seed.copy()
                candidate[28] += wrist_yaw
                result = self.kin.ik(xyz, candidate)
                if result.success:
                    return result.q
        raise ValueError(f"採点用のIKが収束しません: {result.reason}")

    def _move_to(self, xyz, obs, duration, next_phase, initial=False, align_finger=True):
        self.move_start = self.q_cmd.copy()
        self.q_goal = self._solve(xyz, initial=initial, align_finger=align_finger)
        self.move_duration = max(duration, 1.5 * np.max(np.abs(self.q_goal[RIGHT] - self.move_start[RIGHT])) / 0.8)
        self.move_next = next_phase
        self._enter("move", obs.t)

    def _enter(self, phase, t):
        self.phase, self.phase_t = phase, t

    def _start_press(self, obs, rotation, tip):
        # 指令・面・指先を同じ固定方向の基準で扱い、姿勢変化を毎周期反映する。
        self.servo = PressCommandController(self.face_fixed, self.axis_fixed, TIP_RADIUS,
                                            rotation @ tip, self.press_depth_m)
        self.joint_filter = PressJointCommandFilter(self.q_cmd[RIGHT])
        self._enter("press", obs.t)

    def _gravity_offset(self, obs):
        # IMUの重力方向を使う。採点上のPDは腕の重力を補償しない。
        pin = self.kin._pin
        rotation = imu_rotation(obs.imu_quat)
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
        rotation = imu_rotation(obs.imu_quat)
        self.pose_history.add(obs.t, obs.q[12:15], obs.imu_quat)
        self.last_t = obs.t
        if self.q_home is None:
            self.q_home = obs.q.copy()
            self.q_cmd = obs.q.copy()
        elapsed = obs.t - self.phase_t
        tip = self.kin.fk_pos(obs.q)
        if self.face_fixed is not None:
            self.face = rotation.T @ self.face_fixed
            self.axis = rotation.T @ self.axis_fixed
            self.finger = rotation.T @ self.finger_fixed

        if self.phase in ("look", "remeasure"):
            target = self._measure(obs)
            if target is None:
                if elapsed > 2.0:
                    raise ValueError(f"新しい3フレームから安定したボタン位置を計測できません: {self.last_measurement_error}")
            else:
                self.face, self.axis = target
                self.finger = finger_axis(self.axis)
                self.face_fixed, self.axis_fixed = rotation @ self.face, rotation @ self.axis
                self.finger_fixed = rotation @ self.finger
                goal = self.face - (TIP_RADIUS + CLEARANCE) * self.axis
                if self.phase == "remeasure" and np.linalg.norm(goal - tip) > 0.08:
                    raise ValueError("再計測後の位置補正が8cmを超えました")
                if self.phase == "look":
                    self._move_to(raised_approach_point(self.face, self.axis), obs, 2.0,
                                  "approach", initial=True)
                else:
                    self.lamp_check = ButtonLampCheck(obs.rgb, self.target_bbox, obs.image_t)
                    self._move_to(goal, obs, 1.0, "align")
        elif self.phase == "move":
            s = min(1.0, elapsed / self.move_duration)
            s = s*s*(3-2*s)
            self.q_cmd[RIGHT] = (1-s)*self.move_start[RIGHT] + s*self.q_goal[RIGHT]
            if elapsed >= self.move_duration:
                if self.move_next == "check":
                    # 後退中の画像や同一フレームを点灯確認に数えない。
                    self.lamp_check.reset_frames(obs.t)
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
                self._start_press(obs, rotation, tip)
            elif elapsed > 5.0 or np.linalg.norm(error) > 0.01:
                raise ValueError("再計測後の手先位置合わせが収束しません")
            else:
                goal = goal + error * min(1.0, 0.005 / np.linalg.norm(error))
                self.q_cmd = self._solve(goal)
        elif self.phase == "press":
            if elapsed > 20.0:
                raise ValueError("連続押下が制限時間内に収束しません")
            goal = self.servo.next_goal(rotation @ tip, dt)
            if goal is None:
                self._move_to(self.face - (TIP_RADIUS + CLEARANCE)*self.axis, obs, 2.0, "check")
            else:
                solved = self._solve(rotation.T @ goal)
                self.q_cmd[RIGHT] = self.joint_filter.next_goal(solved[RIGHT], dt)
                # 追従差は、IKの要求位置ではなく制限後に送る関節指令から測る。
                # 押下経路はservoのstart/endに別途保持されるので、横方向も同期できる。
                self.servo.command_xyz = rotation @ self.kin.fk_pos(self.q_cmd)
        elif self.phase == "check":
            lit = self.lamp_check.observe(obs.rgb, obs.t, obs.image_t)
            if lit is True:
                self.lamp_confirmed = True
                self._enter("home", obs.t)
            elif lit is False:
                if self.retry_count >= MAX_RETRIES:
                    raise ValueError("押し直し後も対象ボタンの点灯を確認できません")
                self.retry_count += 1
                self.press_depth_m = min(MAX_COMMAND_DEPTH_M, self.press_depth_m + RETRY_EXTRA_DEPTH_M)
                self._start_press(obs, rotation, tip)
            elif elapsed > 2.0:
                raise ValueError("後退後の新しい画像で点灯を確認できません")
        elif self.phase == "home":
            # 点灯確認・押し直しは5cm手前で行い、確認後だけ後退を延ばす。
            # 指を向け続ける必要がないので向きの制約を外し、肘のリミットを避ける。
            self._move_to(self.face - (TIP_RADIUS + RETRACT_CLEARANCE)*self.axis,
                          obs, 1.8, "return_home", align_finger=False)
        elif self.phase == "return_home":
            self.move_start = self.q_cmd.copy()
            self.q_goal = self.q_home.copy()
            self.move_duration, self.move_next = 4.0, "done"
            self._enter("move", obs.t)
        return self._gravity_offset(obs), self.phase == "done"
