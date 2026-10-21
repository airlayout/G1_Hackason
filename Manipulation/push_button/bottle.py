"""台上の直立ボトルのRGB-D計測と、既存の腕制御を使う押し動作。"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from xml.etree import ElementTree as ET
import math

import numpy as np

from alignment import (ContinuousNormalController, JointCommandFilter,
                       SmoothNormalSettings, run_continuous_normal)
from vision import CameraIntrinsics, _points


DEFAULT_TABLE_BOUNDS = (0.20, 0.85, -0.60, 0.02)
FORWARD = np.array([1., 0., 0.])


def validate_push_distance(value):
    value = float(value)
    if not np.isfinite(value) or not 0 < value <= 0.10:
        raise ValueError("push-distanceは0より大きく0.10m以下が必要です")
    return value


@dataclass(frozen=True)
class BottleSettings:
    table_height: float
    push_distance: float = 0.05
    contact_height: float = 0.08
    clearance: float = 0.05
    table_bounds: tuple = DEFAULT_TABLE_BOUNDS

    def __post_init__(self):
        validate_push_distance(self.push_distance)
        if (not np.all(np.isfinite([self.table_height, self.contact_height, self.clearance]))
                or not 0.55 <= self.table_height <= 1.0
                or not 0.04 <= self.contact_height <= 0.15
                or not 0.03 <= self.clearance <= 0.05):
            raise ValueError("台高は0.55～1.0m、接触高さは4～15cm、clearanceは3～5cmが必要です")
        bounds = np.asarray(self.table_bounds)
        if (bounds.shape != (4,) or not np.all(np.isfinite(bounds))
                or bounds[1] <= bounds[0] or bounds[3] <= bounds[2]):
            raise ValueError("table-boundsにはX最小・最大、Y最小・最大を指定してください")


def add_bottle_arguments(parser, *, simulation=False):
    parser.add_argument("--weights", default=None, help="ボトル用YOLO重み。既定: yolo26n.pt (無ければ公式から取得)")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--push-distance", type=float, default=0.05, help="前方への移動距離[m] (0より大きく0.10以下)")
    parser.add_argument("--table-height", type=float, default=0.75 if simulation else None,
                        required=not simulation, help="床基準の台上面高さ[m]")
    parser.add_argument("--contact-height", type=float, default=0.08, help="台上面からの接触高さ[m]")
    parser.add_argument("--table-bounds", type=float, nargs=4, default=DEFAULT_TABLE_BOUNDS,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"), help="実測する台の平面範囲[m]")
    parser.add_argument("--clearance", type=float, default=0.05)
    parser.add_argument("--target-pixel", type=int, nargs=2)


def settings_from_args(args):
    if not np.isfinite(args.confidence) or not 0 < args.confidence < 1:
        raise ValueError("confidenceは0～1の間が必要です")
    return BottleSettings(args.table_height, args.push_distance, args.contact_height,
                          args.clearance, tuple(args.table_bounds))


def make_bottle_detector(weights=None, confidence=0.25, device="auto", *, runtime=None):
    # J1-genとPerceptionのcommonパッケージ名を衝突させない。
    from real.j1gen_camera import J1CameraRuntime

    runtime = runtime or J1CameraRuntime()
    detector = runtime.module("perception_bridge").perception("detector.yolo_detector").YoloDetector
    local = Path(__file__).resolve().parents[2] / "yolo26n.pt"
    model = str(local) if weights is None and local.is_file() else (weights or "yolo26n.pt")
    return detector(model_name=model, classes=["bottle"], confidence_threshold=confidence, device=device)


def add_table(root, settings):
    """同じ実測の台形状を実機の経路検査と模擬環境へ使う。"""
    x0, x1, y0, y1 = settings.table_bounds
    ET.SubElement(root.find("worldbody"), "geom", name="push_table", type="box",
                  pos=f"{(x0+x1)/2} {(y0+y1)/2} {settings.table_height-0.025}",
                  size=f"{(x1-x0)/2} {(y1-y0)/2} 0.025", rgba="0.55 0.38 0.23 1",
                  friction="0.3 0.005 0.0001")


@dataclass(frozen=True)
class BottleTarget:
    center_xy: tuple[float, float]
    radius_m: float
    face_xyz: tuple[float, float, float]
    depth_valid_fraction: float
    fit_inlier_fraction: float
    bbox: tuple[float, float, float, float]


def validate_bottle_target(target, settings, *, require_travel=True):
    x, y, z = target.face_xyz
    if (not np.all(np.isfinite([x, y, z, *target.center_xy, target.radius_m]))
            or not 0.19 <= x <= 0.49 or not -0.45 <= y <= -0.10
            or not 0.70 <= z <= 1.20 or not 0.02 <= target.radius_m <= 0.06):
        raise ValueError("ボトル位置・寸法が右腕の作業範囲外です")
    x0, x1, y0, y1 = settings.table_bounds
    cx, cy = target.center_xy
    radius = target.radius_m + 0.01
    distance = settings.push_distance if require_travel else 0
    if not (x0+radius <= cx and cx+distance+radius <= x1
            and y0+radius <= cy <= y1-radius):
        raise ValueError("ボトルの移動先まで台の余裕がありません")


def _fit_circle(points):
    """鉛直な胴体のXY断面をRANSACで求め、背景の平面を採用しない。"""
    if len(points) > 1600:
        points = points[np.linspace(0, len(points)-1, 1600, dtype=int)]
    xy = points[:, :2]
    rng = np.random.default_rng(0)
    best = np.zeros(len(xy), dtype=bool)
    for _ in range(90):
        p = xy[rng.choice(len(xy), 3, replace=False)]
        matrix = 2 * (p[1:]-p[0])
        if abs(np.linalg.det(matrix)) < 1e-7:
            continue
        center = np.linalg.solve(matrix, (p[1:]**2).sum(axis=1)-(p[0]**2).sum())
        radius = np.linalg.norm(p[0]-center)
        if not 0.02 <= radius <= 0.06:
            continue
        inliers = np.abs(np.linalg.norm(xy-center, axis=1)-radius) < 0.0025
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 30 or best.mean() < 0.60:
        raise ValueError("ボトル胴体の曲面深度を安定して推定できません")
    fitted = xy[best]
    # 原点付近に移し、短い円弧でも最小二乗の桁落ちを避ける。
    origin = fitted.mean(axis=0)
    q = fitted-origin
    cx, cy, c = np.linalg.lstsq(np.c_[2*q, np.ones(len(q))], (q*q).sum(axis=1), rcond=None)[0]
    center = origin + [cx, cy]
    radius = np.sqrt(max(0., c+cx*cx+cy*cy))
    angle = np.unwrap(np.sort(np.arctan2(fitted[:, 1]-center[1], fitted[:, 0]-center[0])))
    # 撮影側が見える円弧に30度以上の曲率を要求する。
    if not 0.02 <= radius <= 0.06 or np.ptp(angle) < np.deg2rad(30):
        raise ValueError("ボトル胴体の半径または見える曲面が不十分です")
    return center, float(radius), float(best.mean())


def estimate_bottle_target(depth_m, bbox, intrinsics: CameraIntrinsics, transform, settings):
    depth = np.asarray(depth_m)
    transform = np.asarray(transform, dtype=float)
    if (depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number)
            or transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=0.002)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=0.002)):
        raise ValueError("ボトル計測の深度画像または校正行列が不正です")
    if (not np.all(np.isfinite([intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy]))
            or min(intrinsics.fx, intrinsics.fy) <= 0):
        raise ValueError("カメラ内部パラメータが不正です")
    box = np.asarray(bbox, dtype=float)
    if (box.shape != (4,) or not np.all(np.isfinite(box)) or min(box[2:]-box[:2]) < 8
            or min(box[:2]) < 0 or box[2] > depth.shape[1] or box[3] > depth.shape[0]):
        raise ValueError("ボトル検出枠が画像外または小さすぎます")
    x0, y0, x1, y1 = box
    # 台・キャップ・輪郭の混合画素を避ける。胴体の高さは床座標で選ぶ。
    margin = (x1-x0)*0.08
    ys, xs = np.mgrid[int(np.ceil(y0)):int(y1), int(np.ceil(x0+margin)):int(x1-margin)]
    values = depth[ys, xs]
    valid = np.isfinite(values) & (values >= 0.15) & (values <= 2.)
    valid_fraction = float(valid.mean())
    if valid.sum() < 40 or valid_fraction < 0.60:
        raise ValueError("ボトル胴体の有効深度が不足しています")
    points = _points(np.c_[xs[valid], ys[valid]], values[valid], intrinsics)
    points = points @ transform[:3, :3].T + transform[:3, 3]
    height = settings.table_height + settings.contact_height
    band = points[np.abs(points[:, 2]-height) < 0.025]
    if len(band) < 40:
        raise ValueError("接触高さに有効なボトル胴体の深度がありません")
    center, radius, fraction = _fit_circle(band)
    # 胴体が複数の高さで同じ円筒に乗ることを確認。平面・大きな傾きは拒否する。
    body = points[(points[:, 2] > settings.table_height+0.025)
                  & (points[:, 2] < height+0.055)]
    radial = np.abs(np.linalg.norm(body[:, :2]-center, axis=1)-radius) < 0.004
    if radial.sum() < 40 or radial.mean() < 0.55 or np.ptp(body[radial, 2]) < 0.035:
        raise ValueError("直立したボトル胴体の深度が不足または背景と混在しています")
    target = BottleTarget(tuple(center), radius, (float(center[0]-radius), float(center[1]), height),
                          valid_fraction, fraction, tuple(box))
    return target


@dataclass(frozen=True)
class BottleFrame:
    color_bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    transform: np.ndarray
    captured_at: float
    sequence: int


@dataclass(frozen=True)
class BottleObservation:
    target: BottleTarget
    captured_at: float
    sequence: int


class BottleLocalizer:
    """各計測で新しい3枚を要求し、押した後は予測移動先の同じ対象を追う。"""

    def __init__(self, read_frame, detector, settings, now, target_pixel=None):
        self.read_frame, self.detector, self.settings, self.now = read_frame, detector, settings, now
        self.target_pixel = target_pixel
        self.previous = None
        self.expected_xy = None
        self.last_sequence = -1
        self.last_capture = -np.inf
        self.last_frame = None
        self.measurements = []
        self.after_push = False
        self.rejections = Counter()

    def expect_translation(self):
        if self.previous is None:
            raise ValueError("移動前のボトル計測がありません")
        self.expected_xy = np.array(self.previous.center_xy) + self.settings.push_distance * FORWARD[:2]
        self.after_push = True

    def observe(self, timeout_s=5.):
        after = self.now()
        deadline = after+timeout_s
        samples = deque(maxlen=3)
        while self.now() < deadline:
            frame = self.read_frame(after, deadline)
            if (not np.isfinite(frame.captured_at) or frame.sequence <= self.last_sequence
                    or frame.captured_at <= max(after, self.last_capture)
                    or not 0 <= self.now()-frame.captured_at <= 0.5):
                samples.clear()
                continue
            self.last_capture, self.last_sequence = frame.captured_at, frame.sequence
            candidates = []
            for detection in self.detector.detect(frame.color_bgr):
                if detection.class_name != "bottle":
                    continue
                if self.previous is None and self.target_pixel is not None:
                    u, v = self.target_pixel
                    x0, y0, x1, y1 = detection.bbox
                    if not (x0 <= u <= x1 and y0 <= v <= y1):
                        continue
                try:
                    target = estimate_bottle_target(frame.depth_m, detection.bbox, frame.intrinsics,
                                                    frame.transform, self.settings)
                    validate_bottle_target(target, self.settings, require_travel=not self.after_push)
                except ValueError as exc:
                    self.rejections[str(exc)] += 1
                    continue
                if self.previous is not None:
                    expected = self.expected_xy if self.expected_xy is not None else self.previous.center_xy
                    if (np.linalg.norm(np.array(target.center_xy)-expected) > 0.025
                            or abs(target.radius_m-self.previous.radius_m) > 0.005):
                        self.rejections["追跡位置または半径が一致しません"] += 1
                        continue
                candidates.append(target)
            # 推論後にも鮮度検査。古い画像を3枚の安定計測へ数えない。
            if not 0 <= self.now()-frame.captured_at <= 0.5:
                samples.clear()
                continue
            if len(candidates) > 1:
                raise ValueError("ボトルを一意に選べません。--target-pixelを指定してください")
            if not candidates:
                samples.clear()
                continue
            samples.append(candidates[0])
            self.last_frame = frame
            if len(samples) == 3:
                positions = np.array([s.center_xy for s in samples])
                center = np.median(positions, axis=0)
                if (np.max(np.linalg.norm(positions-center, axis=1)) > 0.004
                        or np.ptp([s.radius_m for s in samples]) > 0.003):
                    continue
                radius = float(np.median([s.radius_m for s in samples]))
                stable = replace(samples[-1], center_xy=tuple(center), radius_m=radius,
                                 face_xyz=(float(center[0]-radius), float(center[1]), samples[-1].face_xyz[2]))
                self.previous = stable
                obs = BottleObservation(stable, frame.captured_at, frame.sequence)
                self.measurements.append({"captured_at_s": frame.captured_at, "sequence": frame.sequence,
                                          "after_push": self.after_push, "target": asdict(stable)})
                return obs
        raise RuntimeError("安定した新しい3フレームのボトル計測を取得できません（対象消失・深度・鮮度を確認）")


class BottlePushRoutine:
    """motionは既存のArmMotionと同じAPI。シミュレーションも同じ制御を通る。"""

    def __init__(self, kin, motion, localizer, refresh, now, settings, tip_radius,
                 position_uncertainty_m=0.):
        self.kin, self.motion, self.localizer = kin, motion, localizer
        self.refresh, self.now, self.settings = refresh, now, settings
        self.radius = tip_radius
        self.position_uncertainty_m = position_uncertainty_m
        self.report = {"schema": "g1-bottle-push-v1", "task": "bottle", "settings": asdict(settings),
                       "completed": False, "motion_completed": False, "home_completed": False,
                       "visual_success": None, "measured_displacement_m": None,
                       "physical_push_success": None, "phases": [], "normal_steps": []}

    def move(self, xyz, face, duration=2., contact=False):
        self.refresh()
        solution = self.kin.solve(np.asarray(xyz), FORWARD)
        self.kin.validate_path(solution.joint_angles, face, FORWARD, contact)
        delta = np.max(np.abs(solution.joint_angles-self.motion.current_target()))
        self.motion.move(solution.joint_angles, max(duration, math.sqrt(6*delta/2.)), settle_s=0.3)

    def phase(self, name):
        self.report["phase"] = name
        self.report["phases"].append({"phase": name, "time_s": self.now()})

    def raised_point(self, face):
        # 台の手前で先に持ち上げる。低い初期姿勢から台上へ斜めに入らない。
        return np.array([min(self.settings.table_bounds[0]-0.02, face[0]-0.10),
                         face[1]-0.10, max(0.95, face[2]+0.12)])

    def check_observation(self, observation):
        if not 0 <= self.now()-observation.captured_at <= 0.5:
            raise ValueError("ボトル計測が古すぎます")
        return np.array(observation.target.face_xyz)

    def preview(self, observation):
        """初期計測による到達性検査。実行時の再計測を代替しない。"""
        face = self.check_observation(observation)
        if observation.target.radius_m-self.radius < self.position_uncertainty_m+0.003:
            raise ValueError("ボトルと手先の寸法に対して位置の不確かさが大きすぎます")
        original = self.kin.data.qpos.copy()
        poses = {}
        try:
            for name, offset in (("raise", None), ("transit", -0.10), ("approach", -self.settings.clearance),
                                 ("contact", -0.003), ("push", self.settings.push_distance),
                                 ("retract", -self.settings.clearance), ("return_raise", None)):
                xyz = self.raised_point(face) if offset is None else face + (offset-self.radius)*FORWARD
                goal = self.kin.solve(xyz, FORWARD).joint_angles
                self.kin.validate_path(goal, face, FORWARD, name in ("push", "retract"))
                poses[name] = goal.tolist()
                q = self.kin.data.qpos.copy()
                q[self.kin.qadr] = goal
                self.kin.update(q)
            self.kin.validate_path(original[self.kin.qadr])
            poses["home"] = original[self.kin.qadr].tolist()
        finally:
            self.kin.update(original)
        return {"schema": "g1-bottle-preview-v1", "task": "bottle", "settings": asdict(self.settings),
                "target": asdict(observation.target), "poses": poses,
                "requires_mid_approach_remeasurement": True, "physical_push_success": None}

    def execute(self):
        started = self.now()
        face = None
        try:
            self.phase("measure")
            first = self.localizer.observe()
            self.preview(first)  # 腕を有効化する前に全段階と復帰経路を検査する。
            face = self.check_observation(first)
            self.motion.enable()
            self.phase("approach")
            self.move(self.raised_point(face), face, 4.)
            self.move(face-(self.radius+0.10)*FORWARD, face, 4.)
            self.move(face-(self.radius+self.settings.clearance)*FORWARD, face)
            self.phase("remeasure")
            observation = self.localizer.observe()
            face = self.check_observation(observation)
            before = np.array(observation.target.center_xy)
            self.move(face-(self.radius+self.settings.clearance)*FORWARD, face)
            self.refresh()
            if np.linalg.norm(self.kin.tip_position()-(face-(self.radius+self.settings.clearance)*FORWARD)) > 0.008:
                raise ValueError("ボトル手前のFK位置合わせ誤差が8mmを超えました")
            self.phase("push")
            settings = SmoothNormalSettings(press_speed_m_s=0.005, completion_tolerance_m=0.0005)
            servo = ContinuousNormalController(face, FORWARD, self.radius, self.settings.push_distance,
                                                face-(self.radius+self.settings.clearance)*FORWARD, settings,
                                                travel_validator=validate_push_distance)
            joint_filter = JointCommandFilter(self.motion.current_target(), settings)
            lateral_correction = np.zeros(3)

            def read_tip():
                self.refresh()
                return self.kin.tip_position()

            def send_goal(xyz, dt, phase):
                # 画像の目標は固定し、重力で生じる横・高さのFK誤差だけを補償する。
                # 球状の手先が胴体の中心線から外れて横へ転がすことを避ける。
                error = face-self.kin.tip_position()
                error -= (error @ FORWARD)*FORWARD
                if np.linalg.norm(error) > 0.008:
                    raise ValueError("ボトル中心線と手先の横方向の差が8mmを超えました")
                lateral_correction[:] += 2.*dt*error
                size = np.linalg.norm(lateral_correction)
                if size > 0.005:
                    lateral_correction[:] *= 0.005/size
                solution = self.kin.solve(xyz+lateral_correction, FORWARD)
                goal = joint_filter.next_goal(solution.joint_angles, dt)
                self.kin.validate_path(goal, face, FORWARD, phase == "press")
                self.motion.stream(goal)
                return goal

            self.report["normal_result"] = run_continuous_normal(
                servo, self.now, self.motion.wait, read_tip, send_goal, 60., self.report["normal_steps"].append)
            self.report["motion_completed"] = True
            self.phase("retract")
            self.move(face-(self.radius+self.settings.clearance)*FORWARD, face, contact=True)
            # 再認識の前に手を横へ退避させ、対象付近から腕を除く。
            self.move(self.raised_point(face), face, 3.)
            self.phase("verify")
            self.localizer.expect_translation()
            try:
                after = self.localizer.observe()
                self.check_observation(after)
                displacement = np.array(after.target.center_xy)-before
                self.report["measured_displacement_m"] = displacement.tolist()
                self.report["visual_success"] = bool(abs(displacement[0]-self.settings.push_distance) <= 0.01
                                                     and abs(displacement[1]) <= 0.01)
                if not self.report["visual_success"]:
                    self.report["verification_error"] = "画像で確認したボトルの移動量が許容範囲外です"
            except (ValueError, RuntimeError) as exc:
                self.report["verification_error"] = str(exc)
            self.phase("home")
            self.refresh()
            self.kin.validate_path(self.motion.home)
            delta = np.max(np.abs(np.array(self.motion.home)-self.motion.current_target()))
            self.motion.move(self.motion.home, max(4., math.sqrt(6*delta/2.)), settle_s=0.3)
            self.refresh()
            if np.max(np.abs(self.kin.data.qpos[self.kin.qadr]-self.motion.home)) > 0.05:
                raise ValueError("腕が初期姿勢へ復帰していません")
            self.report["home_completed"] = True
            self.report["completed"] = True
            self.phase("done")
        except Exception as exc:
            self.report["error"] = str(exc)
            if face is not None:
                try:
                    self.phase("abort_retract")
                    self.move(face-(self.radius+self.settings.clearance)*FORWARD, face, contact=True)
                except Exception as retreat_error:
                    self.report["retreat_error"] = str(retreat_error)
        finally:
            self.motion.close()
            if self.motion.release_error:
                self.report["release_error"] = self.motion.release_error
                self.report["completed"] = False
            self.report["elapsed_time_s"] = self.now()-started
            self.report["measurements"] = self.localizer.measurements
            self.report["measurement_count"] = len(self.localizer.measurements)
            self.report["perception_rejections"] = dict(self.localizer.rejections)
        return self.report
