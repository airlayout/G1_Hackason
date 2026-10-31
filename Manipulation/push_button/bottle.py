"""台上の直立ボトルのRGB-D計測と、既存の腕制御を使う押し動作。"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from itertools import product
from xml.etree import ElementTree as ET
import math
import argparse
from time import perf_counter

import numpy as np

from alignment import (ContinuousNormalController,
                       SmoothNormalSettings, run_continuous_normal)
from vision import CameraIntrinsics, _points
from return_path import validate_return_path
from cartesian import CartesianJointController
from trial_policy import ObservationUnavailable, TrialPause


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
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--detector-imgsz", type=int, default=960, help="YOLO入力サイズ（32の正の倍数）")
    if not simulation:
        parser.add_argument("--gravity-scale", type=float, default=0.0, help="右腕の重力補償倍率（0〜1、既定は無効）")
        parser.add_argument("--track-table", action=argparse.BooleanOptionalAction, default=False,
                            help="台面と二辺のRGB-Dから機体の並進を追跡する")
        parser.add_argument("--dynamics-feedforward",action="store_true",help="重力補償と併せて公称の摩擦・速度補償も使う")
        parser.add_argument("--stop-before-contact", action="store_true", help="接触前の位置合わせ後、押さずに復帰・解除する診断")
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
    if args.detector_imgsz <= 0 or args.detector_imgsz % 32:
        raise ValueError("detector-imgszは32の正の倍数が必要です")
    scale = getattr(args, "gravity_scale", 0.0)
    if not np.isfinite(scale) or not 0 <= scale <= 1:
        raise ValueError("gravity-scaleは0〜1が必要です")
    return BottleSettings(args.table_height, args.push_distance, args.contact_height,
                          args.clearance, tuple(args.table_bounds))


def make_bottle_detector(weights=None, confidence=0.25, device="auto", *, runtime=None, imgsz=960):
    # J1-genとPerceptionのcommonパッケージ名を衝突させない。
    from real.j1gen_camera import J1CameraRuntime

    if device == 'cpu':
        import torch
        torch.set_num_threads(2)
    runtime = runtime or J1CameraRuntime()
    detector = runtime.module("perception_bridge").perception("detector.yolo_detector").YoloDetector
    local = Path(__file__).resolve().parents[2] / "yolo26n.pt"
    model = str(local) if weights is None and local.is_file() else (weights or "yolo26n.pt")
    return detector(model_name=model, classes=["bottle"], confidence_threshold=confidence, device=device, imgsz=imgsz)


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


def estimate_bottle_target(depth_m, bbox, intrinsics: CameraIntrinsics, transform, settings, *, allow_occlusion=False):
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
    if allow_occlusion:
        # 接触中は手が下部を隠す。複数の高さで円筒を検証できる胴体だけを使う。
        band = points[(points[:, 2] > height+0.025) & (points[:, 2] < height+0.085)]
    if len(band) < 40:
        raise ValueError("接触高さに有効なボトル胴体の深度がありません")
    center, radius, fraction = _fit_circle(band)
    # 胴体が複数の高さで同じ円筒に乗ることを確認。平面・大きな傾きは拒否する。
    body = (band if allow_occlusion else points[(points[:, 2] > settings.table_height+0.025)
                  & (points[:, 2] < height+0.055)])
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

    def __init__(self, read_frame, detector, settings, now, target_pixel=None, *, position_stability_m=.004):
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
        self.diagnostics = []
        self.samples = deque(maxlen=3)
        self.after = -np.inf
        self.tracking = False
        self.progress_m = 0.
        if not np.isfinite(position_stability_m) or not .001 <= position_stability_m <= .02:
            raise ValueError('位置の安定許容幅は1〜20mmが必要です')
        self.position_stability_m = position_stability_m

    def begin(self, after, *, tracking=False):
        self.after, self.tracking = after, tracking
        self.samples.clear()

    def consume(self, frame):
        """1枚を処理する。非同期処理も同期のobserveと同じ判定を使う。"""
        samples = self.samples
        if (not np.isfinite(frame.captured_at) or frame.sequence <= self.last_sequence
                or frame.captured_at <= max(self.after, self.last_capture)
                or not 0 <= self.now()-frame.captured_at <= 0.5):
            samples.clear()
            self.rejections["古い時刻または重複フレーム"] += 1
            return None
        self.last_capture, self.last_sequence = frame.captured_at, frame.sequence
        candidates = []
        inference_started = perf_counter()
        detections = self.detector.detect(frame.color_bgr)
        row = {"sequence": frame.sequence, "captured_at_s": frame.captured_at,
               "age_before_fit_s": self.now()-frame.captured_at,
               "geometry_rejections": [], "targets": [], "tracking": self.tracking,
               "inference_wall_s": perf_counter()-inference_started, "fit_wall_s": [],
               "detections": [{"class": d.class_name, "bbox": list(d.bbox),
                               "confidence": getattr(d, "confidence", None)} for d in detections]}
        self.diagnostics.append(row)
        if not any(d.class_name == "bottle" for d in detections):
            self.rejections["ボトル未検出"] += 1
        for detection in detections:
            if detection.class_name != "bottle":
                continue
            if self.previous is None and self.target_pixel is not None:
                u, v = self.target_pixel
                x0, y0, x1, y1 = detection.bbox
                if not (x0 <= u <= x1 and y0 <= v <= y1):
                    continue
            try:
                fit_started = perf_counter()
                target = estimate_bottle_target(frame.depth_m, detection.bbox, frame.intrinsics,
                    frame.transform, self.settings, allow_occlusion=self.tracking)
                validate_bottle_target(target, self.settings, require_travel=not (self.after_push or self.tracking))
            except ValueError as exc:
                self.rejections["深度または対象幾何の棄却"] += 1
                self.rejections[str(exc)] += 1
                row["geometry_rejections"].append(str(exc))
                continue
            finally:
                row['fit_wall_s'].append(perf_counter()-fit_started)
            row["targets"].append(asdict(target))
            if self.previous is not None:
                expected = np.array(self.expected_xy if self.expected_xy is not None else self.previous.center_xy)
                if self.tracking:
                    expected[0] += self.progress_m
                if (np.linalg.norm(np.array(target.center_xy)-expected) > 0.025
                        or abs(target.radius_m-self.previous.radius_m) > 0.005):
                    self.rejections["追跡位置または半径が一致しません"] += 1
                    continue
            candidates.append(target)
        age = self.now()-frame.captured_at
        row["age_after_fit_s"] = age
        row["candidate_count"] = len(candidates)
        if not 0 <= age <= 0.5:
            samples.clear()
            self.rejections["処理後の画像鮮度超過"] += 1
            return None
        if len(candidates) > 1:
            raise ValueError("ボトルを一意に選べません。--target-pixelを指定してください")
        if not candidates:
            samples.clear()
            return None
        samples.append(candidates[0])
        self.last_frame = frame
        if len(samples) < 3:
            return None
        positions = np.array([s.center_xy for s in samples])
        center = np.median(positions, axis=0)
        if (np.max(np.linalg.norm(positions-center, axis=1)) > self.position_stability_m
                or np.ptp([s.radius_m for s in samples]) > 0.003):
            self.rejections["フレーム間の位置または半径が不安定"] += 1
            return None
        radius = float(np.median([s.radius_m for s in samples]))
        stable = replace(samples[-1], center_xy=tuple(center), radius_m=radius,
                         face_xyz=(float(center[0]-radius), float(center[1]), samples[-1].face_xyz[2]))
        if not self.tracking:
            self.previous = stable
        obs = BottleObservation(stable, frame.captured_at, frame.sequence)
        self.measurements.append({"captured_at_s": frame.captured_at, "sequence": frame.sequence,
                                 "after_push": self.after_push, "tracking": self.tracking, "target": asdict(stable)})
        return obs

    def expect_translation(self):
        if self.previous is None:
            raise ValueError("移動前のボトル計測がありません")
        self.expected_xy = np.array(self.previous.center_xy) + self.settings.push_distance * FORWARD[:2]
        self.after_push = True

    def observe(self, timeout_s=5.):
        after = self.now()
        deadline = after+timeout_s
        self.begin(after)
        while self.now() < deadline:
            try:
                frame = self.read_frame(after, deadline)
            except RuntimeError:
                self.rejections["フレーム取得期限超過"] += 1
                raise
            observation = self.consume(frame)
            if observation is not None:
                return observation
        self.rejections["安定計測の期限超過"] += 1
        raise RuntimeError("安定した新しい3フレームのボトル計測を取得できません（対象消失・深度・鮮度を確認）")


class BottlePushRoutine:
    """motionは既存のArmMotionと同じAPI。シミュレーションも同じ制御を通る。"""

    # Only the first post-plan check uses this age limit; image/track checks stay independent.
    initial_plan_max_age_s = 0.5

    def __init__(self, kin, motion, localizer, refresh, now, settings, tip_radius,
                 position_uncertainty_m=0., *, tracking_check=None, live_tracking=False, stop_before_contact=False,
                 pause_handler=None):
        self.kin, self.motion, self.localizer = kin, motion, localizer
        self.refresh, self.now, self.settings = refresh, now, settings
        self.radius = tip_radius
        self.preview_step_m = 0.005
        self.position_uncertainty_m = position_uncertainty_m
        self.tracking_check = tracking_check
        self.live_tracking = live_tracking
        self.stop_before_contact = stop_before_contact
        self.pause_handler = pause_handler
        self.arm_enabled = False
        self.last_face = None
        self.report = {"schema": "g1-bottle-push-v1", "task": "bottle", "settings": asdict(settings),
                       "completed": False, "motion_completed": False, "home_completed": False,
                       "visual_success": None, "measured_displacement_m": None,
                       "physical_push_success": None, "phases": [], "normal_steps": []}
        self.report.update(retreat_completed=False, release_completed=False, ik_checks=[])
        self.report.update(live_tracking=live_tracking, diagnostic_completed=False,
                           stop_before_contact=stop_before_contact, tracking_steps=[],
                           initial_plan_max_age_s=self.initial_plan_max_age_s)
        self.report['ik_searches'] = []

    def solve_pose(self, xyz, face, *, contact=False):
        """Try bounded numerical seeds, validating every path from the real start."""
        original = self.kin.data.qpos.copy()
        row = {'phase': self.report.get('phase', 'preview'), 'target_xyz_m': np.asarray(xyz).tolist(),
               'allow_tip_contact': contact, 'attempts': [], 'accepted': False}
        self.report['ik_searches'].append(row)
        offsets = [(0., 0., 0.)]+[v for v in product((-.4, 0., .4), (-.4, 0., .4), (-.6, 0., .6))
                                   if v != (0., 0., 0.)]
        try:
            for index, offset in enumerate(offsets):
                attempt = {'shoulder_seed_offset_rad': list(offset)}
                row['attempts'].append(attempt)
                try:
                    if index == 0:
                        solution = self.kin.solve(np.asarray(xyz), FORWARD)
                    else:
                        seed = original[self.kin.qadr].copy()
                        seed[:3] += offset
                        solution = self.kin.solve(np.asarray(xyz), FORWARD, seed=seed)
                    # A numerical seed is not an executed waypoint.
                    self.kin.update(original)
                    self.kin.validate_path(solution.joint_angles, face, FORWARD, contact)
                except ValueError as exc:
                    attempt['reason'] = str(exc)
                else:
                    attempt['accepted'] = row['accepted'] = True
                    return solution
                finally:
                    self.kin.update(original)
        finally:
            self.kin.update(original)
        raise ValueError(f'経路検査付きIKが{len(offsets)}候補で失敗しました: {row["attempts"][-1]["reason"]}')

    def move(self, xyz, face, duration=2., contact=False):
        self.refresh()
        solution = self.solve_pose(xyz, face, contact=contact)
        self.refresh()
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

    def check_observation(self, observation, *, max_age_s=0.5):
        if not 0 <= self.now()-observation.captured_at <= max_age_s:
            error = TrialPause if self.pause_handler is not None else ValueError
            raise error("ボトル計測が古すぎます")
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
                start = self.kin.tip_position()
                count = max(1, math.ceil(np.linalg.norm(xyz-start)/self.preview_step_m)) if name == "push" else 1
                for point in np.linspace(start, xyz, count+1)[1:]:
                    solution = self.solve_pose(point, face, contact=name in ("push", "retract"))
                    goal = solution.joint_angles
                    self.report["ik_checks"].append({"phase": name, "target_xyz_m": point.tolist(),
                        "position_error_m": solution.position_error_m, "axis_error_deg": solution.axis_error_deg,
                        "min_joint_margin_rad": solution.min_joint_margin_rad})
                    q = self.kin.data.qpos.copy()
                    q[self.kin.qadr] = goal
                    self.kin.update(q)
                poses[name] = goal.tolist()
            home = np.array(self.motion.home) if self.motion is not None else original[self.kin.qadr]
            self.kin.validate_path(home)
            poses["home"] = home.tolist()
        finally:
            self.kin.update(original)
        return {"schema": "g1-bottle-preview-v1", "task": "bottle", "settings": asdict(self.settings),
                "target": asdict(observation.target), "poses": poses,
                "ik_searches": self.report['ik_searches'],
                "requires_mid_approach_remeasurement": True, "physical_push_success": None}

    def check_home(self):
        self.refresh()
        if np.max(np.abs(self.kin.data.qpos[self.kin.qadr]-self.motion.home)) > 0.05:
            raise ValueError("腕が初期姿勢へ復帰していません")
        self.report["home_completed"] = True

    def return_after_contact(self, face):
        self.phase("retract")
        self.move(face-(self.radius+self.settings.clearance)*FORWARD, face, contact=True)
        self.report["retreat_completed"] = True
        self.move(self.raised_point(face), face, 3.)

    def return_home(self):
        self.phase("home")
        self.refresh()
        self.kin.validate_path(self.motion.home)
        delta = np.max(np.abs(np.array(self.motion.home)-self.motion.current_target()))
        self.motion.move(self.motion.home, max(4., math.sqrt(6*delta/2.)), settle_s=0.3)
        self.check_home()

    def settle_before_push(self, face, settings):
        """停止後の1サンプルだけでなく、実測TCPの収束を確認してから接触へ進む。"""
        goal = face-(self.radius+self.settings.clearance)*FORWARD
        controller = CartesianJointController(self.motion.current_target(), settings)
        started = previous = self.now()
        correction = np.zeros(3)
        settled = 0
        rows = self.report.setdefault('alignment_steps', [])
        while self.now()-started < 8.:
            self.refresh()
            self.motion.check()
            if self.tracking_check is not None:
                self.tracking_check()
            stamp = self.now()
            dt = settings.period_s if not rows else stamp-previous
            previous = stamp
            error = goal-self.kin.tip_position()
            rows.append({'time_s': stamp, 'dt_s': dt, 'error_xyz_m': error.tolist(),
                         'base_transform': self.kin.body_transform('pelvis').tolist()})
            if not 0 < dt <= .1:
                raise ValueError('接触前の状態更新が100ms以上途絶えました')
            if np.linalg.norm(error) > .008:
                raise ValueError('ボトル手前のFK位置合わせ誤差が8mmを超えました')
            settled = settled+1 if np.linalg.norm(error) <= .003 else 0
            if settled >= 3:
                return controller
            correction += 2.*dt*error
            size = np.linalg.norm(correction)
            if size > .005:
                correction *= .005/size
            target = goal+correction
            if np.linalg.norm(target-self.kin.tip_position()) > .01:
                raise ValueError('手先の指令と実測の差が1cmを超えました')
            joints = controller.next_goal(self.kin, target, FORWARD, dt)
            self.kin.validate_path(joints, face, FORWARD)
            if self.now()-stamp > .1:
                raise ValueError('接触前の指令計算が100msを超えました')
            self.motion.stream(joints, velocity=controller.velocity)
            rows[-1].update(command_xyz_m=target.tolist(), sent_at_s=self.now(), **controller.diagnostics)
            self.motion.wait(max(.002, settings.period_s-(self.now()-stamp)))
        raise ValueError('接触前のFK位置合わせが制限時間内に収束しません')

    def reverse_approach(self, face):
        self.phase("abort_return")
        self.refresh()
        self.motion.check()
        path = self.motion.approach
        if len(path.samples) == 1:
            self.check_home()
            return
        if not path.frozen:
            self.motion.freeze_approach()
        targets = path.reversed_targets()
        validate_return_path(self.kin, targets, face, FORWARD)
        delta = np.max(np.abs(np.array(targets[0])-self.kin.data.qpos[self.kin.qadr]))
        self.motion.move(targets[0], max(0.3, 1.5*delta/0.8, math.sqrt(6*delta/2.)), settle_s=0.3)

        def validate(goal):
            self.refresh()
            self.kin.validate_path(np.asarray(goal), face, FORWARD)

        self.motion.replay(targets, validate)
        self.motion.wait(0.3)
        self.check_home()
        self.report["retreat_completed"] = True

    def prepare_contact(self):
        """再開のたびに新しい対象と実測姿勢から接近区間を計画する。"""
        self.phase('measure')
        first = self.localizer.observe()
        self.preview(first)
        face = self.check_observation(first, max_age_s=self.initial_plan_max_age_s)
        self.last_face = face
        if not self.arm_enabled:
            self.motion.enable()
            self.arm_enabled = True
            self.motion.start_approach()
        self.phase('approach')
        self.move(self.raised_point(face), face, 4.)
        self.move(face-(self.radius+0.10)*FORWARD, face, 4.)
        self.move(face-(self.radius+self.settings.clearance)*FORWARD, face)
        self.phase('remeasure')
        observation = self.localizer.observe()
        face = self.check_observation(observation)
        self.last_face = face
        before = np.array(observation.target.center_xy)
        self.preview(observation)
        self.move(face-(self.radius+self.settings.clearance)*FORWARD, face)
        self.refresh()
        alignment_error = self.kin.tip_position()-(face-(self.radius+self.settings.clearance)*FORWARD)
        self.report['alignment_error_xyz_m'] = alignment_error.tolist()
        if np.linalg.norm(alignment_error) > .008:
            raise ValueError('ボトル手前のFK位置合わせ誤差が8mmを超えました')
        settings = SmoothNormalSettings(approach_speed_m_s=.01, press_speed_m_s=.004,
                                        completion_tolerance_m=.0015, completion_hold_s=.04)
        if self.live_tracking and not self.stop_before_contact:
            self.localizer.start_tracking(observation)
        return before, settings, self.settle_before_push(face, settings)

    def field_return(self, *, after_push=False):
        """古い監視エラーを再送出せず、現在姿勢から復帰を検査する。"""
        self.refresh()
        self.motion.check()
        if not self.arm_enabled:
            self.check_home()
            return
        if self.last_face is not None:
            if after_push and not self.report.get('retreat_completed'):
                self.return_after_contact(self.last_face)
            else:
                self.phase('abort_return')
                self.move(self.raised_point(self.last_face), self.last_face, 3.)
        self.return_home()

    def field_action(self, error, *, after_push=False):
        if self.live_tracking:
            self.localizer.stop_tracking()
        pause = error if isinstance(error, TrialPause) else TrialPause(str(error), code='observation')
        action = self.pause_handler(pause, allow_resume=not after_push)
        # 押下後はハンドラーが誤ってresumeを返しても再押下しない。
        if action == 'resume' and not after_push:
            if hasattr(self.localizer, 'reset_measurement_reference'):
                self.localizer.reset_measurement_reference()
            return True
        if action == 'return':
            self.field_finish_return(after_push=after_push)
        self.report['cancelled'] = True
        return False

    def field_finish_return(self, *, after_push=False):
        while True:
            try:
                self.field_return(after_push=after_push)
                self.report['operator_returned'] = True
                return True
            except (TrialPause, ObservationUnavailable) as exc:
                pause = exc if isinstance(exc, TrialPause) else TrialPause(str(exc), code='observation')
                action = self.pause_handler(pause, allow_resume=False)
                if action != 'return':
                    self.report['cancelled'] = True
                    return False

    def execute(self):
        started = self.now()
        face = None
        pushing = False
        try:
            while True:
                try:
                    before, settings, joint_filter = self.prepare_contact()
                    break
                except (TrialPause, ObservationUnavailable) as exc:
                    if self.pause_handler is None:
                        raise
                    if not self.field_action(exc):
                        return self.report
            face = self.last_face
            if self.stop_before_contact:
                if self.pause_handler is not None:
                    if not self.field_finish_return():
                        return self.report
                else:
                    self.reverse_approach(face)
                self.report['diagnostic_completed'] = True
                self.phase('done')
                return self.report
            self.motion.freeze_approach()
            pushing = True
            self.phase("push")
            initial_command = self.kin.tip_position()
            # 法線方向は実測位置から開始し、横・高さは実際に送る中心線指令と
            # 一致させる。鮮度監視の指令差に古い横位置を残さない。
            initial_command[1:] = face[1:]
            servo = ContinuousNormalController(face, FORWARD, self.radius, self.settings.push_distance,
                                                initial_command, settings,
                                                travel_validator=validate_push_distance,
                                                position_gain=.8, tracking_lead_m=.007, latch_completion=True)
            lateral_correction = np.zeros(3)
            visual_correction = np.zeros(3)
            tracked_center = before.copy()
            command_diagnostics = {}

            def read_tip():
                self.refresh()
                if self.tracking_check is not None:
                    self.tracking_check()
                tip = self.kin.tip_position()
                if self.live_tracking:
                    progress = max(0., servo.penetration(tip))
                    tracked = self.localizer.latest_tracking(progress)
                    tracked_center[:] = tracked.target.center_xy
                    # A sphere behind the original line drives a displaced cylinder
                    # farther away. Follow its current centre plus restoring feedback.
                    visual_correction[1] = 2.*(tracked.target.center_xy[1]-before[1])
                    visual_correction[1] = np.clip(visual_correction[1], -.005, .005)
                    self.report['tracking_steps'].append({'time_s': self.now(),
                        'captured_at_s': tracked.captured_at, 'sequence': tracked.sequence,
                        'center_xy_m': list(tracked.target.center_xy),
                        'visual_correction_m': visual_correction.tolist()})
                self.report['last_control_state'] = {'time_s':self.now(),'tip_xyz_m':tip.tolist(),
                    'base_transform':self.kin.body_transform('pelvis').tolist(),
                    'motor_q_rad':self.kin.data.qpos.tolist(),
                    'command_xyz_m':servo.command_xyz.tolist()}
                return tip

            def send_goal(xyz, dt, phase):
                # 前方の終点は固定し、横方向の画像補正とFK補償の合計を制限する。
                actual_centerline = face.copy()
                actual_centerline[1] = tracked_center[1]
                measured_error = actual_centerline-self.kin.tip_position()
                measured_error[0] = 0.
                if np.linalg.norm(measured_error) > .008:
                    raise ValueError("ボトル中心線と手先の横方向の差が8mmを超えました")
                error = face+visual_correction-self.kin.tip_position()
                error -= (error @ FORWARD)*FORWARD
                if np.linalg.norm(error) > 0.008:
                    raise ValueError("ボトル中心線と手先の横方向の差が8mmを超えました")
                lateral_correction[:] += 2.*dt*error
                total_correction = lateral_correction+visual_correction
                size = np.linalg.norm(total_correction)
                if size > 0.005:
                    total_correction *= 0.005/size
                    lateral_correction[:] = total_correction-visual_correction
                centerline_goal = np.asarray(xyz).copy()
                centerline_goal[1:] = face[1:]
                if np.linalg.norm(centerline_goal+total_correction-self.kin.tip_position()) > .01:
                    raise ValueError('手先の指令と実測の差が1cmを超えました')
                goal = joint_filter.next_goal(self.kin,centerline_goal+total_correction,FORWARD,dt)
                command_diagnostics.update(joint_filter.diagnostics,
                    lateral_correction_m=lateral_correction.tolist())
                # 追従差の範囲内で実測TCPが先に接触域へ入った場合も、
                # 許可する接触は手先球だけ。腕・台・自己衝突の検査は継続する。
                near_contact = servo.penetration(self.kin.tip_position()) >= -.003
                self.kin.validate_path(goal, face, FORWARD, phase == "press" or near_contact)
                if self.now()-self.report['last_control_state']['time_s'] > .1:
                    raise ValueError('押下指令の計算が100msを超えました')
                self.motion.stream(goal,velocity=joint_filter.velocity)
                return goal

            def record_step(row):
                self.report["normal_steps"].append(dict(row, **command_diagnostics))

            self.report["normal_result"] = run_continuous_normal(
                servo, self.now, self.motion.wait, read_tip, send_goal, 60., record_step)
            self.report["motion_completed"] = True
            if self.live_tracking:
                self.localizer.stop_tracking()
            self.return_after_contact(face)
            self.return_home()
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
                if self.pause_handler is not None:
                    self.motion.fatal_check()
                    self.localizer.check()
                self.report["verification_error"] = str(exc)
            self.report["completed"] = True
            self.phase("done")
        except Exception as exc:
            self.report["error"] = str(exc)
            if self.live_tracking:
                self.localizer.stop_tracking()
            face = self.last_face
            if self.pause_handler is not None:
                if isinstance(exc, (TrialPause, ObservationUnavailable)):
                    try:
                        self.field_action(exc, after_push=pushing)
                    except Exception as recovery_error:
                        self.report['recovery_error'] = str(recovery_error)
                        self.report['fatal_error'] = True
                else:
                    self.report['fatal_error'] = True
                # 通信・指令・衝突などの異常では追加の復帰動作を送らない。
            elif face is not None and self.arm_enabled:
                try:
                    self.refresh()
                    self.motion.check()
                    if not pushing:
                        self.reverse_approach(face)
                    else:
                        self.return_after_contact(face)
                        self.return_home()
                except Exception as retreat_error:
                    self.report["recovery_error"] = str(retreat_error)
        finally:
            self.motion.close()
            if self.pause_handler is not None and getattr(self.motion, 'error', None) is not None:
                self.report['error'] = str(self.motion.error)
                self.report['fatal_error'] = True
                self.report['completed'] = self.report['diagnostic_completed'] = False
            self.report["release_completed"] = not bool(self.motion.release_error)
            if self.motion.release_error:
                self.report["release_error"] = self.motion.release_error
                self.report["completed"] = False
                self.report['diagnostic_completed'] = False
            self.report["elapsed_time_s"] = self.now()-started
            self.report["measurements"] = self.localizer.measurements
            self.report["measurement_count"] = len(self.localizer.measurements)
            self.report["perception_rejections"] = dict(self.localizer.rejections)
            self.report["perception_frames"] = self.localizer.diagnostics
        return self.report
