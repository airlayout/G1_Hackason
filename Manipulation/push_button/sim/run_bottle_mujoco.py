#!/usr/bin/env python3
"""RGB-D + YOLOで台上の自由ボトルを押す。骨盤固定による腕単体の物理検証。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bottle import (BottleFrame, BottleLocalizer, BottlePushRoutine, add_bottle_arguments,
                    add_table, make_bottle_detector, settings_from_args)
from kinematics import ArmKinematics, arm_addresses, load_robot
from trajectory import interpolate, validate_arm_q
from vision import CameraIntrinsics


def build_scene(root, settings, x, y):
    add_table(root, settings)
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "960")
    global_visual.set("offheight", "540")
    quality = visual.find("quality")
    if quality is None:
        quality = ET.SubElement(visual, "quality")
    quality.set("numslices", "64")
    quality.set("numstacks", "16")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", rgba="0.28 0.30 0.32 1")
    ET.SubElement(world, "light", pos="-0.5 -0.8 2.5", dir="0.3 0.4 -1", directional="true",
                  diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "light", pos="0.6 0.4 2.0", dir="-0.2 -0.4 -1", directional="true",
                  diffuse="0.5 0.5 0.5")
    # カメラの向きは初期配置から固定。動くボトルを真値で追尾しない。
    target = np.array([x, y, settings.table_height+0.10])
    for name, position, fovy in (("bottle_rgbd", [-0.12, -0.62, 1.19], 48),
                                  ("bottle_demo", [-0.65, -1.0, 1.48], 53)):
        position = np.array(position)
        back = position-target
        back /= np.linalg.norm(back)
        right = np.cross([0, 0, 1], back)
        right /= np.linalg.norm(right)
        up = np.cross(back, right)
        ET.SubElement(world, "camera", name=name, pos=" ".join(map(str, position)),
                      xyaxes=" ".join(map(str, [*right, *up])), fovy=str(fovy))
    bottle = ET.SubElement(world, "body", name="push_bottle", pos=f"{x} {y} {settings.table_height+0.105}")
    ET.SubElement(bottle, "freejoint", name="bottle_free")
    # 半分程度の水が入った500mL容器の模擬値。剛体でありPETのたわみは扱わない。
    ET.SubElement(bottle, "inertial", pos="0 0 -0.025", mass="0.35", diaginertia="0.0011 0.0011 0.0002")
    ET.SubElement(bottle, "geom", name="bottle_collision", type="cylinder", size="0.034 0.10",
                  rgba="0.56 0.85 0.89 1", friction="0.3 0.005 0.0001", condim="4")
    ET.SubElement(bottle, "geom", name="bottle_shoulder", type="ellipsoid", pos="0 0 0.095",
                  size="0.034 0.034 0.03", rgba="0.56 0.85 0.89 1", contype="0", conaffinity="0", mass="0")
    ET.SubElement(bottle, "geom", name="bottle_neck", type="cylinder", pos="0 0 0.125",
                  size="0.014 0.017", rgba="0.58 0.85 0.89 1", contype="0", conaffinity="0", mass="0")
    ET.SubElement(bottle, "geom", name="bottle_cap", type="cylinder", pos="0 0 0.146",
                  size="0.016 0.008", rgba="0.04 0.19 0.65 1", contype="0", conaffinity="0", mass="0")
    ET.SubElement(bottle, "geom", name="bottle_label", type="cylinder", pos="0 0 -0.012",
                  size="0.0342 0.037", rgba="0.93 0.96 0.98 1", contype="0", conaffinity="0", mass="0")
    for z in (-0.045, 0.021):
        ET.SubElement(bottle, "geom", type="cylinder", pos=f"0 0 {z}", size="0.0345 0.004",
                      rgba="0.03 0.28 0.70 1", contype="0", conaffinity="0", mass="0")


class Simulation:
    def __init__(self, args, settings):
        self.args, self.settings = args, settings
        self.model, rest = load_robot(Path(args.model).expanduser().resolve(), tuple(args.tip_offset),
                                     0.79, args.tip_radius,
                                     scene_builder=lambda root: build_scene(root, settings, args.bottle_x, args.bottle_y))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:29] = rest
        self.data.ctrl[:] = rest
        mujoco.mj_forward(self.model, self.data)
        self.qadr, _, self.actuator = arm_addresses(self.model)
        self.home = rest[self.qadr].copy()
        self.target = self.home.copy()
        self.release_error = None
        self.frame_id = 0
        self.segmentation = None
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)
        self.video_renderer = mujoco.Renderer(self.model, height=540, width=960) if args.video_out else None
        self.video = None
        self.next_video_t = 0.
        self.max_table_force = 0.
        self.snapshots = []
        if args.video_out:
            import cv2
            path = Path(args.video_out)
            path.parent.mkdir(parents=True, exist_ok=True)
            codec = "VP80" if path.suffix.lower() == ".webm" else "mp4v"
            self.video = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), args.video_fps, (960, 540))
            if not self.video.isOpened():
                raise RuntimeError("動画エンコーダを開けません")
        self.wait(0.5)
        body = self.model.body("push_bottle").id
        self.initial_bottle_position = self.data.xpos[body].copy()

    def now(self):
        return self.data.time

    def wait(self, duration):
        end = self.now()+duration
        while self.now() < end-1e-9:
            self.data.ctrl[self.actuator] = self.target
            mujoco.mj_step(self.model, self.data)
            self._evaluate_contacts()
            if self.video is not None and self.now() >= self.next_video_t:
                self.video_renderer.update_scene(self.data, camera="bottle_demo")
                frame = self.video_renderer.render()
                self.video.write(frame[:, :, ::-1])
                self.next_video_t += 1/self.args.video_fps

    def _evaluate_contacts(self):
        # 診断だけ。接触の真値は動作の制御入力には渡さない。
        table = self.model.geom("push_table").id
        bottle = self.model.body("push_bottle").id
        for i, contact in enumerate(self.data.contact):
            pair = [contact.geom1, contact.geom2]
            if table in pair:
                other = pair[1] if pair[0] == table else pair[0]
                if self.model.geom_bodyid[other] != bottle:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, i, force)
                    self.max_table_force = max(self.max_table_force, float(np.linalg.norm(force[:3])))

    def current_target(self):
        return self.target.copy()

    def enable(self):
        pass  # 腕・骨盤は開始時から位置サーボ。実機の重み切替の検証ではない。

    def move(self, goal, duration, settle_s=0.3):
        start = tuple(self.target)
        for q in interpolate(start, tuple(goal), duration, 50.):
            self.stream(q)
            self.wait(0.02)
        self.wait(settle_s)

    def stream(self, goal):
        self.target = np.array(validate_arm_q(np.asarray(goal).tolist()))

    def close(self):
        # Routineのfinallyから呼ばれる。画像・動画は評価と保存後に閉じる。
        pass

    def read_frame(self, after, deadline):
        self.wait(max(0.10, after-self.now()+0.10))
        if self.now() >= deadline:
            raise RuntimeError("模擬RGB-D計測の制限時間超過")
        self.renderer.update_scene(self.data, camera="bottle_rgbd")
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        if self.args.detector == "oracle":
            self.renderer.enable_segmentation_rendering()
            self.segmentation = self.renderer.render().copy()
            self.renderer.disable_segmentation_rendering()
        camera = self.model.camera("bottle_rgbd").id
        transform = np.eye(4)
        transform[:3, :3] = self.data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1])
        transform[:3, 3] = self.data.cam_xpos[camera]
        focal = 480/(2*math.tan(math.radians(self.model.cam_fovy[camera])/2))
        self.frame_id += 1
        frame = BottleFrame(rgb[:, :, ::-1].copy(), depth, CameraIntrinsics(focal, focal, 319.5, 239.5),
                            transform, self.now(), self.frame_id)
        self.last_frame = frame
        return frame

    def oracle_detect(self, image):
        ids = np.flatnonzero(self.model.geom_bodyid == self.model.body("push_bottle").id)
        mask = np.isin(self.segmentation[:, :, 0], ids) & (self.segmentation[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM)
        ys, xs = np.where(mask)
        if len(ys) < 30:
            return []
        return [SimpleNamespace(class_name="bottle", confidence=1., bbox=(float(xs.min()), float(ys.min()),
                                                                       float(xs.max()+1), float(ys.max()+1)))]

    def evaluation(self, report):
        body = self.model.body("push_bottle").id
        displacement = self.data.xpos[body]-self.initial_bottle_position
        rotation = self.data.xmat[body].reshape(3, 3)
        tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1, 1))))
        success = (report["completed"] and report["visual_success"] is True
                   and abs(displacement[0]-self.settings.push_distance) <= 0.01
                   and abs(displacement[1]) <= 0.01 and tilt <= 15 and self.max_table_force < 0.1)
        return {"ground_truth_used_for_control": False, "base": "fixed_pelvis_arm_only",
                "bottle_displacement_m": displacement.tolist(), "bottle_tilt_deg": tilt,
                "max_unwanted_table_force_n": self.max_table_force, "success": bool(success)}

    def dispose(self):
        self.renderer.close()
        if self.video_renderer:
            self.video_renderer.close()
        if self.video:
            self.video.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--detector", choices=("yolo", "oracle"), default="yolo",
                        help="oracleはセグメンテーション正解枠を使う切り分け専用")
    parser.add_argument("--bottle-x", type=float, default=0.33)
    parser.add_argument("--bottle-y", type=float, default=-0.25)
    parser.add_argument("--tip-offset", type=float, nargs=3, default=(0.110, -0.003, 0.))
    parser.add_argument("--tip-radius", type=float, default=0.015)
    parser.add_argument("--report-out", required=True)
    parser.add_argument("--video-out")
    parser.add_argument("--video-fps", type=int, default=12)
    parser.add_argument("--snapshot", help="認識画像PNGの保存先。初回と押した後を保存")
    add_bottle_arguments(parser, simulation=True)
    args = parser.parse_args(argv)
    sim = routine = None
    report = {"schema": "g1-bottle-push-v1", "task": "bottle", "completed": False, "detector": args.detector}
    try:
        settings = settings_from_args(args)
        if (not 1 <= args.video_fps <= 30 or not np.all(np.isfinite([args.bottle_x, args.bottle_y]))):
            raise ValueError("ボトル座標または動画fpsが不正です")
        # 検出器の初期化は物理時間の進行前に行う。
        detector = make_bottle_detector(args.weights, args.confidence, args.device) if args.detector == "yolo" else None
        sim = Simulation(args, settings)
        if detector is None:
            detector = SimpleNamespace(detect=sim.oracle_detect)
        model, _ = load_robot(Path(args.model).expanduser().resolve(), tuple(args.tip_offset), 0.79, args.tip_radius,
                              scene_builder=lambda root: add_table(root, settings))
        kin = ArmKinematics(model)

        def refresh():
            # 腕の計画モデルには計測した機体関節だけを渡す。ボトルの自由関節は渡さない。
            kin.update(sim.data.qpos[:29].copy())

        refresh()
        localizer = BottleLocalizer(sim.read_frame, detector, settings, sim.now, args.target_pixel)
        original_observe = localizer.observe

        def observe_with_snapshot(*a, **kw):
            observation = original_observe(*a, **kw)
            if args.snapshot:
                import cv2
                path = Path(args.snapshot)
                suffix = "after" if localizer.after_push else f"before{len(localizer.measurements)}"
                path = path.with_name(f"{path.stem}_{suffix}{path.suffix}")
                path.parent.mkdir(parents=True, exist_ok=True)
                image = localizer.last_frame.color_bgr.copy()
                x0, y0, x1, y1 = map(int, observation.target.bbox)
                cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(image, f"{args.detector}: bottle", (x0, max(16, y0-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                if not cv2.imwrite(str(path), image):
                    raise OSError(f"画像保存に失敗: {path}")
                sim.snapshots.append(str(path))
            return observation

        localizer.observe = observe_with_snapshot
        routine = BottlePushRoutine(kin, sim, localizer, refresh, sim.now, settings, args.tip_radius)
        report = routine.execute()
        report["detector"] = args.detector
        report["evaluation"] = sim.evaluation(report)
        report["snapshots"] = sim.snapshots
        report["physical_push_success"] = report["evaluation"]["success"]
        report["runtime"] = {"mujoco": mujoco.__version__, "numpy": np.__version__,
                             "model_sha256": hashlib.sha256(Path(args.model).expanduser().read_bytes()).hexdigest()}
        if args.detector == "yolo":
            from importlib.metadata import version
            weights = Path(args.weights) if args.weights else Path(__file__).resolve().parents[3] / "yolo26n.pt"
            report["runtime"]["ultralytics"] = version("ultralytics")
            report["runtime"]["weights"] = str(weights)
            if weights.is_file():
                report["runtime"]["weights_sha256"] = hashlib.sha256(weights.read_bytes()).hexdigest()
        print(json.dumps({k: v for k, v in report.items() if k not in ("normal_steps", "measurements")}, indent=2))
        return 0 if report["evaluation"]["success"] else 1
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        report["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if sim is not None:
            sim.dispose()
        path = Path(args.report_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    raise SystemExit(main())
