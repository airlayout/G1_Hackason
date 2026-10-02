#!/usr/bin/env python3
"""29DoF G1 で、上下2つの仮想エレベーターボタンを押す。

MuJoCo Menagerie の unitree_g1/g1.xml を使用する。既定は自由立位、
--fixed-base は腕単体の切り分け用。歩行コントローラの検証ではない。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trajectory import (ARM_JOINTS, DEFAULT_BUTTON_STROKE_M, DEFAULT_DURATIONS,
                        PHASES, interpolate, validate_arm_q, validate_button_stroke)
from kinematics import IkSolution, arm_addresses, solve_ik


TIP_RADIUS = 0.015
BUTTON_HALF_DEPTH = 0.010
BUTTON_RADIUS = 0.032
BUTTON_VERTICAL_SPACING = 0.14
DEFAULT_TIP_OFFSET = (0.110, -0.003, 0.0)  # 右の固定手先。実機では測り直す。

def build_model(model_path: Path, button_x: float, button_y: float, height: float,
                stroke: float, tip_offset: tuple[float, float, float], fixed_base: bool,
                rgbd_camera: bool = False, button_direction: str = "up",
                panel_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
                panel_yaw_deg: float = 0.0,
                rgbd_position: str = "-0.10 -0.45 1.14",
                rgbd_fovy: float = 60.0,
                ) -> tuple[mujoco.MjModel, np.ndarray]:
    """G1 29DoF モデルに、写真を参考にした上下の可動ボタンを追加する。"""
    stroke = validate_button_stroke(stroke)
    if button_direction not in ("up", "down"):
        raise ValueError("ボタン方向は up または down を指定してください")
    root = ET.parse(model_path).getroot()
    if root.get("model", "").find("29dof") < 0:
        raise ValueError("29DoF の g1.xml を指定してください")
    compiler = root.find("compiler")
    if compiler is None:
        raise ValueError("MJCF compiler がありません")
    compiler.set("meshdir", str((model_path.parent / "assets").resolve()))
    stand = root.find("./keyframe/key[@name='stand']")
    if stand is None or stand.get("ctrl") is None:
        raise ValueError("stand キーフレームを持つ Menagerie モデルが必要です")
    stand_q = np.fromstring(stand.get("ctrl"), sep=" ")
    if stand_q.size != 29:
        raise ValueError("stand の関節数が29ではありません")
    pelvis = root.find(".//body[@name='pelvis']")
    if pelvis is None or pelvis.find("freejoint") is None:
        raise ValueError("pelvis/freejoint が見つかりません")
    if fixed_base:
        pelvis.remove(pelvis.find("freejoint"))
    root.remove(root.find("keyframe"))

    wrist = root.find(".//body[@name='right_wrist_yaw_link']")
    if wrist is None:
        raise ValueError("右手首が見つかりません")
    tip_pos = " ".join(str(value) for value in tip_offset)
    ET.SubElement(wrist, "site", name="button_tcp", pos=tip_pos, size="0.006",
                  rgba="0 1 0 1")
    # 元モデルのゴム手は visual のみ。実際に接触する球を追加する。
    ET.SubElement(wrist, "geom", name="button_tip", type="sphere", pos=tip_pos,
                  size=str(TIP_RADIUS), mass="0.01", rgba="0.2 0.7 0.9 1")

    world = root.find("worldbody")
    if rgbd_camera:
        # 検証用のカメラ。実機の取り付け位置・姿勢を再現した値ではない。
        ET.SubElement(world, "camera", name="button_rgbd", mode="targetbody",
                      target=f"elevator_button_{button_direction}",
                      pos=rgbd_position, fovy=str(rgbd_fovy))
        ET.SubElement(world, "camera", name="demo_camera", mode="targetbody",
                      target=f"elevator_button_{button_direction}",
                      pos="-0.45 -0.72 1.38", fovy="55")
    ET.SubElement(world, "light", name="button_scene_light", directional="true",
                  pos="-0.5 -1.0 2.5", dir="0.3 0.4 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", name="button_scene_fill", directional="true",
                  pos="0.5 1.0 2.0", dir="-0.2 -0.5 -1", diffuse="0.4 0.4 0.4")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05",
                  rgba="0.2 0.2 0.2 1")
    angle = math.radians(panel_yaw_deg) / 2
    world = ET.SubElement(world, "body", name="elevator_panel_frame",
                          pos=" ".join(map(str, panel_offset)),
                          quat=f"{math.cos(angle)} 0 0 {math.sin(angle)}")
    # 写真は寸法資料ではない。パネル、ボタン間隔、銀色の縁は模擬寸法。
    panel_center_z = height - BUTTON_VERTICAL_SPACING / 2
    ET.SubElement(world, "geom", name="elevator_panel", type="box",
                  pos=f"{button_x + 0.020} {button_y} {panel_center_z}",
                  size="0.012 0.115 0.34", rgba="0.16 0.17 0.17 1",
                  contype="0", conaffinity="0")
    # 可動ボタンの裏をパネル衝突形状で塞がないよう、2つの開口を残す。
    panel_parts = (
        (button_y - .080, panel_center_z, .035, .34),
        (button_y + .080, panel_center_z, .035, .34),
        (button_y, height + .14, .045, .10),
        (button_y, height - BUTTON_VERTICAL_SPACING / 2, .045, .025),
        (button_y, height - BUTTON_VERTICAL_SPACING - .14, .045, .10),
    )
    for index, (y, z, half_y, half_z) in enumerate(panel_parts):
        ET.SubElement(world, "geom", name=f"panel_collision_{index}", type="box",
                      pos=f"{button_x + 0.020} {y} {z}",
                      size=f"0.012 {half_y} {half_z}", rgba="0 0 0 0")
    for side in (-1, 1):
        ET.SubElement(world, "geom", name=f"panel_side_{side}", type="box",
                      pos=f"{button_x + 0.021} {button_y + side * 0.122} {panel_center_z}",
                      size="0.014 0.007 0.34", rgba="0.72 0.71 0.67 1",
                      contype="0", conaffinity="0")

    def stroke_geom(name: str, y0: float, z0: float, y1: float, z1: float,
                    radius: float, rgba: str, x: float) -> None:
        ET.SubElement(world, "geom", name=name, type="capsule",
                      fromto=f"{x} {y0} {z0} {x} {y1} {z1}",
                      size=str(radius), rgba=rgba, contype="0", conaffinity="0")

    # RF の表記と左側の案内記号。細い装飾ジオメトリで描き、接触判定には使わない。
    label_x = button_x + 0.007
    label_z = height + 0.105
    white = "0.93 0.93 0.90 1"
    glyphs = [
        (-.006, -.016, -.006, .016), (-.006, .016, .007, .016),
        (.007, .016, .012, .010), (.012, .010, .007, .003),
        (.007, .003, -.006, .003), (.001, .003, .014, -.016),  # R
        (.021, -.016, .021, .016), (.021, .016, .039, .016),
        (.021, .002, .035, .002),  # F
    ]
    for index, (y0, z0, y1, z1) in enumerate(glyphs):
        stroke_geom(f"label_stroke_{index}", button_y - y0, label_z + z0,
                    button_y - y1, label_z + z1, .0015, white, label_x)
    for index, (y0, z0, y1, z1) in enumerate((
            (-.046, -.014, -.023, .014), (-.046, .014, -.023, -.014))):
        stroke_geom(f"label_symbol_{index}", button_y - y0, label_z + z0,
                    button_y - y1, label_z + z1, .0012, white, label_x)
    for index, (y, z) in enumerate(((-.0345, -.019), (-.0345, .019),
                                    (-.052, 0), (-.017, 0))):
        ET.SubElement(world, "geom", name=f"label_dot_{index}", type="sphere",
                      pos=f"{label_x} {button_y - y} {label_z + z}",
                      size="0.002", rgba=white, contype="0", conaffinity="0")

    for direction, z in (("up", height), ("down", height - BUTTON_VERTICAL_SPACING)):
        # 外周の暗い影と銀色の縁。白い中心面だけがスライドする。
        ET.SubElement(world, "geom", name=f"button_shadow_{direction}",
                      type="cylinder", quat="0.70710678 0 0.70710678 0",
                      pos=f"{button_x + 0.003} {button_y} {z}", size="0.042 0.007",
                      rgba="0.055 0.06 0.06 1", contype="0", conaffinity="0")
        ET.SubElement(world, "geom", name=f"button_bezel_{direction}",
                      type="cylinder", quat="0.70710678 0 0.70710678 0",
                      pos=f"{button_x - 0.004} {button_y} {z}", size="0.039 0.004",
                      rgba="0.75 0.77 0.75 1", contype="0", conaffinity="0")
        button = ET.SubElement(world, "body", name=f"elevator_button_{direction}",
                               pos=f"{button_x} {button_y} {z}")
        ET.SubElement(button, "joint", name=f"button_slide_{direction}",
                      type="slide", axis="1 0 0", limited="true",
                      range=f"0 {stroke}", stiffness="350", damping="3",
                      # 短いストロークの機械終端。既定の軟らかい制約だと
                      # 1.5mmの上限を超えて沈む。2ms刻みで安定な4msの時定数。
                      solreflimit="0.004 1", solimplimit="0.99 0.99 0.0001")
        ET.SubElement(button, "geom", name=f"button_face_{direction}",
                      type="cylinder", quat="0.70710678 0 0.70710678 0",
                      size=f"{BUTTON_RADIUS} {BUTTON_HALF_DEPTH}", mass="0.035",
                      rgba="0.91 0.90 0.84 1")
        arrow_tip = .018 if direction == "up" else -.018
        arrow_tail = -.014 if direction == "up" else .014
        arrow_shoulder = .006 if direction == "up" else -.006
        arrow_x = -BUTTON_HALF_DEPTH - .0001
        for index, (y0, z0, y1, z1) in enumerate((
                (0, arrow_tail, 0, arrow_tip),
                (-.012, arrow_shoulder, 0, arrow_tip),
                (.012, arrow_shoulder, 0, arrow_tip))):
            # 印刷した矢印を薄い箱で表す。太いカプセルだと深度上で約4mm
            # 突出し、表面位置を誤って計測してしまう。
            angle = math.atan2(-(y1-y0), z1-z0) / 2
            length = math.hypot(y1-y0, z1-z0)
            ET.SubElement(button, "geom", name=f"arrow_{direction}_{index}",
                          type="box", pos=f"{arrow_x} {(y0+y1)/2} {(z0+z1)/2}",
                          quat=f"{math.cos(angle)} {math.sin(angle)} 0 0",
                          size=f"0.00005 0.0028 {length/2}", rgba="0.025 0.025 0.025 1",
                          contype="0", conaffinity="0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    expected_nq = 31 if fixed_base else 38
    if model.nu != 29 or model.nq != expected_nq:
        raise ValueError(f"想定外のモデル構成: nu={model.nu}, nq={model.nq}")
    model.opt.timestep = 0.002
    return model, stand_q



def run(args: argparse.Namespace, target_face_xyz: np.ndarray | None = None,
        press_direction: np.ndarray | None = None) -> dict:
    path = Path(args.model).expanduser().resolve()
    button_direction = getattr(args, "button_direction", "up")
    if button_direction not in ("up", "down"):
        raise ValueError("ボタン方向は up または down を指定してください")
    target_height = (args.height if button_direction == "up"
                     else args.height - BUTTON_VERTICAL_SPACING)
    gif_out = getattr(args, "gif_out", None)
    gif_fps = getattr(args, "gif_fps", 10)
    model, stand_q = build_model(path, args.button_x, args.button_y,
                                 args.height, args.stroke, tuple(args.tip_offset),
                                 fixed_base=args.fixed_base,
                                 rgbd_camera=bool(gif_out),
                                 button_direction=button_direction)
    data = mujoco.MjData(model)
    if not args.fixed_base:
        data.qpos[:7] = (0.0, 0.0, 0.79, 1.0, 0.0, 0.0, 0.0)
        data.qpos[7:36] = stand_q
    else:
        data.qpos[:29] = stand_q
    data.ctrl[:] = stand_q
    mujoco.mj_forward(model, data)
    qadr, dadr, actuator = arm_addresses(model)
    rest = data.qpos[qadr].copy()
    # 真のボタンは args の位置に残す。視覚推定値だけで腕の経路を作ることで
    # 位置推定誤差が実際の押下成否に反映される。
    if target_face_xyz is None:
        target_face_xyz = np.array([args.button_x - BUTTON_HALF_DEPTH,
                                    args.button_y, target_height], dtype=float)
    else:
        target_face_xyz = np.asarray(target_face_xyz, dtype=float)
    if press_direction is None:
        press_direction = np.array([1.0, 0.0, 0.0])
    else:
        press_direction = np.asarray(press_direction, dtype=float)
    if (target_face_xyz.shape != (3,) or press_direction.shape != (3,)
            or not np.all(np.isfinite(target_face_xyz))
            or not np.all(np.isfinite(press_direction))
            or np.linalg.norm(press_direction) < 0.9):
        raise ValueError("視覚目標の位置または押下方向が不正です")
    press_direction /= np.linalg.norm(press_direction)
    if np.dot(press_direction, [1.0, 0.0, 0.0]) < 0.9:
        raise ValueError("この模擬パネルは +X 方向からしか押せません")
    contact_center = target_face_xyz - TIP_RADIUS * press_direction
    approach_target = contact_center - args.clearance * press_direction
    contact_target = contact_center - 0.003 * press_direction
    press_target = contact_center + args.stroke * press_direction
    poses: dict[str, list[float]] = {}
    ik_validation: dict[str, dict[str, float | int]] = {}
    seed = rest.copy()
    for phase, target in (("approach", approach_target), ("contact", contact_target),
                          ("press", press_target)):
        solution = solve_ik(model, data, target, seed, rest, qadr, dadr,
                            desired_axis=press_direction,
                            position_tolerance_m=min(0.0001, args.stroke / 50))
        seed = solution.joint_angles
        poses[phase] = [float(value) for value in seed]
        validate_arm_q(poses[phase])
        ik_validation[phase] = {
            "position_error_m": solution.position_error_m,
            "axis_error_deg": solution.axis_error_deg,
            "min_joint_margin_rad": solution.min_joint_margin_rad,
            "iterations": solution.iterations,
        }
        print(f"[ik:{phase}] 位置誤差={solution.position_error_m*1000:.2f} mm, "
              f"方向誤差={solution.axis_error_deg:.2f} 度, "
              f"関節余裕={solution.min_joint_margin_rad:.3f} rad")
    poses["retract"] = poses["approach"].copy()
    poses["home"] = [float(value) for value in rest]

    if not args.fixed_base:
        data.qpos[:7] = (0.0, 0.0, 0.79, 1.0, 0.0, 0.0, 0.0)
        data.qpos[7:36] = stand_q
    else:
        data.qpos[:29] = stand_q
    for direction in ("up", "down"):
        data.qpos[model.joint(f"button_slide_{direction}").qposadr[0]] = 0.0
    data.ctrl[:] = stand_q
    mujoco.mj_forward(model, data)
    button_adr = model.joint(f"button_slide_{button_direction}").qposadr[0]
    other_direction = "down" if button_direction == "up" else "up"
    other_button_adr = model.joint(f"button_slide_{other_direction}").qposadr[0]
    face_id = model.geom(f"button_face_{button_direction}").id
    max_stroke = 0.0
    max_other_stroke = 0.0
    min_pelvis_z = float(data.body("pelvis").xpos[2])
    max_base_drift = 0.0
    min_upright_cos = 1.0
    viewer = None
    renderer = None
    gif_frames = []
    next_gif_time = 0.0
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
    if gif_out:
        from PIL import Image, ImageDraw

        renderer = mujoco.Renderer(model, height=480, width=640)

        def capture_gif_frame(phase: str) -> None:
            renderer.update_scene(data, camera="demo_camera")
            frame = Image.fromarray(renderer.render().copy())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((8, 8, 250, 39), fill=(10, 10, 10))
            draw.text((16, 17), f"{phase}  t={data.time:.1f}s", fill=(255, 255, 255))
            gif_frames.append(frame.quantize(colors=128))

        capture_gif_frame("start")
        next_gif_time = 1.0 / gif_fps
    current = rest.copy()
    viewer_start = time.perf_counter()
    try:
        for phase in PHASES:
            goal = np.asarray(poses[phase])
            duration = DEFAULT_DURATIONS[phase]
            for q_target in interpolate(tuple(current), tuple(goal), duration,
                                        hz=1.0 / model.opt.timestep):
                data.ctrl[actuator] = q_target
                mujoco.mj_step(model, data)
                max_stroke = max(max_stroke, float(data.qpos[button_adr]))
                max_other_stroke = max(max_other_stroke,
                                       float(data.qpos[other_button_adr]))
                if not args.fixed_base:
                    min_pelvis_z = min(min_pelvis_z, float(data.qpos[2]))
                    max_base_drift = max(max_base_drift,
                                         float(np.linalg.norm(data.qpos[:2])))
                    min_upright_cos = min(min_upright_cos,
                                          float(data.body("pelvis").xmat[8]))
                if max_stroke >= 0.90 * args.stroke:
                    model.geom_rgba[face_id] = (0.95, 0.81, 0.49, 1.0)
                if viewer is not None:
                    if not viewer.is_running():
                        raise RuntimeError("viewer が閉じられました")
                    viewer.sync()
                    time.sleep(max(0.0, viewer_start + data.time - time.perf_counter()))
                if renderer is not None and data.time >= next_gif_time:
                    capture_gif_frame(phase)
                    next_gif_time += 1.0 / gif_fps
            current = goal
            print(f"[{phase}] button={data.qpos[button_adr]*1000:.1f} mm, "
                  f"max={max_stroke*1000:.1f} mm", flush=True)
    finally:
        if viewer is not None:
            viewer.close()
        if renderer is not None:
            renderer.close()
    if gif_out:
        gif_path = Path(gif_out)
        gif_path.parent.mkdir(parents=True, exist_ok=True)
        gif_frames[0].save(gif_path, save_all=True,
                           append_images=gif_frames[1:],
                           duration=round(1000 / gif_fps), loop=0)
        print(f"MuJoCoアニメーション: {gif_path} ({len(gif_frames)}フレーム)")
    # 可動端の数値誤差も監視する。大きく押し込み過ぎた試行は成功扱いにしない。
    success = (0.90 * args.stroke <= max_stroke <= 1.10 * args.stroke
               and max_other_stroke < 0.10 * args.stroke
               and min_pelvis_z > 0.65 and max_base_drift < 0.10
               and min_upright_cos > math.cos(math.radians(20)))
    max_tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, min_upright_cos))))
    plan = {
        "schema": "g1-push-button-plan-v1",
        "source_model_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "arm_joint_names": list(ARM_JOINTS),
        "button": {"x": args.button_x, "y": args.button_y,
                   "height": target_height, "upper_height": args.height,
                   "direction": button_direction, "stroke": args.stroke,
                   "clearance": args.clearance},
        "vision_target": {"face_xyz": target_face_xyz.tolist(),
                          "press_direction": press_direction.tolist()},
        "ik_validation": ik_validation,
        "tip_offset": args.tip_offset,
        "poses": poses,
        "durations": DEFAULT_DURATIONS,
        "sim_result": {"success": success, "max_stroke_m": max_stroke,
                       "max_other_stroke_m": max_other_stroke,
                       "base_fixed": args.fixed_base,
                       "min_pelvis_height_m": min_pelvis_z,
                       "max_base_drift_m": max_base_drift,
                       "max_tilt_deg": max_tilt_deg},
    }
    print(f"RESULT_{'SUCCESS' if success else 'FAIL'}: "
          f"最大押下量 {max_stroke*1000:.1f} / {args.stroke*1000:.1f} mm, "
          f"最低骨盤高 {min_pelvis_z:.3f} m, 最大移動 {max_base_drift:.3f} m, "
          f"最大傾き {max_tilt_deg:.1f}°", flush=True)
    if args.plan_out:
        output = Path(args.plan_out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")
        print(f"経路を保存: {output}", flush=True)
    return plan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerie の unitree_g1/g1.xml")
    parser.add_argument("--button-x", type=float, default=0.36,
                        help="ロボット前方のボタン中心 X [m]")
    parser.add_argument("--button-y", type=float, default=-0.25,
                        help="ボタン中心 Y [m]。右側は負")
    parser.add_argument("--height", type=float, default=1.0, help="ボタン中心の床上高さ [m]")
    parser.add_argument("--button-direction", choices=("up", "down"), default="up",
                        help="押すボタン。--height は上ボタンの中心高さ")
    parser.add_argument("--stroke", type=float, default=DEFAULT_BUTTON_STROKE_M,
                        help="ボタンのストローク [m] (既定: 0.0015)")
    parser.add_argument("--clearance", type=float, default=0.030, help="待機距離 [m]")
    parser.add_argument("--tip-offset", type=float, nargs=3,
                        default=list(DEFAULT_TIP_OFFSET), metavar=("X", "Y", "Z"),
                        help="右手首座標系における固定手先の接触点 [m]")
    parser.add_argument("--plan-out", help="実機向け関節経路の出力先 JSON")
    parser.add_argument("--target-json", help="real/localize_button.py の校正済み3D目標 JSON")
    parser.add_argument("--viewer", action="store_true", help="MuJoCo viewer を表示")
    parser.add_argument("--gif-out", help="押下動作をGIFで保存")
    parser.add_argument("--gif-fps", type=int, default=10)
    parser.add_argument("--fixed-base", action="store_true",
                        help="骨盤を固定し、腕単体の接触試験をする")
    args = parser.parse_args()
    if not 1 <= args.gif_fps <= 30:
        parser.error("--gif-fps は1～30で指定してください")
    target_face = None
    press_direction = None
    if args.target_json:
        try:
            with Path(args.target_json).open(encoding="utf-8") as stream:
                target = json.load(stream)
            if (target.get("schema") != "g1-button-target-v1"
                    or target.get("frame") != "robot_base"
                    or target.get("valid_frames", 0) < 3
                    or target.get("max_position_spread_m", math.inf) > 0.015):
                raise ValueError("校正済みで安定したボタン目標が必要です")
            target_face = np.asarray(target["face_xyz_m"], dtype=float)
            press_direction = np.asarray(target["press_direction"], dtype=float)
            if target_face.shape != (3,) or not np.all(np.isfinite(target_face)):
                raise ValueError("ボタン表面の3D位置が不正です")
            args.button_x = float(target_face[0] + BUTTON_HALF_DEPTH)
            args.button_y = float(target_face[1])
            args.height = float(target_face[2] + (BUTTON_VERTICAL_SPACING
                                                   if args.button_direction == "down" else 0))
        except (OSError, KeyError, TypeError, ValueError) as error:
            parser.error(f"--target-json: {error}")
    if not (0.2 <= args.button_x <= 0.5 and -0.45 <= args.button_y <= -0.10
            and 0.7 <= args.height <= 1.2 and 0.0015 <= args.stroke <= 0.015
            and 0.01 <= args.clearance <= 0.08
            and 0.04 <= args.tip_offset[0] <= 0.16
            and all(abs(value) <= 0.05 for value in args.tip_offset[1:])):
        parser.error("ボタン位置またはストロークが許容範囲外です")
    try:
        plan = run(args, target_face_xyz=target_face,
                   press_direction=press_direction)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0 if plan["sim_result"]["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
