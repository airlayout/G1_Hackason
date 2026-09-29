#!/usr/bin/env python3
"""29DoF G1 で、高さ1 mの仮想エレベーターボタンを押す。

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
from trajectory import ARM_JOINTS, DEFAULT_DURATIONS, PHASES, interpolate, validate_arm_q


TIP_RADIUS = 0.015
BUTTON_HALF_DEPTH = 0.010
BUTTON_RADIUS = 0.025
DEFAULT_TIP_OFFSET = (0.110, -0.003, 0.0)  # 右の固定手先。実機では測り直す。


def build_model(model_path: Path, button_x: float, button_y: float, height: float,
                stroke: float, tip_offset: tuple[float, float, float], fixed_base: bool
                ) -> tuple[mujoco.MjModel, np.ndarray]:
    """G1 29DoF モデルに固定接触点、可動ボタン、壁を追加する。"""
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
    ET.SubElement(world, "light", name="button_scene_light", directional="true",
                  pos="-0.5 -1.0 2.5", dir="0.3 0.4 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", name="button_scene_fill", directional="true",
                  pos="0.5 1.0 2.0", dir="-0.2 -0.5 -1", diffuse="0.4 0.4 0.4")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05",
                  rgba="0.2 0.2 0.2 1")
    ET.SubElement(world, "geom", name="elevator_panel", type="box",
                  pos=f"{button_x + 0.045} {button_y} {height}", size="0.015 0.13 0.22",
                  rgba="0.45 0.48 0.52 1")
    ET.SubElement(world, "geom", name="button_indicator", type="sphere",
                  pos=f"{button_x + 0.025} {button_y} {height + 0.08}",
                  size="0.012", rgba="0.9 0.1 0.1 1", contype="0", conaffinity="0")
    button = ET.SubElement(world, "body", name="elevator_button",
                           pos=f"{button_x} {button_y} {height}")
    ET.SubElement(button, "joint", name="button_slide", type="slide", axis="1 0 0",
                  limited="true", range=f"0 {stroke}", stiffness="350", damping="3")
    ET.SubElement(button, "geom", name="button_face", type="cylinder",
                  quat="0.70710678 0 0.70710678 0",
                  size=f"{BUTTON_RADIUS} {BUTTON_HALF_DEPTH}", mass="0.035",
                  rgba="0.95 0.65 0.05 1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    expected_nq = 30 if fixed_base else 37
    if model.nu != 29 or model.nq != expected_nq:
        raise ValueError(f"想定外のモデル構成: nu={model.nu}, nq={model.nq}")
    model.opt.timestep = 0.002
    return model, stand_q


def arm_addresses(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joints = [model.joint(name) for name in ARM_JOINTS]
    qadr = np.array([joint.qposadr[0] for joint in joints], dtype=int)
    dadr = np.array([joint.dofadr[0] for joint in joints], dtype=int)
    actuator = np.array([model.actuator(name).id for name in ARM_JOINTS], dtype=int)
    return qadr, dadr, actuator


def solve_ik(model: mujoco.MjModel, data: mujoco.MjData, target: np.ndarray,
             seed: np.ndarray, rest: np.ndarray, qadr: np.ndarray,
             dadr: np.ndarray) -> np.ndarray:
    """手先位置と前向き姿勢を、右腕7軸の減衰付き最小二乗で解く。"""
    site_id = model.site("button_tcp").id
    q = seed.copy()
    lower = model.jnt_range[[model.joint(name).id for name in ARM_JOINTS], 0] + 0.025
    upper = model.jnt_range[[model.joint(name).id for name in ARM_JOINTS], 1] - 0.025
    desired_axis = np.array([1.0, 0.0, 0.0])
    for _ in range(350):
        data.qpos[qadr] = q
        mujoco.mj_forward(model, data)
        position_error = target - data.site_xpos[site_id]
        axis = data.site_xmat[site_id].reshape(3, 3)[:, 0]
        if np.linalg.norm(position_error) < 0.0015 and np.dot(axis, desired_axis) > 0.92:
            return q
        jac_pos = np.zeros((3, model.nv))
        jac_rot = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
        # d(axis)/dq = angular_velocity × axis
        axis_jac = np.cross(jac_rot[:, dadr].T, axis).T
        matrix = np.vstack((jac_pos[:, dadr], 0.10 * axis_jac, 0.003 * np.eye(7)))
        residual = np.concatenate((position_error, 0.10 * (desired_axis - axis),
                                   0.003 * (rest - q)))
        step = np.linalg.lstsq(matrix, residual, rcond=None)[0]
        step = np.clip(step, -0.10, 0.10)
        q = np.clip(q + step, lower, upper)
    data.qpos[qadr] = q
    mujoco.mj_forward(model, data)
    error = np.linalg.norm(target - data.site_xpos[site_id])
    raise ValueError(f"手先位置のIKが収束しません (誤差 {error:.3f} m): {target}")


def run(args: argparse.Namespace) -> dict:
    path = Path(args.model).expanduser().resolve()
    model, stand_q = build_model(path, args.button_x, args.button_y,
                                 args.height, args.stroke, tuple(args.tip_offset),
                                 fixed_base=args.fixed_base)
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
    # ボタンは +X の壁面にあり、G1 は +X を向く。右腕を使用する。
    face_x = args.button_x - BUTTON_HALF_DEPTH
    contact_center_x = face_x - TIP_RADIUS
    approach_target = np.array([contact_center_x - args.clearance,
                                args.button_y, args.height])
    contact_target = np.array([contact_center_x - 0.003,
                               args.button_y, args.height])
    press_target = np.array([contact_center_x + args.stroke,
                             args.button_y, args.height])
    poses: dict[str, list[float]] = {}
    seed = rest.copy()
    for phase, target in (("approach", approach_target), ("contact", contact_target),
                          ("press", press_target)):
        seed = solve_ik(model, data, target, seed, rest, qadr, dadr)
        poses[phase] = [float(value) for value in seed]
        validate_arm_q(poses[phase])
    poses["retract"] = poses["approach"].copy()
    poses["home"] = [float(value) for value in rest]

    if not args.fixed_base:
        data.qpos[:7] = (0.0, 0.0, 0.79, 1.0, 0.0, 0.0, 0.0)
        data.qpos[7:36] = stand_q
    else:
        data.qpos[:29] = stand_q
    data.qpos[model.joint("button_slide").qposadr[0]] = 0.0
    data.ctrl[:] = stand_q
    mujoco.mj_forward(model, data)
    button_adr = model.joint("button_slide").qposadr[0]
    indicator_id = model.geom("button_indicator").id
    max_stroke = 0.0
    min_pelvis_z = float(data.body("pelvis").xpos[2])
    max_base_drift = 0.0
    min_upright_cos = 1.0
    viewer = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
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
                if not args.fixed_base:
                    min_pelvis_z = min(min_pelvis_z, float(data.qpos[2]))
                    max_base_drift = max(max_base_drift,
                                         float(np.linalg.norm(data.qpos[:2])))
                    min_upright_cos = min(min_upright_cos,
                                          float(data.body("pelvis").xmat[8]))
                if max_stroke >= 0.90 * args.stroke:
                    model.geom_rgba[indicator_id] = (0.1, 0.9, 0.15, 1.0)
                if viewer is not None:
                    if not viewer.is_running():
                        raise RuntimeError("viewer が閉じられました")
                    viewer.sync()
                    time.sleep(max(0.0, viewer_start + data.time - time.perf_counter()))
            current = goal
            print(f"[{phase}] button={data.qpos[button_adr]*1000:.1f} mm, "
                  f"max={max_stroke*1000:.1f} mm", flush=True)
    finally:
        if viewer is not None:
            viewer.close()
    # 可動端の数値誤差も監視する。大きく押し込み過ぎた試行は成功扱いにしない。
    success = (0.90 * args.stroke <= max_stroke <= 1.10 * args.stroke
               and min_pelvis_z > 0.65 and max_base_drift < 0.10
               and min_upright_cos > math.cos(math.radians(20)))
    max_tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, min_upright_cos))))
    plan = {
        "schema": "g1-push-button-plan-v1",
        "source_model_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "arm_joint_names": list(ARM_JOINTS),
        "button": {"x": args.button_x, "y": args.button_y,
                   "height": args.height, "stroke": args.stroke,
                   "clearance": args.clearance},
        "tip_offset": args.tip_offset,
        "poses": poses,
        "durations": DEFAULT_DURATIONS,
        "sim_result": {"success": success, "max_stroke_m": max_stroke,
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
    parser.add_argument("--stroke", type=float, default=0.008, help="ボタンのストローク [m]")
    parser.add_argument("--clearance", type=float, default=0.030, help="待機距離 [m]")
    parser.add_argument("--tip-offset", type=float, nargs=3,
                        default=list(DEFAULT_TIP_OFFSET), metavar=("X", "Y", "Z"),
                        help="右手首座標系における固定手先の接触点 [m]")
    parser.add_argument("--plan-out", help="実機向け関節経路の出力先 JSON")
    parser.add_argument("--viewer", action="store_true", help="MuJoCo viewer を表示")
    parser.add_argument("--fixed-base", action="store_true",
                        help="骨盤を固定し、腕単体の接触試験をする")
    args = parser.parse_args()
    if not (0.2 <= args.button_x <= 0.5 and -0.45 <= args.button_y <= -0.10
            and 0.7 <= args.height <= 1.2 and 0.002 <= args.stroke <= 0.015
            and 0.01 <= args.clearance <= 0.08
            and 0.04 <= args.tip_offset[0] <= 0.16
            and all(abs(value) <= 0.05 for value in args.tip_offset[1:])):
        parser.error("ボタン位置またはストロークが許容範囲外です")
    try:
        plan = run(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0 if plan["sim_result"]["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
