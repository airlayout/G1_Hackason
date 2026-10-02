"""Isaac Sim の世界（G1 + 乗り場 + 照明 + カメラ）を組み立てる部品。Isaac Sim のアプリの起動後に import すること。

sim/isaac/view_hall_isaac.py（シーンを見る）と contest/robots/isaac_robot.py（評価）の両方が使う。
エントリポイント（AppLauncher を起動するファイル）は他から import できないので、組み立ての部品はここに置く。
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch
import warp as wp

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.sim import SimulationContext

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import REPO_ROOT  # noqa: E402
from common.j1gen_bridge import j1gen, j1gen_config  # noqa: E402
from common.scene_spec import HallScene, build_hall_scene  # noqa: E402
from isaac_hall import HallIsaac, build_hall, hall_articulation_cfg  # noqa: E402

ROBOT_PATH = "/World/G1"
OVERVIEW_CAMERA = "hall_overview"
# 物理の刻み。MuJoCo 版（既定 0.002）とそろえる
PHYSICS_DT = 0.002
# 画像を作るのは 10 ステップに 1 回（50 Hz）
RENDER_EVERY = 10
# 関節の PD の強さを指定しないときの値（シーンを見るだけのとき。胴体固定で腕と脚が垂れないように）
HOLD_STIFFNESS = 400.0
HOLD_DAMPING = 20.0
# 中指の先の衝突判定の球（MuJoCo 版と同じ）
FINGERTIP_NAME = "fingertip_collision"
# 関節の名前 → configs/contest.yaml の gains の組
GAIN_GROUPS: dict[str, list[str]] = {
    "legs": [".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
    "waist": ["waist_.*"],
    "arm": [".*_shoulder_.*", ".*_elbow_.*"],
    "wrist": [".*_wrist_.*"],
}


def quat_xyzw_from_axes(fwd: np.ndarray, left: np.ndarray, up: np.ndarray) -> tuple[float, float, float, float]:
    """列が (前, 左, 上) の回転行列を四元数 (x, y, z, w) にする（IsaacLab 3.0 は xyzw の順）。"""
    m = np.column_stack([fwd, left, up])
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return (x, y, z, w)


def look_at_xyzw(eye: np.ndarray, target: np.ndarray) -> tuple[float, float, float, float]:
    """CameraCfg の convention="world"（前 +x、上 +z）で、eye から target を見る向き。"""
    fwd = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], fwd)
    left /= np.linalg.norm(left)
    return quat_xyzw_from_axes(fwd, left, np.cross(fwd, left))


def find_link_path(stage: Any, root: str, name: str) -> str:
    """root の下で、名前が name の剛体（リンク）の prim のパス。

    URDF の取り込み方で階層が変わるので探す。同じ名前の見た目のメッシュの prim もあるので、剛体を優先する。
    """
    from pxr import Usd, UsdPhysics

    fallback = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.GetName() == name:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return str(prim.GetPath())
            fallback = fallback or str(prim.GetPath())
    if fallback is None:
        raise RuntimeError(f"{root} の下に {name} が無い")
    return fallback


def add_fingertips(stage: Any, robot_cfg: dict, radius: float) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics

    for side, ee in robot_cfg["end_effector"].items():
        link = find_link_path(stage, ROBOT_PATH, ee["link"])
        sphere = UsdGeom.Sphere.Define(stage, f"{link}/{side}_{FINGERTIP_NAME}")
        sphere.CreateRadiusAttr(radius)
        UsdGeom.XformCommonAPI(sphere).SetTranslate(Gf.Vec3d(*ee["offset"]))
        # guide は画像に映らない（衝突判定は効く）
        sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())


def _actuators(gains_cfg: dict | None) -> dict[str, ImplicitActuatorCfg]:
    if gains_cfg is None:
        return {"hold": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=HOLD_STIFFNESS, damping=HOLD_DAMPING)}
    return {
        group: ImplicitActuatorCfg(joint_names_expr=exprs, stiffness=float(gains_cfg[group]["kp"]),
                                   damping=float(gains_cfg[group]["kd"]))
        for group, exprs in GAIN_GROUPS.items()
    }


def make_robot(robot_cfg: dict, scene_cfg: dict, pelvis_world: np.ndarray, gains_cfg: dict | None = None) -> Articulation:
    """G1 を公式 URDF から読み込む（pelvis 固定）。gains_cfg は configs/contest.yaml の gains（None なら保持用の値）。"""
    from isaaclab.sim.converters import UrdfConverterCfg

    rm = j1gen("robot_model")
    urdf = j1gen("config").resolve_repo_path(robot_cfg["model"]["urdf_path"])
    if not urdf.exists():
        raise FileNotFoundError(f"公式モデルが無い: {urdf}\n先に `bash Button_Press/J1-gen/sim/fetch_models.sh` を実行すること。")
    joint_pos = {n: 0.0 for n in rm.JOINT_NAMES}
    for idx, deg in scene_cfg.get("initial_pose_deg", {}).items():
        joint_pos[rm.JOINT_NAMES[int(idx)]] = math.radians(float(deg))
    cfg = ArticulationCfg(
        prim_path=ROBOT_PATH,
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(urdf),
            usd_dir=str(REPO_ROOT / "_local" / "button_press_yada" / "usd"),
            fix_base=True,
            merge_fixed_joints=True,
            self_collision=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=HOLD_STIFFNESS, damping=HOLD_DAMPING)
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=tuple(pelvis_world.tolist()), joint_pos=joint_pos),
        actuators=_actuators(gains_cfg),
    )
    return Articulation(cfg)


def hold_initial_pose(robot: Articulation) -> None:
    """関節を init_state の姿勢にして、その姿勢を PD の目標にする。

    IsaacLab は init_state を default_joint_pos に入れるだけで、シミュレーションへは書かない
    （普段は環境の reset が書く）。書かないと 0 度の姿勢（肘が曲がり、手が頭カメラの前に来る）のままになる。
    """
    q = wp.to_torch(robot.data.default_joint_pos).clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
    robot.set_joint_position_target_index(target=q)
    robot.write_data_to_sim()


def head_camera_cfg(stage: Any, robot_cfg: dict, data_types: list[str]) -> CameraCfg:
    """頭カメラ（J1-gen の robot.yaml の取り付け位置と画角）。"""
    cam = robot_cfg["head_camera"]
    w, h = int(cam["width"]), int(cam["height"])
    # 垂直画角から焦点距離を決める（ピクセルは正方形）。fy = (h/2) / tan(fovy/2)
    fy = (h / 2.0) / math.tan(math.radians(float(cam["fovy_deg"])) / 2.0)
    aperture_h = 20.955
    focal = fy * aperture_h / w
    torso = find_link_path(stage, ROBOT_PATH, cam["parent_body"])
    roll, pitch, yaw = cam["rpy"]
    if abs(roll) > 1e-9 or abs(yaw) > 1e-9:
        raise ValueError("頭カメラの向きは pitch だけを想定している")
    return CameraCfg(
        prim_path=f"{torso}/head_camera",
        offset=CameraCfg.OffsetCfg(pos=tuple(cam["xyz"]), rot=(0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2)),
                                   convention="world"),
        width=w, height=h, data_types=data_types,
        spawn=sim_utils.PinholeCameraCfg(focal_length=focal, horizontal_aperture=aperture_h, clipping_range=(0.02, 20.0)),
    )


def overview_camera_cfg(scene: HallScene, pelvis_world: np.ndarray) -> CameraCfg:
    """全体を見るカメラ（MuJoCo 版と同じ位置）。"""
    target = pelvis_world + np.array([scene.boxes[0].center[0], scene.buttons[0].face_center[1], 0.1])
    eye = pelvis_world + np.array([-1.3, -1.1, 0.9])
    return CameraCfg(
        prim_path=f"/World/{OVERVIEW_CAMERA}",
        offset=CameraCfg.OffsetCfg(pos=tuple(eye.tolist()), rot=look_at_xyzw(eye, target), convention="world"),
        width=960, height=720, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955, clipping_range=(0.05, 50.0)),
    )


@dataclass
class World:
    sim: SimulationContext
    scene: HallScene
    robot_cfg: dict
    robot: Articulation
    hall_art: Articulation
    hall: HallIsaac
    cameras: dict[str, Camera] = field(default_factory=dict)
    pelvis_world: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def step(self, n: int) -> list[str]:
        """物理を n ステップ進める（RENDER_EVERY ごとに描画してカメラを更新）。押されたボタンの名前を返す。"""
        pressed: list[str] = []
        for i in range(n):
            self.robot.write_data_to_sim()
            self.hall_art.write_data_to_sim()
            render = (i + 1) % RENDER_EVERY == 0
            self.sim.step(render=render)
            self.robot.update(PHYSICS_DT)
            self.hall_art.update(PHYSICS_DT)
            pressed += self.hall.update()
            if render:
                for c in self.cameras.values():
                    c.update(PHYSICS_DT * RENDER_EVERY)
        return pressed

    def close(self) -> None:
        """この世界を片付ける（同じアプリの中で、次の世界を作れるようにする）。"""
        if not self.sim.get_setting("/isaaclab/has_gui"):
            self.sim.stop()
        self.sim.clear_instance()


def build_world(scene_cfg: dict, device: str, gains_cfg: dict | None = None, cameras: bool = True,
                head_data_types: tuple[str, ...] = ("rgb",), overview: bool = True, new_stage: bool = False) -> World:
    """G1 + 乗り場 + 照明 + カメラを作り、sim.reset() して初期の姿勢にした World を返す。

    new_stage=True で新しいステージに作る（評価で試行ごとに作り直すとき）。
    """
    if new_stage:
        sim_utils.create_new_stage()
    scene = build_hall_scene(scene_cfg)
    robot_cfg = j1gen_config("robot.yaml")
    pelvis_world = np.array([0.0, 0.0, scene.pelvis_height])

    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, render_interval=RENDER_EVERY, device=device))
    stage = sim_utils.get_current_stage()
    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    dome = sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 0.95))
    dome.func("/World/DomeLight", dome)
    sun = sim_utils.DistantLightCfg(intensity=1500.0, angle=1.0)
    sun.func("/World/DistantLight", sun)

    robot = make_robot(robot_cfg, scene_cfg, pelvis_world, gains_cfg)
    fingertip = scene_cfg.get("fingertip", {})
    if fingertip.get("enabled"):
        add_fingertips(stage, robot_cfg, float(fingertip["radius"]))
    shaders = build_hall(stage, scene, pelvis_world)
    hall_art = Articulation(hall_articulation_cfg(scene))
    cams: dict[str, Camera] = {}
    if cameras:
        cams[robot_cfg["head_camera"]["name"]] = Camera(head_camera_cfg(stage, robot_cfg, list(head_data_types)))
        if overview:
            cams[OVERVIEW_CAMERA] = Camera(overview_camera_cfg(scene, pelvis_world))

    sim.reset()
    hold_initial_pose(robot)
    hall = HallIsaac(hall_art, scene, shaders)
    return World(sim=sim, scene=scene, robot_cfg=robot_cfg, robot=robot, hall_art=hall_art, hall=hall,
                 cameras=cams, pelvis_world=pelvis_world)


def camera_rgb(c: Camera) -> np.ndarray:
    out = c.data.output["rgb"]
    t = out if isinstance(out, torch.Tensor) else wp.to_torch(out)
    return t[0][..., :3].cpu().numpy().astype(np.uint8)


def save_png(cameras: dict[str, Camera], prefix: Path) -> None:
    from PIL import Image

    prefix.parent.mkdir(parents=True, exist_ok=True)
    for name, c in cameras.items():
        p = prefix.with_name(f"{prefix.name}_{name}.png")
        Image.fromarray(camera_rgb(c)).save(p)
        print(f"[png] {p}")
