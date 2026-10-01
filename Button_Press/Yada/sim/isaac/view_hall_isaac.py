"""Isaac Sim でエレベーター乗り場のシーンを見る・確かめる。run.sh から起動する（直接は起動しない）。

    bash Button_Press/Yada/sim/isaac/run.sh                         # 画面で見る（Shift + ドラッグでボタンを押せる）
    bash Button_Press/Yada/sim/isaac/run.sh --headless --png _local/button_press_yada/isaac_hall
    bash Button_Press/Yada/sim/isaac/run.sh --headless --press up --png _local/button_press_yada/isaac_pressed

G1 は IK（J1-gen の Pinocchio）と同じ公式 URDF（g1_29dof_rev_1_0）から読み込み、pelvis をワールドに固定する。
関節は configs/elevator_hall.yaml の initial_pose_deg の姿勢に保つ（歩行も腕の制御もまだ無いため）。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# --- Isaac Sim の起動は他の import より先に行う ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac Sim のエレベーター乗り場のシーン")
parser.add_argument("--scene", default="elevator_hall.yaml", help="シーンの設定（configs/ 基準）")
parser.add_argument("--png", default="", help="画像を保存する（パスの先頭。_<カメラ名>.png を付けて保存）")
parser.add_argument("--press", default="", help="このボタンをばねの目標を変えて押し、沈み量と点灯を確かめる（up / down）")
parser.add_argument("--max-steps", type=int, default=0, help="画面で見るとき、この物理ステップ数で終了する（0 なら閉じるまで）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- ここから下は Isaac Sim 起動後にのみ import できる ---
import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import REPO_ROOT, load_config  # noqa: E402
from common.j1gen_bridge import j1gen, j1gen_config  # noqa: E402
from common.scene_spec import build_hall_scene  # noqa: E402
from isaac_hall import HallIsaac, build_hall, hall_articulation_cfg  # noqa: E402

ROBOT_PATH = "/World/G1"
# 物理の刻み。MuJoCo 版（既定 0.002）とそろえる。ボタンのばね（1000 N/m、20 g）の周期は約 28 ms
PHYSICS_DT = 0.002
# 画像を作るのは 10 ステップに 1 回（50 Hz）
RENDER_EVERY = 10
# 関節を初期の姿勢に保つ PD の強さ（胴体固定で、腕と脚が重力で垂れないように）
HOLD_STIFFNESS = 400.0
HOLD_DAMPING = 20.0
# 中指の先の衝突判定の球（公式 URDF のハンドの衝突形状は大まかなので、MuJoCo 版と同じ球を足す）
FINGERTIP_NAME = "fingertip_collision"


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


def find_prim_path(stage, root: str, name: str) -> str:
    """root の下で、名前が name の prim のパス（URDF の取り込み方で階層が変わるので探す）。"""
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.GetName() == name:
            return str(prim.GetPath())
    raise RuntimeError(f"{root} の下に {name} が無い")


def add_fingertips(stage, robot_cfg: dict, radius: float) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics

    for side, ee in robot_cfg["end_effector"].items():
        link = find_prim_path(stage, ROBOT_PATH, ee["link"])
        sphere = UsdGeom.Sphere.Define(stage, f"{link}/{side}_{FINGERTIP_NAME}")
        sphere.CreateRadiusAttr(radius)
        UsdGeom.XformCommonAPI(sphere).SetTranslate(Gf.Vec3d(*ee["offset"]))
        # guide は画像に映らない（衝突判定は効く）
        sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())


def make_robot(robot_cfg: dict, scene_cfg: dict, pelvis_world: np.ndarray) -> Articulation:
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
            self_collision=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=HOLD_STIFFNESS, damping=HOLD_DAMPING)
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=tuple(pelvis_world.tolist()), joint_pos=joint_pos),
        actuators={"hold": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=HOLD_STIFFNESS, damping=HOLD_DAMPING)},
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


def make_cameras(stage, robot_cfg: dict, scene, pelvis_world: np.ndarray) -> dict[str, Camera]:
    """頭カメラ（J1-gen の robot.yaml の取り付け位置）と、全体を見るカメラ（MuJoCo 版と同じ位置）。"""
    cam = robot_cfg["head_camera"]
    w, h = int(cam["width"]), int(cam["height"])
    # 垂直画角から焦点距離を決める（ピクセルは正方形）。fy = (h/2) / tan(fovy/2)
    fy = (h / 2.0) / math.tan(math.radians(float(cam["fovy_deg"])) / 2.0)
    aperture_h = 20.955
    focal = fy * aperture_h / w
    torso = find_prim_path(stage, ROBOT_PATH, cam["parent_body"])
    roll, pitch, yaw = cam["rpy"]
    if abs(roll) > 1e-9 or abs(yaw) > 1e-9:
        raise ValueError("頭カメラの向きは pitch だけを想定している")
    head = Camera(CameraCfg(
        prim_path=f"{torso}/head_camera",
        offset=CameraCfg.OffsetCfg(pos=tuple(cam["xyz"]), rot=(0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2)),
                                   convention="world"),
        width=w, height=h, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=focal, horizontal_aperture=aperture_h, clipping_range=(0.02, 20.0)),
    ))
    target = pelvis_world + np.array([scene.boxes[0].center[0], scene.buttons[0].face_center[1], 0.1])
    eye = pelvis_world + np.array([-1.3, -1.1, 0.9])
    overview = Camera(CameraCfg(
        prim_path="/World/hall_overview",
        offset=CameraCfg.OffsetCfg(pos=tuple(eye.tolist()), rot=look_at_xyzw(eye, target), convention="world"),
        width=960, height=720, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955, clipping_range=(0.05, 50.0)),
    ))
    return {cam["name"]: head, "hall_overview": overview}


def step(sim, robot: Articulation, hall_art: Articulation, hall: HallIsaac, n: int, cameras=None) -> list[str]:
    pressed: list[str] = []
    for i in range(n):
        robot.write_data_to_sim()
        hall_art.write_data_to_sim()
        render = (i + 1) % RENDER_EVERY == 0
        sim.step(render=render)
        robot.update(PHYSICS_DT)
        hall_art.update(PHYSICS_DT)
        pressed += hall.update()
        if render and cameras:
            for c in cameras.values():
                c.update(PHYSICS_DT * RENDER_EVERY)
    return pressed


def save_png(cameras: dict[str, Camera], prefix: Path) -> None:
    from PIL import Image

    prefix.parent.mkdir(parents=True, exist_ok=True)
    for name, c in cameras.items():
        rgb = wp.to_torch(c.data.output["rgb"])[0] if not isinstance(c.data.output["rgb"], torch.Tensor) \
            else c.data.output["rgb"][0]
        p = prefix.with_name(f"{prefix.name}_{name}.png")
        Image.fromarray(rgb[..., :3].cpu().numpy().astype(np.uint8)).save(p)
        print(f"[png] {p}")


def main() -> None:
    scene_cfg = load_config(args.scene)
    scene = build_hall_scene(scene_cfg)
    robot_cfg = j1gen_config("robot.yaml")
    pelvis_world = np.array([0.0, 0.0, scene.pelvis_height])

    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, render_interval=RENDER_EVERY, device=args.device))
    stage = sim_utils.get_current_stage()

    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 0.95)).func(
        "/World/DomeLight", sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 0.95)))
    sim_utils.DistantLightCfg(intensity=1500.0, angle=1.0).func(
        "/World/DistantLight", sim_utils.DistantLightCfg(intensity=1500.0, angle=1.0))

    robot = make_robot(robot_cfg, scene_cfg, pelvis_world)
    print(f"[OK] G1 を読み込んだ（胴体固定、pelvis = {pelvis_world.tolist()}）")
    fingertip = scene_cfg.get("fingertip", {})
    if fingertip.get("enabled"):
        add_fingertips(stage, robot_cfg, float(fingertip["radius"]))
    shaders = build_hall(stage, scene, pelvis_world)
    hall_art = Articulation(hall_articulation_cfg(scene))
    print(f"[OK] 乗り場を作った（ボタン: {[b.name for b in scene.buttons]}）")
    cameras = make_cameras(stage, robot_cfg, scene, pelvis_world) if args.enable_cameras else {}

    sim.reset()
    sim.set_camera_view(eye=(pelvis_world + [-1.3, -1.1, 0.9]).tolist(),
                        target=(pelvis_world + [0.42, -0.10, 0.1]).tolist())
    hold_initial_pose(robot)
    hall = HallIsaac(hall_art, scene, shaders)
    print(f"[hall] 関節: {hall_art.joint_names}")

    # 置いた直後の揺れが収まるまで進める
    step(sim, robot, hall_art, hall, 200, cameras)
    print(f"[hall] 沈み量 [mm]: { {n: round(d * 1000, 2) for n, d in hall.depths().items()} }")

    if args.press:
        b = scene.button(args.press)
        hall.set_targets({b.name: b.travel})
        pressed = step(sim, robot, hall_art, hall, 150, cameras)
        print(f"[press] 押す: 沈み量 {hall.depths()[b.name] * 1000:.2f} mm、点灯 {hall.lit(b.name)}、押された {pressed}")
        hall.set_targets({b.name: 0.0})
        step(sim, robot, hall_art, hall, 150, cameras)
        print(f"[press] 離す: 沈み量 {hall.depths()[b.name] * 1000:.2f} mm、点灯 {hall.lit(b.name)}")
        others = [n for n in hall.states if n != b.name and hall.lit(n)]
        print(f"[press] ほかに点灯しているボタン: {others}")

    if args.png:
        if not cameras:
            print("[NG] --png には --enable_cameras が要る（run.sh は自動で付ける）")
        else:
            step(sim, robot, hall_art, hall, RENDER_EVERY * 5, cameras)
            prefix = Path(args.png)
            save_png(cameras, prefix if prefix.is_absolute() else REPO_ROOT / prefix)

    if not args.headless:
        print("[view] 画面で見る（Shift + 左ドラッグでボタンを押せる。閉じると終了）")
        n = 0
        while simulation_app.is_running() and (args.max_steps == 0 or n < args.max_steps):
            for name in step(sim, robot, hall_art, hall, RENDER_EVERY, cameras):
                print(f"[view] {name} が押された")
            n += RENDER_EVERY
    else:
        print("[INFO] ヘッドレスで起動したので終了する（画面で見るときは --headless を付けずに run.sh で起動する）")


if __name__ == "__main__":
    main()
    # IsaacLab の tutorials と同じく main() の外で閉じる（finally で閉じると例外が隠れる。CLAUDE.md）
    simulation_app.close()
