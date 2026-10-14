"""DoorMan 同梱の HOMIE（下半身の歩行の方策）を、Unitree 公式の MuJoCo のモデルで動かす（段 3 の「脚だけ sim-to-sim」）。

DoorMan（Isaac Sim 5.1）での入出力を、コード（homie_base.py・homie_modules.py・ppo_trainer_homie_api.py）から読んで再現した:
  - 1 ステップの観測 86 = コマンド 7・胴の角速度 3（×0.5）・重力の向き 3・関節角 29（既定の姿勢との差）・関節速度 29（×0.05）・前回の脚の行動 15
  - 6 ステップ分（古い → 新しい、今のステップが末尾）＝ 516 を入れる。行動 15（脚 12・腰 3）
  - コマンド = [vx, vy, wz, 高さ 0.74, roll, pitch, yaw]。|(vx, vy, wz)| > 0.5 なら歩く方策、それ以下は立つ方策（立つ方策には速度 0 を渡す）
  - 目標角 = 行動 × 0.25 + 既定の姿勢。PD のゲインは g1_43dof.yaml（腕は既定の姿勢に保つ）。方策 50 Hz・物理 200 Hz
  - 立たせてから渡すと倒れるので、置いた直後から方策を動かす（unitree_rl_lab で確かめた落とし穴）

使い方（Mac、G1_Hackason/Navigation/.venv の Python 3.10・torch・mujoco）:
    python homie_mujoco.py --mjcf <unitree_mujoco>/unitree_robots/g1/scene_29dof.xml \
        --homie-modules <VIRAL>/gr00t/rl/trl/modules/homie_modules.py \
        --walk <VIRAL>/models/model_walk.pt --stand <VIRAL>/models/model_stand.pt \
        --cmd 0.8,0,0 --seconds 10 [--video out.mp4]
"""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import torch

NUM_BODY_DOF = 29
NUM_LOWER = 15
ONE_STEP = 86
HISTORY = 6
ACTION_SCALE = 0.25
SIM_DT = 0.005
DECIMATION = 4
SWITCH_THRESHOLD = 0.5
DEFAULT_HEIGHT_CMD = 0.74
SPAWN_HEIGHT = 0.78
FALL_HEIGHT = 0.5

# g1_43dof.yaml の default_joint_angles（先頭 29 関節）
DEFAULT_POS = np.array(
    [-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0, 0, 0, 0] + [0.0] * 14, dtype=np.float64
)
# g1_43dof.yaml の control.stiffness / damping（関節名の部分一致）
KP = {"hip_yaw": 150, "hip_roll": 150, "hip_pitch": 150, "knee": 200, "ankle_pitch": 40, "ankle_roll": 40,
      "waist_yaw": 250, "waist_roll": 250, "waist_pitch": 250, "shoulder_pitch": 100, "shoulder_roll": 100,
      "shoulder_yaw": 40, "elbow": 40, "wrist_roll": 20, "wrist_pitch": 20, "wrist_yaw": 20}
KD = {"hip_yaw": 2.0, "hip_roll": 2.0, "hip_pitch": 2.0, "knee": 4.0, "ankle_pitch": 2.0, "ankle_roll": 2.0,
      "waist_yaw": 5.0, "waist_roll": 5.0, "waist_pitch": 5.0, "shoulder_pitch": 5.0, "shoulder_roll": 5.0,
      "shoulder_yaw": 2.0, "elbow": 2.0, "wrist_roll": 2.0, "wrist_pitch": 2.0, "wrist_yaw": 2.0}
# g1_43dof.yaml の dof_effort_limit_list（先頭 29）× dof_effort_limit_scale 0.95
EFFORT = 0.95 * np.array(
    [88, 139, 88, 139, 35, 35, 88, 139, 88, 139, 35, 35, 88, 35, 35,
     25, 25, 25, 25, 25, 5, 5, 25, 25, 25, 25, 25, 5, 5], dtype=np.float64
)
JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]


def gains_for(names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    kp, kd = np.zeros(len(names)), np.zeros(len(names))
    for i, n in enumerate(names):
        key = next(k for k in KP if k in n)
        kp[i], kd[i] = KP[key], KD[key]
    return kp, kd


def load_homie(modules_path: Path, weights: Path) -> torch.nn.Module:
    spec = importlib.util.spec_from_file_location("homie_modules", modules_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    net = mod.HIMActorCritic(**mod.init_actor_critic_dict)
    net.load_state_dict(torch.load(weights, map_location="cpu", weights_only=False)["model_state_dict"])
    return mod.HomieActorModule(net).eval()


def quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q_wxyz
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ v


def yaw_of(q_wxyz: np.ndarray) -> float:
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class HomieRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(args.mjcf))
        self.model.opt.timestep = SIM_DT
        self.data = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint(f"{n}_joint").qposadr[0] for n in JOINT_NAMES])
        self.vadr = np.array([self.model.joint(f"{n}_joint").dofadr[0] for n in JOINT_NAMES])
        self.act = np.array([self.model.actuator(n).id for n in JOINT_NAMES])
        self.kp, self.kd = gains_for(JOINT_NAMES)
        self.walk = load_homie(args.homie_modules, args.walk)
        self.stand = load_homie(args.homie_modules, args.stand)
        self.history = np.zeros((HISTORY - 1, ONE_STEP), dtype=np.float32)
        self.last_action = np.zeros(NUM_LOWER, dtype=np.float32)
        self.target = DEFAULT_POS.copy()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = SPAWN_HEIGHT
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[self.qadr] = DEFAULT_POS
        mujoco.mj_forward(self.model, self.data)

    def one_step_obs(self, cmd: np.ndarray) -> np.ndarray:
        quat = self.data.qpos[3:7]
        ang_vel = self.data.qvel[3:6]  # free joint の角速度は胴の座標系
        gravity = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0]))
        q = self.data.qpos[self.qadr] - DEFAULT_POS
        dq = self.data.qvel[self.vadr]
        command = np.array([cmd[0], cmd[1], cmd[2], DEFAULT_HEIGHT_CMD, 0, 0, 0])
        if np.linalg.norm(command[:3]) < 0.1:
            command[:3] = 0.0
        return np.concatenate([command, ang_vel * 0.5, gravity, q, dq * 0.05, self.last_action]).astype(np.float32)

    @torch.no_grad()
    def policy_step(self, cmd: np.ndarray) -> bool:
        current = self.one_step_obs(cmd)
        obs = np.concatenate([self.history.reshape(-1), current])[None]
        use_walk = np.linalg.norm(current[:3]) > SWITCH_THRESHOLD
        if use_walk:
            action = self.walk(torch.from_numpy(obs))["actions"][0].numpy()
        else:
            stand_obs = obs.reshape(HISTORY, ONE_STEP).copy()
            stand_obs[:, :3] = 0.0
            action = self.stand(torch.from_numpy(stand_obs.reshape(1, -1)))["actions"][0].numpy()
        self.history = np.concatenate([self.history[1:], current[None]], axis=0)
        self.last_action = action.astype(np.float32)
        self.target = DEFAULT_POS.copy()
        self.target[:NUM_LOWER] += action * ACTION_SCALE
        return bool(use_walk)

    def physics_step(self) -> None:
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.vadr]
        tau = np.clip(self.kp * (self.target - q) - self.kd * dq, -EFFORT, EFFORT)
        self.data.ctrl[self.act] = tau
        mujoco.mj_step(self.model, self.data)


def run(args: argparse.Namespace) -> dict:
    cmd = np.array([float(v) for v in args.cmd.split(",")])
    runner = HomieRunner(args)
    runner.reset()
    renderer, frames = None, []
    if args.video:
        renderer = mujoco.Renderer(runner.model, height=360, width=640)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = runner.model.body("pelvis").id
        cam.distance, cam.elevation, cam.azimuth = 3.0, -15.0, 135.0
    start_xy = runner.data.qpos[:2].copy()
    prev_yaw = yaw_of(runner.data.qpos[3:7])
    total_yaw = 0.0  # 1 周を超えても数えられるよう、毎ステップの差を積み上げる
    steps = int(args.seconds / (SIM_DT * DECIMATION))
    min_height, walk_steps, fell_at = 10.0, 0, None
    for i in range(steps):
        walk_steps += runner.policy_step(cmd)
        for _ in range(DECIMATION):
            runner.physics_step()
        yaw = yaw_of(runner.data.qpos[3:7])
        total_yaw += (yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi
        prev_yaw = yaw
        h = float(runner.data.qpos[2])
        min_height = min(min_height, h)
        if fell_at is None and h < FALL_HEIGHT:
            fell_at = i * SIM_DT * DECIMATION
        if renderer is not None and i % 2 == 0:
            renderer.update_scene(runner.data, cam)
            frames.append(renderer.render())
    end_xy = runner.data.qpos[:2]
    result = {
        "cmd": cmd.tolist(),
        "seconds": args.seconds,
        "displacement_xy": (end_xy - start_xy).round(3).tolist(),
        "distance": round(float(np.linalg.norm(end_xy - start_xy)), 3),
        "mean_speed": round(float(np.linalg.norm(end_xy - start_xy)) / args.seconds, 3),
        "yaw_change_deg": round(math.degrees(total_yaw), 1),
        "mean_yaw_rate": round(total_yaw / args.seconds, 3),
        "final_height": round(float(runner.data.qpos[2]), 3),
        "min_height": round(min_height, 3),
        "fell_at_s": fell_at,
        "walk_policy_ratio": round(walk_steps / steps, 3),
    }
    if renderer is not None and frames:
        write_video(args.video, frames, fps=int(1 / (SIM_DT * DECIMATION * 2)))
        result["video"] = str(args.video)
    return result


def write_video(path: Path, frames: list[np.ndarray], fps: int) -> None:
    """ffmpeg（Mac は Homebrew の /opt/homebrew/bin/ffmpeg）に生の RGB を流して H.264 の mp4 にする。"""
    import shutil
    import subprocess

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg が見つからない（brew install ffmpeg）")
    h, w, _ = frames[0].shape
    cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
           "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(path)]
    subprocess.run(cmd, input=b"".join(f.astype(np.uint8).tobytes() for f in frames), check=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", type=Path, required=True)
    ap.add_argument("--homie-modules", type=Path, required=True)
    ap.add_argument("--walk", type=Path, required=True)
    ap.add_argument("--stand", type=Path, required=True)
    ap.add_argument("--cmd", default="0,0,0", help="vx,vy,wz（実際の速度 m/s・rad/s）")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--video", type=Path, default=None)
    args = ap.parse_args()
    print("[homie_mujoco] " + json.dumps(run(args), ensure_ascii=False))


if __name__ == "__main__":
    main()
