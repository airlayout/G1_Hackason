"""教師が学習していく様子の動画を作る（学習の途中の方策を順に動かして録画し、1 本にまとめる）。record_progress.sh から起動する。

    bash Button_Press/RL/scripts/record_progress.sh                                  # 既定の時点で 1 分ほど
    bash Button_Press/RL/scripts/record_progress.sh --iters 0 600 1200 1499 2700 --clip_s 8

学習の途中の方策は logs/rsl_rl/button_press_teacher/<日時>/model_<回数>.pt（100 回ごとに保存）。回数から、どのフォルダに
あるかを探す（続きから学習したフォルダも含めて、新しいほうを使う）。どの時点も同じ乱数の種で始めるので、乗り場と立ち位置は同じ。
ボタンが点灯したら、評価環境と同じく白く光らせる（学習では光らせていない）。
"""

from __future__ import annotations

import argparse
import importlib.metadata as metadata
import re
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

parser = argparse.ArgumentParser(description="学習の様子の動画")
parser.add_argument("--task", default="G1-ButtonPress-Teacher-v0")
parser.add_argument("--iters", type=int, nargs="+", default=[0, 400, 800, 1200, 1499, 2000, 2700],
                    help="映す学習の回数（100 回ごとと、各フォルダの最後の回数がある）")
parser.add_argument("--clip_s", type=float, default=7.0, help="1 つの時点の長さ [s]")
parser.add_argument("--seed", type=int, default=3, help="乗り場と立ち位置を決める乱数の種（どの時点も同じ）")
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--fps", type=int, default=25)
parser.add_argument("--num_envs_trained", type=int, default=2048, help="学習の並列数（経験の時間の計算に使う）")
parser.add_argument("--steps_per_iter", type=int, default=32, help="1 回の学習で 1 環境が進める周期の数")
parser.add_argument("--out", default="", help="動画の保存先（既定は _local/button_press_rl/videos/progress_<日時>.mp4）")
parser.add_argument("--part", choices=["clips", "crowd"], default="clips",
                    help="clips: タイトルと各時点（まとめの画面なし）。crowd: たくさんの G1 を並べてカメラを引く場面とまとめの画面")
parser.add_argument("--crowd_envs", type=int, default=2048, help="crowd で並べる環境の数")
parser.add_argument("--crowd_s", type=float, default=9.0, help="crowd のカメラを引く長さ [s]")
parser.add_argument("--crowd_noise", type=float, default=0.5, help="crowd の行動に足す揺らぎ（学習中の探索に見せる）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import ViewerCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import button_press_rl.tasks  # noqa: E402, F401
from button_press_rl.paths import LOCAL_DIR  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_LIGHT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
# 斜め後ろ（右後ろの上）から、腕・柱・ボタンが入るように（ロボットの腰が原点、前が +x）
EYE = (-0.70, -1.10, 1.55)
LOOKAT = (0.25, -0.05, 0.95)


def _iters(run: Path) -> list[int]:
    return sorted(int(m.group(1)) for p in run.glob("model_*.pt") if (m := re.search(r"model_(\d+)", p.name)))


def find_checkpoint(log_root: Path, it: int) -> Path:
    """回数 it の checkpoint（新しいフォルダを優先）。動作の確認の短い学習（回数の幅が 100 未満）のフォルダは使わない。"""
    runs = [d for d in log_root.iterdir() if d.is_dir() and len(i := _iters(d)) > 1 and i[-1] - i[0] >= 100]
    for run in sorted(runs, key=lambda d: d.name, reverse=True):
        p = run / f"model_{it}.pt"
        if p.exists():
            return p
    have = sorted({int(m.group(1)) for p in log_root.glob("*/model_*.pt") if (m := re.search(r"model_(\d+)", p.name))})
    raise FileNotFoundError(f"学習 {it} 回の checkpoint が無い（あるのは {have}）")


def font(size: int, light: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_LIGHT if light else FONT, size)


def overlay(frame: np.ndarray, it: int, hours: float) -> np.ndarray:
    """左上に、学習の回数とロボットの経験の時間を重ねる。"""
    img = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    d.rounded_rectangle((28, 24, 470, 150), radius=14, fill=(0, 0, 0, 150))
    d.text((52, 34), f"学習 {it:,} 回", font=font(52), fill=(255, 255, 255, 255))
    d.text((54, 104), f"ロボットの経験 約 {hours:,.0f} 時間分", font=font(28, light=True), fill=(220, 220, 220, 255))
    return np.asarray(Image.alpha_composite(img, layer).convert("RGB"))


def card(size: tuple[int, int], lines: list[tuple[str, int, bool]]) -> np.ndarray:
    """黒地に文字を中央に並べた画面。lines = [(文字, 大きさ, 細字か), ...]"""
    w, h = size
    img = Image.new("RGB", (w, h), (12, 12, 14))
    d = ImageDraw.Draw(img)
    total = sum(s * 1.5 for _, s, _ in lines)
    y = (h - total) / 2
    for text, s, light in lines:
        f = font(s, light)
        tw = d.textlength(text, font=f)
        d.text(((w - tw) / 2, y), text, font=f, fill=(235, 235, 235) if not light else (180, 180, 185))
        y += s * 1.5
    return np.asarray(img)


def set_lit(stage, name: str, lit: bool, scene) -> None:
    """env 0 のボタンの面の色を、点灯なら白く光らせる（評価環境の isaac_hall と同じ色）。"""
    from pxr import Gf

    b = scene.button(name)
    sh = stage.GetPrimAtPath(f"/World/envs/env_0/Scene/Hall/Looks/mat_button_{name}_face/Shader")
    if not sh.IsValid():
        return
    rgb = b.lit_rgba[:3] if lit else b.off_rgba[:3]
    sh.GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(*rgb))
    em = np.asarray(b.lit_rgba[:3]) * 0.6 if lit else (0.0, 0.0, 0.0)
    sh.GetAttribute("inputs:emissiveColor").Set(Gf.Vec3f(*[float(v) for v in em]))


def ease(s: float) -> float:
    """0〜1 を滑らかに（始めと終わりをゆっくり）。"""
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def crowd() -> None:
    """たくさんの G1 を並べて動かし、1 体のアップから全体が見えるところまでカメラを引く。最後にまとめの画面。"""
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.crowd_envs)
    env_cfg.video_camera_size = (args.width, args.height)
    env_cfg.seed = args.seed
    agent_cfg = load_cfg_from_registry(args.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
    log_root = (Path("logs/rsl_rl") / agent_cfg.experiment_name).resolve()
    ckpt = find_checkpoint(log_root, args.iters[-1])
    print(f"[video] crowd: {args.crowd_envs} 環境、方策 {ckpt}")
    env = RslRlVecEnvWrapper(gym.make(args.task, cfg=env_cfg), clip_actions=agent_cfg.clip_actions)
    raw = env.unwrapped
    cam = raw.video_camera
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(ckpt))
    policy = runner.get_inference_policy(device=raw.device)

    # 並んだ環境の中心と広さ（env 0 から見た位置）
    org = raw.scene.env_origins.detach().cpu().numpy()
    rel = org - org[0]
    center = rel.mean(0)
    extent = float(np.ptp(rel[:, :2], axis=0).max())
    eye0, look0 = np.array(EYE), np.array(LOOKAT)
    # カメラの通り道（見る点のまわりの球面の座標）。環境は 4 m おきに並び、どれにも高さ 2.9 m の壁があるので、
    # 引き始めにまず見下ろす角度を上げて壁より高くなってから下がる（低いまま下がると、後ろの環境の壁を通り抜けた）。
    # 全体を入れると G1 が点になるので、G1 だと分かる距離（並びの広さの 3 割）で止める
    look1 = 0.35 * center + np.array([0.0, 0.0, 0.9])
    off0 = eye0 - look0
    dist0 = float(np.linalg.norm(off0))
    dist1 = 0.30 * extent
    az0 = float(np.arctan2(off0[1], off0[0]))
    az1 = float(np.arctan2(-0.40, -0.80))
    el0 = float(np.arcsin(off0[2] / dist0))
    el_mid, el1 = np.radians(65.0), np.radians(45.0)

    def camera_at(s: float) -> tuple[np.ndarray, np.ndarray]:
        """s（0〜1）での (eye, look)。env 0 の原点から見た位置。

        見る点は、引き始めてしばらくは env 0 のロボットに置く（早く動かすと、ロボットが画面から外れた）。
        見下ろす角度は、最初の 2 割で 65° まで上げてから、終わりの 45° へ下げる
        """
        u = ease((s - 0.35) / 0.65)
        look = (1 - u) * look0 + u * look1
        el = el0 + (el_mid - el0) * ease(s / 0.2) if s < 0.2 else el_mid + (el1 - el_mid) * ease((s - 0.2) / 0.8)
        az = az0 + (az1 - az0) * ease(s)
        dist = float(np.exp((1 - ease(s)) * np.log(dist0) + ease(s) * np.log(dist1)))  # 遠ざかる速さを一定に見せる
        d = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        return look + d * dist, look

    print(f"[video] crowd: 並びの広さ {extent:.0f} m")
    root0 = raw._robot.data.root_link_pos_w
    root0 = (root0.torch if hasattr(root0, "torch") else root0)[0].detach().cpu().numpy()
    print(f"[video] crowd: env 0 の原点 {org[0].round(2).tolist()}、env 0 のロボットの腰 {root0.round(2).tolist()}、"
          f"原点の最小 {org.min(0).round(1).tolist()} 最大 {org.max(0).round(1).tolist()}")

    out = Path(args.out)
    writer = imageio.get_writer(str(out), fps=args.fps, codec="libx264", quality=8, macro_block_size=8)
    size = (args.width, args.height)
    steps_per_frame = max(1, round(1.0 / (args.fps * raw.step_dt)))
    torch.manual_seed(args.seed)
    obs, _ = env.reset()
    n = int(args.crowd_s * args.fps)
    hold_n = int(1.5 * args.fps)
    label = f"{args.num_envs_trained:,} 環境を並列に学習"
    for k in range(n + hold_n):
        eye, look = camera_at(min(k / max(n - 1, 1), 1.0))
        eye_w, look_w = org[0] + eye, org[0] + look
        cam.set_world_poses_from_view(eyes=torch.tensor([eye_w], dtype=torch.float32, device=raw.device),
                                      targets=torch.tensor([look_w], dtype=torch.float32, device=raw.device))
        for _ in range(steps_per_frame):
            with torch.no_grad():
                a = policy(obs)
                a = a + args.crowd_noise * torch.randn_like(a)
                obs, _, _, _ = env.step(a)
        cam.update(raw.step_dt, force_recompute=True)
        rgb = cam.data.output["rgb"]
        rgb = rgb.torch if hasattr(rgb, "torch") else rgb
        frame = rgb[0][..., :3].cpu().numpy().astype(np.uint8)
        alpha = ease((k / max(n - 1, 1) - 0.35) / 0.3)  # 引き始めてから文字を出す
        if alpha > 0:
            img = Image.fromarray(frame).convert("RGBA")
            layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
            dr = ImageDraw.Draw(layer)
            f = font(56)
            tw = dr.textlength(label, font=f)
            x, y = (size[0] - tw) / 2, size[1] - 130
            dr.rounded_rectangle((x - 30, y - 14, x + tw + 30, y + 80), radius=16, fill=(0, 0, 0, int(160 * alpha)))
            dr.text((x, y), label, font=f, fill=(255, 255, 255, int(255 * alpha)))
            frame = np.asarray(Image.alpha_composite(img, layer).convert("RGB"))
        writer.append_data(frame)
        if k % 25 == 0:
            print(f"[video] crowd: {k + 1}/{n + hold_n} コマ")
    hours_per_iter = args.num_envs_trained * args.steps_per_iter * raw.step_dt / 3600.0
    end = card(size, [(f"{args.num_envs_trained:,} 環境を並列に学習", 44, False),
                      (f"学習 {args.iters[-1]:,} 回 ＝ ロボットの経験 約 {args.iters[-1] * hours_per_iter:,.0f} 時間分", 32, True),
                      ("中指で、適正な力で押し、点灯したら離す", 32, True)])
    for _ in range(int(4.0 * args.fps)):
        writer.append_data(end)
    writer.close()
    env.close()
    print(f"[video] 保存した: {out}")


def main() -> None:
    if args.part == "crowd":
        crowd()
        return
    from button_press_rl.paths import use_yada

    use_yada()
    from common.scene_spec import build_hall_scene

    import isaaclab.sim as sim_utils

    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env_cfg.viewer = ViewerCfg(eye=EYE, lookat=LOOKAT, resolution=(args.width, args.height), origin_type="env",
                               env_index=0)
    env_cfg.seed = args.seed
    agent_cfg = load_cfg_from_registry(args.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
    log_root = (Path("logs/rsl_rl") / agent_cfg.experiment_name).resolve()
    ckpts = [(it, find_checkpoint(log_root, it)) for it in args.iters]
    for it, p in ckpts:
        print(f"[video] 学習 {it} 回: {p}")

    env = RslRlVecEnvWrapper(gym.make(args.task, cfg=env_cfg, render_mode="rgb_array"),
                             clip_actions=agent_cfg.clip_actions)
    raw = env.unwrapped
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    stage = sim_utils.get_current_stage()
    scene = build_hall_scene(raw.task.scene_cfg)
    tgt_name = raw.cfg.target

    out = Path(args.out) if args.out else LOCAL_DIR / "videos" / f"progress_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(str(out), fps=args.fps, codec="libx264", quality=8, macro_block_size=8)
    size = (args.width, args.height)
    steps_per_frame = max(1, round(1.0 / (args.fps * raw.step_dt)))  # 実時間で再生する

    def hold(frame: np.ndarray, seconds: float) -> None:
        for _ in range(int(seconds * args.fps)):
            writer.append_data(frame)

    hold(card(size, [("G1 エレベーターのボタン押し", 60, False), ("Isaac Lab の強化学習で、押し方を一から覚える", 32, True)]), 3.0)

    hours_per_iter = args.num_envs_trained * args.steps_per_iter * raw.step_dt / 3600.0
    for it, path in ckpts:
        runner.load(str(path))
        policy = runner.get_inference_policy(device=raw.device)
        # どの時点も同じ乗り場・立ち位置にする
        torch.manual_seed(args.seed)
        np.random.seed(args.seed)
        obs, _ = env.reset()
        set_lit(stage, tgt_name, False, scene)
        for other in raw._button_names:
            set_lit(stage, other, False, scene)
        n_frames = int(args.clip_s * args.fps)
        frame = None
        ended = False
        for k in range(n_frames):
            if not ended:
                for _ in range(steps_per_frame):
                    # inference_mode だと、次の時点の reset で環境のテンソルを書き換えられない
                    with torch.no_grad():
                        obs, _, dones, _ = env.step(policy(obs))
                    lit = raw._lit[0].tolist()
                    for name, on in zip(raw._button_names, lit):
                        set_lit(stage, name, bool(on), scene)
                    if bool(dones[0]):
                        ended = True  # 試行が終わった（点灯して指を離した）。最後の絵のまま止める
                        break
                if not ended or frame is None:
                    frame = overlay(raw.render(), it, it * hours_per_iter)
            writer.append_data(frame)
        print(f"[video] 学習 {it} 回: 点灯 {bool(raw._ep['success'][0] > 0) or ended}")

    writer.close()
    env.close()
    print(f"[video] 保存した: {out}")


if __name__ == "__main__":
    main()
    simulation_app.close()
