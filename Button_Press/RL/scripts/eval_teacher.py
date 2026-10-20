"""学習した教師を、学習環境の中でたくさんの試行で動かし、成功率と押す力の分布を測る。eval_teacher.sh から起動する。

    bash Button_Press/RL/scripts/eval_teacher.sh --episodes 2000                   # 最新の checkpoint
    bash Button_Press/RL/scripts/eval_teacher.sh --load_run <フォルダ名> --checkpoint model_900.pt

学習のログの Metrics/ は、各回で終わった試行の平均なので件数が少なくばらつく。ここでは決まった数の試行を集めて
分布（押す力が 3 N 未満・3〜8 N・8 N 超の割合など）を出す。結果は JSON にも保存する。
"""

from __future__ import annotations

import argparse
import importlib.metadata as metadata
import json
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

parser = argparse.ArgumentParser(description="教師の評価")
parser.add_argument("--task", default="G1-ButtonPress-Teacher-v0")
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--episodes", type=int, default=2000, help="集める試行の数")
parser.add_argument("--load_run", default=".*", help="logs/rsl_rl/<experiment>/ の中のフォルダ（正規表現。既定は最新）")
parser.add_argument("--checkpoint", default="model_.*.pt", help="checkpoint のファイル名（既定は最新）")
parser.add_argument("--out", default="", help="結果の JSON（既定は checkpoint の横の eval_<日時>.json）")
parser.add_argument("--export", default="", help="方策を TorchScript で書き出す先（評価用の Agent が読む。例: Button_Press/RL/agent/policy.pt）")
parser.add_argument("--no-eval", action="store_true", help="書き出すだけ（試行は集めない）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import button_press_rl.tasks  # noqa: E402, F401
from button_press_rl.spec import load_task_spec  # noqa: E402


def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    env_cfg.record_episodes = True
    agent_cfg = load_cfg_from_registry(args.task, "rsl_rl_cfg_entry_point")
    # train.py / play.py と同じ（設定の形を入っている rsl_rl の版に合わせる。無いと MLPModel の引数で落ちる）
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
    # 絶対パスにする（Isaac Lab の get_checkpoint_path は相対パスだとフォルダを二重につなぐ）
    log_root = (Path("logs/rsl_rl") / agent_cfg.experiment_name).resolve()
    ckpt = get_checkpoint_path(str(log_root), args.load_run, args.checkpoint)
    print(f"[eval] checkpoint: {ckpt}")

    env = RslRlVecEnvWrapper(gym.make(args.task, cfg=env_cfg), clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(ckpt)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    if args.export:
        out = Path(args.export).resolve()
        runner.export_policy_to_jit(path=str(out.parent), filename=out.name)
        print(f"[eval] 方策を書き出した: {out}")
        if args.no_eval:
            env.close()
            return
    raw = env.unwrapped
    # 作った直後の試行（途中から始まるもの）は数えない: 1 回全部をリセットしてから集める
    raw.episode_records.clear()

    obs = env.get_observations()
    t0 = time.time()
    steps = 0
    while len(raw.episode_records) < args.episodes:
        with torch.inference_mode():
            obs, _, dones, _ = env.step(policy(obs))
            policy.reset(dones)
        steps += 1
        if steps % 100 == 0:
            print(f"[eval] {len(raw.episode_records)} / {args.episodes} 試行（{time.time() - t0:.0f} s）")
    recs = raw.episode_records[: args.episodes]
    env.close()

    spec = load_task_spec(env_cfg.target)
    lo, hi = spec.press_ok
    ok = [r for r in recs if r["success"] > 0]
    f = np.array([r["max_force"] for r in ok])
    summary = {
        "checkpoint": str(ckpt), "episodes": len(recs),
        "success_rate": len(ok) / len(recs),
        "tip_press_rate": float(np.mean([r["tip_press"] for r in ok])) if ok else None,
        "wrong_rate": float(np.mean([r["wrong"] for r in recs])),
        "released_rate": float(np.mean([r["released"] for r in ok])) if ok else None,
        "force_abort_rate": float(np.mean([r["force_abort"] for r in recs])),
        "in_band_rate": float(np.mean([r["in_band"] for r in ok])) if ok else None,
        "max_force_n": {"mean": float(f.mean()), "median": float(np.median(f)),
                        "p10": float(np.percentile(f, 10)), "p90": float(np.percentile(f, 90)),
                        "max": float(f.max())} if ok else None,
        "max_force_split": {f"below_{lo:g}N": float((f < lo).mean()), f"{lo:g}-{hi:g}N": float(((f >= lo) & (f <= hi)).mean()),
                            f"above_{hi:g}N": float((f > hi).mean()),
                            f"above_{spec.press_max:g}N": float((f > spec.press_max).mean())} if ok else None,
        "min_dist_m_mean": float(np.mean([r["min_dist"] for r in recs])),
    }
    print("[eval] " + json.dumps(summary, ensure_ascii=False, indent=2))
    out = Path(args.out) if args.out else Path(ckpt).with_name(f"eval_{time.strftime('%Y%m%d_%H%M%S')}.json")
    out.write_text(json.dumps({"summary": summary, "episodes": recs}, ensure_ascii=False, indent=1))
    print(f"[eval] 保存した: {out}")


if __name__ == "__main__":
    main()
    simulation_app.close()
