"""ボトルを押す方策を PPO（stable-baselines3）で学習する。環境は rl/press_env.py、設定は configs/rl_press.yaml。

    P=_local/rl/venv/bin/python
    $P Button_Press/J1-gen/rl/train.py --steps 2000000 --envs 8
    $P Button_Press/J1-gen/rl/train.py --steps 2000000 --envs 8 --resume _local/rl/runs/<前の記録>/model_latest.zip

用語:
- PPO: よく使われる強化学習の方法。方策（行動を出すネットワーク）と価値関数（この先もらえる報酬の見積もり）を一緒に学習する。
  集めた経験で方策を少しずつ（変えすぎないように）更新する
- 並列の環境（--envs）: 環境を別々のプロセスで同時に動かして、経験を速く集める。CPU のコア数くらいまで増やせる
- 観測の正規化（VecNormalize）: 観測の各値を平均 0・ばらつき 1 くらいにそろえる（関節角 [rad] と位置 [m] の大きさの違いをならす）。
  平均とばらつきは学習中に更新されるので、方策と一緒に保存する（vecnormalize.pkl）。評価や実機ではこれを読んで同じ変換をする

記録（--out、既定は _local/rl/runs/<日時>）:
  model_latest.zip / vecnormalize_latest.pkl   最新の方策と正規化の値（--save-every ごとに上書き）
  model_<周期>.zip / vecnormalize_<周期>.pkl   途中の方策（--keep-every ごと）
  progress.csv                                 学習の経過（成功・当たった・時間切れの割合、平均の報酬など）
  config.yaml                                  使った設定（configs/rl_press.yaml と --set の内容）
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config  # noqa: E402

OUTCOMES = ("success", "collision", "overforce", "timeout")


def make_env_fn(cfg: dict[str, Any], rank: int, seed: int):
    """並列の環境を作る関数（別のプロセスの中で呼ばれる）。"""

    def _init():
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from stable_baselines3.common.monitor import Monitor

        from rl.press_env import PressEnv

        env = Monitor(PressEnv(cfg), info_keywords=("outcome",))
        env.reset(seed=seed + rank)
        return env

    return _init


def parse_set(items: list[str]) -> dict[str, Any]:
    """--set reward.progress=20 のような上書きを、入れ子の辞書にする（値は YAML として読む）。"""
    out: dict[str, Any] = {}
    for it in items:
        key, _, val = it.partition("=")
        if not val:
            raise ValueError(f"--set は key=value の形: {it}")
        node = out
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = yaml.safe_load(val)
    return out


def make_callback(out: Path, save_every: int, keep_every: int):
    from stable_baselines3.common.callbacks import BaseCallback

    class Progress(BaseCallback):
        """エピソードの終わり方を数えて記録し、方策を定期的に保存する。"""

        def __init__(self) -> None:
            super().__init__()
            self.counts = dict.fromkeys(OUTCOMES, 0)
            self.ep_steps: list[int] = []
            self.t0 = time.monotonic()
            self.last_save = 0
            self.last_keep = 0

        def _on_step(self) -> bool:
            for info in self.locals["infos"]:
                ep = info.get("episode")
                if ep is None:
                    continue
                self.counts[info.get("outcome") or "timeout"] += 1
                self.ep_steps.append(int(ep["l"]))
            if self.num_timesteps - self.last_save >= save_every:
                self.save("latest")
                self.last_save = self.num_timesteps
            if keep_every and self.num_timesteps - self.last_keep >= keep_every:
                self.save(str(self.num_timesteps))
                self.last_keep = self.num_timesteps
            return True

        def _on_rollout_end(self) -> None:
            n = sum(self.counts.values())
            if n:
                for k in OUTCOMES:
                    self.logger.record(f"outcome/{k}", self.counts[k] / n)
                self.logger.record("outcome/episodes", n)
                self.logger.record("outcome/mean_steps", float(np.mean(self.ep_steps)))
            self.logger.record("time/steps_per_s", self.num_timesteps / max(1e-6, time.monotonic() - self.t0))
            self.counts = dict.fromkeys(OUTCOMES, 0)
            self.ep_steps = []

        def save(self, tag: str) -> None:
            self.model.save(out / f"model_{tag}.zip")
            env = self.model.get_env()
            if env is not None and hasattr(env, "save"):
                env.save(str(out / f"vecnormalize_{tag}.pkl"))

    return Progress()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=2_000_000, help="学習する周期の数（全部の環境の合計）")
    ap.add_argument("--envs", type=int, default=8, help="並列の環境の数（CPU のコア数くらいまで）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, help="記録の保存先（既定は _local/rl/runs/<日時>）")
    ap.add_argument("--resume", type=Path, help="続きから学習する model_*.zip（同じ場所の vecnormalize_*.pkl も読む）")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE",
                    help="configs/rl_press.yaml の値を上書きする（例: reward.progress=20 success.lateral_tol_m=0.01）")
    ap.add_argument("--n-steps", type=int, default=1024, help="PPO: 1 回の更新の前に、環境ごとに集める周期の数")
    ap.add_argument("--batch", type=int, default=4096, help="PPO: ミニバッチの大きさ")
    ap.add_argument("--lr", type=float, default=3e-4, help="PPO: 学習率")
    ap.add_argument("--gamma", type=float, default=0.99, help="PPO: 割引率（先の報酬をどれだけ重く見るか）")
    ap.add_argument("--torch-threads", type=int, default=1, help="学習の計算（torch）のスレッド数")
    ap.add_argument("--save-every", type=int, default=200_000)
    ap.add_argument("--keep-every", type=int, default=1_000_000)
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

    from common.config import deep_merge

    cfg = deep_merge(load_config("rl_press.yaml"), parse_set(args.set))
    out = args.out or REPO_ROOT / "_local" / "rl" / "runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.yaml").write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    # 方策は小さいので、学習の計算は 1 スレッドで足りる。ノート PC（aarch64）では、1 周期ごとの小さな計算（10 並列分の
    # 観測から行動を出す）が 2 スレッド以上だと 1 スレッドの 3 倍以上遅かった（2026-11-01 実測）
    torch.set_num_threads(args.torch_threads)

    venv = SubprocVecEnv([make_env_fn(cfg, i, args.seed) for i in range(args.envs)])
    if args.resume:
        vn = args.resume.with_name(args.resume.name.replace("model_", "vecnormalize_")).with_suffix(".pkl")
        env = VecNormalize.load(str(vn), venv)
        env.training = True
        model = PPO.load(str(args.resume), env=env, device="cpu")
        print(f"[train] 続きから: {args.resume}（正規化 {vn}）")
    else:
        env = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=args.gamma)
        model = PPO("MlpPolicy", env, device="cpu", seed=args.seed, n_steps=args.n_steps, batch_size=args.batch,
                    learning_rate=args.lr, gamma=args.gamma, gae_lambda=0.95, n_epochs=10, clip_range=0.2,
                    ent_coef=0.0, policy_kwargs={"net_arch": [256, 256], "log_std_init": -0.5})
    model.set_logger(configure(str(out), ["stdout", "csv"]))
    print(f"[train] 記録: {out}")
    print(f"[train] 並列 {args.envs}、{args.steps:,} 周期を学習する")
    cb = make_callback(out, args.save_every, args.keep_every)
    t0 = time.monotonic()
    try:
        model.learn(total_timesteps=args.steps, callback=cb, reset_num_timesteps=not args.resume)
    except KeyboardInterrupt:
        print("[train] Ctrl+C: ここまでの方策を保存して終わる")
    cb.save("latest")
    env.close()
    print(f"[train] 終了: {(time.monotonic() - t0) / 60:.1f} 分。方策 {out / 'model_latest.zip'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
