"""ボタン押しの学習環境（Isaac Lab のタスクとして登録する）。

    G1-ButtonPress-Teacher-v0  教師（ボタンの位置を観測に入れる。PPO）
"""

import gymnasium as gym

from . import agents

gym.register(
    id="G1-ButtonPress-Teacher-v0",
    entry_point=f"{__name__}.press_env:ButtonPressEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.press_env_cfg:ButtonPressEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ButtonPressPPORunnerCfg",
    },
)
