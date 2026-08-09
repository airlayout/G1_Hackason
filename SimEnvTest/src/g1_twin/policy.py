"""学習済み歩行ポリシー (Isaac-Velocity-Flat-G1-v0) を Isaac Sim 上で動かす。

NVIDIA が公開している rsl_rl の checkpoint をそのまま読み込み、
IsaacLab の学習時と同一の観測を組み立ててアクションを得る。

観測の仕様（IsaacLab の velocity_env_cfg.py PolicyCfg と一致させること）:
    [ 0: 3] base_lin_vel        胴体座標系での並進速度
    [ 3: 6] base_ang_vel        胴体座標系での角速度
    [ 6: 9] projected_gravity   胴体座標系での重力方向
    [ 9:12] velocity_commands   (vx, vy, yaw_rate)
    [12:49] joint_pos - default 関節位置の既定値からの差 (37)
    [49:86] joint_vel           関節速度 (37)
    [86:123] last_action        前回のアクション (37)
    合計 123 次元

重要:
- Flat タスクは height_scan を使わない（obs 123）。Rough は +187 で 310 になる。
- 観測にスケーリングは掛けない（IsaacLab の PolicyCfg にスケール指定が無い）。
  Unitree 公式リポジトリのポリシーは 0.25 / 0.05 等のスケールを掛けるので混同しないこと。
- アクションは scale 0.5 を掛けて既定関節位置に加算する（use_default_offset=True 相当）。
"""

from __future__ import annotations

import io
from pathlib import Path

import torch

# 観測・アクションの次元（checkpoint の実測値）
OBS_DIM: int = 123
ACTION_DIM: int = 37

# IsaacLab の ActionsCfg.joint_pos.scale と一致させる
ACTION_SCALE: float = 0.5

# 学習済み checkpoint の取得元（Isaac Sim 6.0 と同一バージョン）
CHECKPOINT_URL: str = (
    "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
    "/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Flat-G1-v0/checkpoint.pt"
)


def _build_actor(state_dict: dict[str, torch.Tensor]) -> torch.nn.Sequential:
    """checkpoint の state_dict から actor ネットワークを再構築する。

    rsl_rl の ActorCritic は actor が ELU 活性化の MLP になっている。
    層の形状は state_dict から読み取るため、隠れ層サイズをハードコードしない。

    Args:
        state_dict: checkpoint の "model_state_dict"

    Returns:
        actor の重みを読み込んだ Sequential モデル
    """
    # actor.0.weight, actor.2.weight, ... の順に層を集める
    indices = sorted(
        int(k.split(".")[1])
        for k in state_dict
        if k.startswith("actor.") and k.endswith(".weight")
    )

    layers: list[torch.nn.Module] = []
    for pos, idx in enumerate(indices):
        weight = state_dict[f"actor.{idx}.weight"]
        out_features, in_features = weight.shape
        linear = torch.nn.Linear(in_features, out_features)
        with torch.no_grad():
            linear.weight.copy_(weight)
            linear.bias.copy_(state_dict[f"actor.{idx}.bias"])
        layers.append(linear)
        # 最終層以外に活性化関数を挟む
        if pos < len(indices) - 1:
            layers.append(torch.nn.ELU())

    return torch.nn.Sequential(*layers)


class G1FlatPolicy:
    """G1 の平地歩行ポリシー。

    観測の組み立てとアクションの適用を担う。ロボット本体の操作は
    呼び出し側（ランナー）が Articulation 経由で行う。
    """

    def __init__(self, checkpoint_path: str | Path, device: str = "cuda:0") -> None:
        """checkpoint を読み込む。

        Args:
            checkpoint_path: `checkpoint.pt` のパス
            device: 推論に使うデバイス
        """
        self._device = torch.device(device)
        checkpoint = torch.load(checkpoint_path, map_location=self._device, weights_only=False)
        state_dict = checkpoint["model_state_dict"]

        self._actor = _build_actor(state_dict).to(self._device).eval()

        # 形状の検証（黙って間違った次元で動かさない）
        first = next(m for m in self._actor if isinstance(m, torch.nn.Linear))
        last = [m for m in self._actor if isinstance(m, torch.nn.Linear)][-1]
        if first.in_features != OBS_DIM:
            raise ValueError(
                f"[G1] 観測次元が一致しません: checkpoint={first.in_features}, 期待={OBS_DIM}"
            )
        if last.out_features != ACTION_DIM:
            raise ValueError(
                f"[G1] アクション次元が一致しません: checkpoint={last.out_features}, 期待={ACTION_DIM}"
            )

        self._last_action = torch.zeros(ACTION_DIM, device=self._device)
        iteration = checkpoint.get("iter", "unknown")
        print(f"[OK] 歩行ポリシーを読み込みました (obs={OBS_DIM}, action={ACTION_DIM}, iter={iteration})")

    @property
    def last_action(self) -> torch.Tensor:
        """前回のアクション（観測に含めるため公開する）。"""
        return self._last_action

    def reset(self) -> None:
        """内部状態を初期化する。エピソード開始時に呼ぶ。"""
        self._last_action = torch.zeros(ACTION_DIM, device=self._device)

    def build_observation(
        self,
        base_lin_vel_b: torch.Tensor,
        base_ang_vel_b: torch.Tensor,
        projected_gravity_b: torch.Tensor,
        command: tuple[float, float, float],
        joint_pos_rel: torch.Tensor,
        joint_vel: torch.Tensor,
    ) -> torch.Tensor:
        """観測ベクトルを組み立てる。

        引数はいずれも胴体座標系・既定値との相対値で渡すこと。

        Args:
            base_lin_vel_b: 胴体座標系の並進速度 (3,)
            base_ang_vel_b: 胴体座標系の角速度 (3,)
            projected_gravity_b: 胴体座標系の重力方向 (3,)
            command: (vx, vy, yaw_rate)
            joint_pos_rel: 既定関節位置からの差 (37,)
            joint_vel: 関節速度 (37,)

        Returns:
            123 次元の観測ベクトル
        """
        obs = torch.zeros(OBS_DIM, device=self._device)
        obs[0:3] = base_lin_vel_b
        obs[3:6] = base_ang_vel_b
        obs[6:9] = projected_gravity_b
        obs[9:12] = torch.tensor(command, device=self._device)
        obs[12:49] = joint_pos_rel
        obs[49:86] = joint_vel
        obs[86:123] = self._last_action
        return obs

    @torch.no_grad()
    def act(self, observation: torch.Tensor) -> torch.Tensor:
        """観測からアクションを推論する。

        Args:
            observation: 123 次元の観測ベクトル

        Returns:
            37 次元のアクション（スケール適用前）
        """
        action = self._actor(observation)
        self._last_action = action.clone()
        return action

    def joint_position_targets(
        self, action: torch.Tensor, default_joint_pos: torch.Tensor
    ) -> torch.Tensor:
        """アクションを関節位置目標へ変換する。

        IsaacLab の JointPositionActionCfg(scale=0.5, use_default_offset=True) と等価。

        Args:
            action: `act()` の出力 (37,)
            default_joint_pos: 既定の関節位置 (37,)

        Returns:
            関節位置目標 (37,)
        """
        return default_joint_pos + action * ACTION_SCALE
