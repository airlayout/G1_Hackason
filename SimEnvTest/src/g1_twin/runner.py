"""G1 デジタルツインのシーン構築とシミュレーションループ。

Warehouse シーンに G1 を配置し、キーボードからの速度コマンドで歩かせる。
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
import warp as wp

from .command import SimCommandSink, VelocityCommand
from .policy import G1FlatPolicy

# 物理・制御の刻み（IsaacLab の学習時と一致させる）
PHYSICS_DT: float = 0.005  # 200 Hz
CONTROL_DT: float = 0.02  # 50 Hz
DECIMATION: int = int(round(CONTROL_DT / PHYSICS_DT))  # 4

# G1 の配置（G1_CFG.init_state.pos と一致）
SPAWN_HEIGHT: float = 0.74

# Warehouse シーンの相対パス
WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"


@dataclass
class RunnerConfig:
    """ランナーの設定。"""

    # Warehouse を使うか（False なら平地のみ）
    use_warehouse: bool = True
    # G1 のスポーン位置 (x, y)
    spawn_xy: tuple[float, float] = (0.0, 0.0)
    device: str = "cuda:0"


class G1TwinRunner:
    """G1 デジタルツインの実行ループ。

    Isaac Sim のアプリ起動後に生成すること（omni.* の import が必要なため）。
    """

    def __init__(self, checkpoint_path: str, config: RunnerConfig | None = None) -> None:
        self._config = config or RunnerConfig()
        self._device = torch.device(self._config.device)
        self._policy = G1FlatPolicy(checkpoint_path, device=self._config.device)
        self._sink = SimCommandSink()
        self._commander = None  # キーボードは reset 後に生成する
        self._robot = None
        self._step_count = 0
        # 直近に表示した指令（変化時のみログを出すため）
        self._last_logged_command: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # シーン構築
    # ------------------------------------------------------------------
    def build_scene(self) -> None:
        """Warehouse シーンと G1 をステージへ配置する。"""
        import isaacsim.core.utils.prims as prim_utils
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        from pxr import UsdLux

        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation
        from isaaclab_assets.robots.unitree import G1_CFG

        assets_root = get_assets_root_path()
        if assets_root is None:
            raise RuntimeError(
                "[G1] アセットサーバーに接続できません。ネットワーク接続を確認してください。"
            )

        if self._config.use_warehouse:
            warehouse_path = assets_root + WAREHOUSE_USD
            add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")
            print(f"[OK] Warehouse シーンを読み込みました: {warehouse_path}")
        else:
            # 平地のみ
            ground = sim_utils.GroundPlaneCfg()
            ground.func("/World/GroundPlane", ground)
            print("[OK] 平地を生成しました")

        # 照明（Warehouse には照明が含まれるが、暗い場合の補助として追加）
        light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
        light.func("/World/DomeLight", light)

        # G1 を配置
        x, y = self._config.spawn_xy
        robot_cfg = G1_CFG.replace(prim_path="/World/G1")
        robot_cfg.init_state = robot_cfg.init_state.replace(pos=(x, y, SPAWN_HEIGHT))
        self._robot = Articulation(robot_cfg)
        print(f"[OK] G1 を配置しました: pos=({x}, {y}, {SPAWN_HEIGHT})")

    def start_keyboard(self) -> None:
        """キーボード操作を開始する。"""
        from .keyboard import KeyboardCommander

        self._commander = KeyboardCommander()

    # ------------------------------------------------------------------
    # 観測の構築
    # ------------------------------------------------------------------
    def _compute_observation(self, command: VelocityCommand) -> torch.Tensor:
        """ロボットの現在状態から観測ベクトルを組み立てる。

        IsaacLab の mdp 関数群と同じ量を計算する:
            base_lin_vel      -> root_lin_vel_b
            base_ang_vel      -> root_ang_vel_b
            projected_gravity -> projected_gravity_b
            joint_pos_rel     -> joint_pos - default_joint_pos
            joint_vel_rel     -> joint_vel - default_joint_vel
        """
        data = self._robot.data
        # Isaac Sim 6.0 の Articulation.data は warp 配列を返すため、
        # IsaacLab の mdp 関数と同様に wp.to_torch() で変換してから添字を取る。
        # （warp 配列は直接の要素添字に対応していない）
        lin_vel_b = wp.to_torch(data.root_lin_vel_b)[0]
        ang_vel_b = wp.to_torch(data.root_ang_vel_b)[0]
        gravity_b = wp.to_torch(data.projected_gravity_b)[0]

        joint_pos_rel = (wp.to_torch(data.joint_pos) - wp.to_torch(data.default_joint_pos))[0]
        joint_vel_rel = (wp.to_torch(data.joint_vel) - wp.to_torch(data.default_joint_vel))[0]

        return self._policy.build_observation(
            base_lin_vel_b=lin_vel_b,
            base_ang_vel_b=ang_vel_b,
            projected_gravity_b=gravity_b,
            command=command.as_tuple(),
            joint_pos_rel=joint_pos_rel,
            joint_vel=joint_vel_rel,
        )

    # ------------------------------------------------------------------
    # 実行ループ
    # ------------------------------------------------------------------
    def run(self, sim, simulation_app) -> None:
        """シミュレーションループを回す。

        Args:
            sim: SimulationContext
            simulation_app: AppLauncher が返す app（実行継続の判定に使う）
        """
        self._robot.reset()
        self._policy.reset()
        self._step_count = 0
        print(
            f"[OK] シミュレーションを開始します "
            f"(is_running={simulation_app.is_running()}, is_playing={sim.is_playing()})"
        )

        while simulation_app.is_running():
            # キーボードからコマンドを取得して送信先へ渡す
            if self._commander is not None:
                self._sink.send(self._commander.poll())
            command = self._sink.latest

            # 指令が変わったときだけ表示する（キー入力が効いているかの確認用）
            if command.as_tuple() != self._last_logged_command:
                print(
                    f"[G1] 指令変更: vx={command.vx:+.2f} vy={command.vy:+.2f} "
                    f"yaw={command.yaw_rate:+.2f}"
                )
                self._last_logged_command = command.as_tuple()

            # 制御周期 (50Hz) ごとにポリシーを評価
            obs = self._compute_observation(command)
            action = self._policy.act(obs)
            default_joint_pos = wp.to_torch(self._robot.data.default_joint_pos)[0]
            targets = self._policy.joint_position_targets(action, default_joint_pos)
            self._robot.set_joint_position_target(targets.unsqueeze(0))
            self._robot.write_data_to_sim()

            # 物理は 200Hz で decimation 回進める
            for _ in range(DECIMATION):
                sim.step(render=False)
            sim.render()
            self._robot.update(CONTROL_DT)

            # Kit の UI イベント（キーボード入力を含む）を処理する。
            # これを呼ばないとキーボードのコールバックが発火せず操作できない。
            simulation_app.update()

            self._step_count += 1
            # 最初の数ステップは必ず出力してループ突入を確認できるようにする
            if self._step_count <= 3:
                print(f"[G1] step {self._step_count} 実行")
            # 5 秒ごとに状態を出力
            if self._step_count % 250 == 0:
                pos = wp.to_torch(self._robot.data.root_pos_w)[0]
                print(
                    f"[G1] cmd=({command.vx:+.2f}, {command.vy:+.2f}, {command.yaw_rate:+.2f}) "
                    f"pos=({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f})"
                )

        print(f"[WARN] ループを抜けました (step={self._step_count})")

    def close(self) -> None:
        """リソースを解放する。"""
        if self._commander is not None:
            self._commander.close()
