"""G1 デジタルツインのシーン構築とシミュレーションループ。

Warehouse シーンに G1 を配置し、キーボードからの速度コマンドで歩かせる。
"""

from __future__ import annotations

import time
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


# ROS へスキャンを流す周期。50Hz は slam_toolbox には過剰で負荷も高いため
# 10Hz に間引く（実機の 2D LiDAR も 10〜20Hz 程度）。
SCAN_PUBLISH_EVERY: int = 5  # 50Hz / 5 = 10Hz


@dataclass
class RunnerConfig:
    """ランナーの設定。"""

    # Warehouse を使うか（False なら平地のみ）
    use_warehouse: bool = True
    # G1 のスポーン位置 (x, y)
    spawn_xy: tuple[float, float] = (0.0, 0.0)
    device: str = "cuda:0"
    # ROS 2 へ /scan と /odom を配信するか（SLAM / Nav2 で使う）
    enable_ros: bool = False
    # 速度指令の供給源: "keyboard" / "patrol"（自動巡回） / "ros"（Nav2）
    command_source: str = "keyboard"
    # 自動巡回の乱数種（再現性のため）
    patrol_seed: int = 0
    # この制御ステップ数で自動終了する（0 なら無制限）。
    # 人が見ていない自動巡回に時間制限をかけるために使う。
    max_steps: int = 0


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
        # SLAM / Nav2 用（enable_ros のときだけ生成する）
        self._lidar = None
        self._ros = None
        self._patrol = None
        # 直近の LiDAR スキャン（間引くため前回値を保持する）
        self._latest_scan = None
        # 実時間比の計測用
        self._last_rate_time: float = 0.0

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

        # LiDAR は sim.reset() より前に構築する必要がある（センサの登録のため）
        if self._config.enable_ros:
            from .lidar import G1Lidar

            # 平地のときは Warehouse が無いので地面を raycast 対象にする
            mesh_paths = (
                ["/World/Warehouse"]
                if self._config.use_warehouse
                else ["/World/GroundPlane"]
            )
            self._lidar = G1Lidar(
                robot_prim_path="/World/G1", mesh_prim_paths=mesh_paths
            )

    def start_keyboard(self) -> None:
        """キーボード操作を開始する。"""
        from .keyboard import KeyboardCommander

        self._commander = KeyboardCommander()

    def start_ros(self) -> None:
        """ROS 2 ブリッジを開始する。sim.reset() の後に呼ぶこと。"""
        from .ros_bridge import RosBridge

        self._ros = RosBridge()

    def start_patrol(self) -> None:
        """自動巡回を開始する。"""
        from .patrol import AutoPatrol

        self._patrol = AutoPatrol(seed=self._config.patrol_seed)
        print("[OK] 自動巡回モードで動作します")

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

    def _is_fallen(self) -> bool:
        """転倒しているかを判定する。

        胴体座標系での重力方向 (projected_gravity_b) を見る。直立していれば
        重力は真下 (0, 0, -1) を向く。大きく傾くと z 成分が 0 に近づく。
        胴体の高さも併せて見て、しゃがみ込みと転倒を区別する。
        """
        data = self._robot.data
        gravity_b = wp.to_torch(data.projected_gravity_b)[0]
        height = float(wp.to_torch(data.root_pos_w)[0][2])
        # cos(60 度) = 0.5。60 度以上傾いていれば転倒とみなす。
        tilted = float(gravity_b[2]) > -0.5
        # 通常の歩行高さは 0.74 付近。0.4 を下回れば倒れている。
        too_low = height < 0.4
        return tilted or too_low

    # ------------------------------------------------------------------
    # ROS への配信
    # ------------------------------------------------------------------
    def _publish_ros(self, scan) -> None:
        """odom と scan を ROS へ流す。

        odom は Sim の真値をそのまま使う（ドリフトが無いので SLAM が安定する）。
        scan は 50Hz では過剰なので SCAN_PUBLISH_EVERY 周期に間引く。
        """
        from .ros_bridge import OdomState, quat_wxyz_to_yaw

        # 先に /clock を配信する。以降のメッセージのタイムスタンプは
        # ここで設定したシム内時刻に揃う。
        self._ros.publish_clock(self._step_count * CONTROL_DT)

        data = self._robot.data
        pos = wp.to_torch(data.root_pos_w)[0]
        quat = wp.to_torch(data.root_quat_w)[0]
        lin_vel_b = wp.to_torch(data.root_lin_vel_b)[0]
        ang_vel_b = wp.to_torch(data.root_ang_vel_b)[0]

        # root_quat_w は (w, x, y, z) 順（実測で確認済み）
        yaw = quat_wxyz_to_yaw(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        )

        self._ros.publish_odom(
            OdomState(
                x=float(pos[0]),
                y=float(pos[1]),
                yaw=yaw,
                vx=float(lin_vel_b[0]),
                vy=float(lin_vel_b[1]),
                yaw_rate=float(ang_vel_b[2]),
            )
        )

        # scan は呼び出し側で既に SCAN_PUBLISH_EVERY に間引かれている。
        # 更新された周期だけ配信する（同じスキャンを重複配信しないため）。
        if scan is not None and self._step_count % SCAN_PUBLISH_EVERY == 0:
            self._ros.publish_scan(scan)

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
        self._last_rate_time = time.perf_counter()
        print(
            f"[OK] シミュレーションを開始します "
            f"(is_running={simulation_app.is_running()}, is_playing={sim.is_playing()})"
        )

        while simulation_app.is_running():
            # LiDAR を更新する（巡回の判断と /scan 配信の両方で使う）。
            # レイキャストは 1 回 76 ms かかり 50Hz には収まらないため、
            # 配信周期 (10Hz) に合わせて間引く。間の周期では前回値を使う。
            if self._lidar is not None and self._step_count % SCAN_PUBLISH_EVERY == 0:
                self._lidar.update(CONTROL_DT * SCAN_PUBLISH_EVERY)
                self._latest_scan = self._lidar.read_scan()
            scan = self._latest_scan

            # 速度指令の供給源を選ぶ
            if self._patrol is not None and scan is not None:
                # 自動巡回: LiDAR を見て自分で進路を決める。
                # 位置も渡す。LiDAR は地上 1.1 m を見ているため足元の低い
                # 障害物を検出できず、「前方は開いているのに進めない」状況が
                # 起きる。位置が動いているかで足踏みを検知する。
                position = wp.to_torch(self._robot.data.root_pos_w)[0]
                self._patrol.notify_position(
                    float(position[0]), float(position[1])
                )
                self._sink.send(self._patrol.step(scan))
            elif self._config.command_source == "ros" and self._ros is not None:
                # Nav2 からの /cmd_vel。後退はポリシーが転倒するため許可しない。
                received = self._ros.latest_command
                self._sink.send(
                    VelocityCommand(
                        vx=max(0.0, received.vx),
                        vy=received.vy,
                        yaw_rate=received.yaw_rate,
                    )
                )
            elif self._commander is not None:
                # キーボードからコマンドを取得して送信先へ渡す
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

            # ROS へ配信する（odom は毎周期、scan は間引く）
            if self._ros is not None:
                self._publish_ros(scan)
                # /cmd_vel の受信コールバックを処理する
                self._ros.spin_once()

            # Kit の UI イベント（キーボード入力を含む）を処理する。
            # これを呼ばないとキーボードのコールバックが発火せず操作できない。
            simulation_app.update()

            self._step_count += 1
            # 最初の数ステップは必ず出力してループ突入を確認できるようにする
            if self._step_count <= 3:
                print(f"[G1] step {self._step_count} 実行")

            # 指定ステップ数に達したら終了する（自動巡回の時間制限）
            if 0 < self._config.max_steps <= self._step_count:
                print(f"[OK] 指定ステップ数 {self._config.max_steps} に到達しました")
                break

            # 転倒を検知したら止める。転んだまま歩き続けると
            # LiDAR が床や天井を向いて地図が壊れるため。
            if self._step_count % 10 == 0 and self._is_fallen():
                print(f"[WARN] 転倒を検知しました (step={self._step_count})。終了します")
                break
            # 実時間に対する進み具合を出す。1.0 未満なら実時間より遅い。
            # SLAM は実時間性が要るので、ここが極端に低いと地図が歪む。
            if self._step_count % 250 == 0:
                now = time.perf_counter()
                elapsed = now - self._last_rate_time
                if elapsed > 0:
                    sim_time = 250 * CONTROL_DT
                    print(
                        f"[Perf] 実時間比 {sim_time / elapsed:.2f}x "
                        f"({250 / elapsed:.1f} step/s)"
                    )
                self._last_rate_time = now

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
        if self._ros is not None:
            self._ros.close()
        if self._patrol is not None:
            stats = self._patrol.stats
            print(
                f"[Patrol] 巡回の統計: 合計 {stats.steps} step "
                f"(前進 {stats.forward_steps} / 旋回 {stats.turn_steps} / "
                f"後退 {stats.backup_steps} / 足踏み脱出 {stats.stall_recoveries} 回 / "
                f"閉じ込め脱出 {stats.confined_recoveries} 回)"
            )
