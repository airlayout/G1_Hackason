"""ボタン押しの学習環境（教師）の設定。

Isaac Lab 3.0 は、シミュレーターを起動する前にこの設定を読み込むので、ここでは Isaac Sim の部品（pxr、
isaac_world など）を import しない。乗り場の USD とボタンの関節機構は、起動後に press_env.py が作る。
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.converters import UrdfConverterCfg
from isaaclab.utils.configclass import configclass

from ..paths import REPO_ROOT, use_yada
from ..proprio import NUM_ACTIONS, POLICY_OBS_DIM
from ..rewards import RewardWeights
from ..spec import TARGET_BUTTON, load_task_spec

# 評価環境（Yada）と同じ物理の刻みと制御の周期（0.002 s × 10 = 50 Hz）
PHYSICS_DT = 0.002
DECIMATION = 10
# 観測の数（教師の方策 / critic）。中身は press_env.py の _get_observations。
# 方策には、実機でも用意できるもの（体の感覚 + ボタンの面の位置・指先の位置）だけを渡す。ボタンの沈み・点灯・
# 点灯のあとの時間は実機では直接は取れないので critic にだけ渡す（押した手応えは、目標と角度の差から分かる）。
# こうしておくと、教師を「画像の検出 + 方策」の形でそのまま評価環境の Agent にでき、生徒は面の位置の推定だけを
# 画像から学べばよい（2026-10-20）
CRITIC_OBS_DIM = POLICY_OBS_DIM + 8

# 関節の名前 → configs/contest.yaml の gains の組（Yada の sim/isaac/isaac_world.py の GAIN_GROUPS と同じ）
GAIN_GROUPS: dict[str, list[str]] = {
    "legs": [".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
    "waist": ["waist_.*"],
    "arm": [".*_shoulder_.*", ".*_elbow_.*"],
    "wrist": [".*_wrist_.*"],
}
# URDF を変換するときの関節のドライブ（アクチュエータの設定で上書きされる）。Yada の HOLD_STIFFNESS / HOLD_DAMPING と同じ
HOLD_STIFFNESS = 400.0
HOLD_DAMPING = 20.0


def robot_cfg(target: str = TARGET_BUTTON) -> ArticulationCfg:
    """G1（腰を固定）。モデル・PD の強さ・初期の姿勢は評価環境と同じ。"""
    use_yada()
    from common.j1gen_bridge import j1gen_config

    spec = load_task_spec(target)
    robot = j1gen_config("robot.yaml")
    urdf = REPO_ROOT / robot["model"]["urdf_path"]
    gains = spec.contest_cfg["gains"]
    return ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(urdf),
            usd_dir=str(REPO_ROOT / "_local" / "button_press_yada" / "usd"),
            fix_base=True,
            merge_fixed_joints=True,
            self_collision=True,
            # ⚠️ URDF の変換の self_collision=True は、変換した USD に反映されない（PhysX の自己衝突の属性が付かず、
            #    腕が胴体を通り抜ける）。Isaac Lab が spawn のときに付ける articulation_props で有効にする。
            #    これが無いと、方策は腕を胴体に通して振り上げる動きを覚え、MuJoCo では胴体に当たって止まった（2026-10-20）
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True),
            # 接触の報告は、測るリンクにだけ press_env.py で付ける（入れ子のリンクには、この設定では付かない）
            activate_contact_sensors=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=HOLD_STIFFNESS, damping=HOLD_DAMPING)
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, float(spec.scene_cfg["pelvis_height"])),
                                                   joint_pos=dict(spec.initial_pose)),
        actuators={
            g: ImplicitActuatorCfg(joint_names_expr=exprs, stiffness=float(gains[g]["kp"]),
                                   damping=float(gains[g]["kd"]))
            for g, exprs in GAIN_GROUPS.items()
        },
    )


@configclass
class DomainRandCfg:
    """乱し（configs/contest.yaml の realism を目安にした値）。"""

    # ロボットの立つ位置のずらし幅を、評価の条件の範囲より少し広げる割合
    offset_margin: float = 0.2
    # 右腕の PD の強さの倍率（関節ごと）と、回転子の見かけの慣性 [kg·m²]
    kp_scale: tuple[float, float] = (0.9, 1.1)
    armature: tuple[float, float] = (0.001, 0.03)
    # 指令が 1 周期遅れる環境の割合（realism.latency.command_s = 0〜0.02 s ≒ 0〜1 周期）
    command_delay_prob: float = 0.5
    # 関節のセンサのノイズの標準偏差（観測だけ）
    q_noise: float = 0.0005
    dq_noise: float = 0.01
    # 試行の始めの、右腕の角度のずれ [rad]
    init_q_noise: float = 0.03
    # ボタンのばねの強さ [N/m] と、行き止まりまでの沈み [m]（評価は 1000 N/m・3 mm）。
    # 固定だと「決まった深さまで押す」ことを覚え、行き止まりの硬さが違う MuJoCo では力が 2 倍以上になった
    # （Isaac Sim 7 N → MuJoCo 14 N、2026-10-20）。ばらつかせて、手応え（目標と角度の差）で押す力を加減させる
    button_stiffness: tuple[float, float] = (600.0, 1500.0)
    button_travel: tuple[float, float] = (0.0025, 0.0045)


@configclass
class ButtonPressEnvCfg(DirectRLEnvCfg):
    """右腕で一般用の ▼ を押す（教師。ボタンの位置を観測に入れる）。"""

    target: str = TARGET_BUTTON
    # 使った学習の設定（configs/profiles/<名前>.yaml。環境変数 BUTTON_PRESS_PROFILE で選ぶ）。ログの params/env.yaml に残る
    profile: str = "default"
    episode_length_s: float = 8.0
    decimation: int = DECIMATION
    action_space: int = NUM_ACTIONS
    observation_space: int = POLICY_OBS_DIM
    state_space: int = CRITIC_OBS_DIM

    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=DECIMATION)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=4.0, replicate_physics=True)

    robot: ArticulationCfg = robot_cfg()
    rewards: RewardWeights = RewardWeights()
    rand: DomainRandCfg = DomainRandCfg()
    viewer: ViewerCfg = ViewerCfg(eye=(-1.3, -1.1, 1.7), lookat=(0.3, 0.0, 0.9))

    # 押す力がこれを超えたら大きく減点する（configs/contest.yaml の press_force.max_n の倍）
    force_abort_scale: float = 2.0
    # 違うボタン・押しすぎで試行を打ち切るか。False（既定）では減点だけで続ける。
    # 毎周期の項が減点だけなので、打ち切ると「わざと違うボタンを押して早く終える」ことを覚える（2026-10-19。FAILURES.md）
    terminate_on_failure: bool = False
    # 中指の先で押したとみなす範囲: 指先の球の表面と面のすき間 [m]、横のずれ（ボタンの半径に足す）[m]
    tip_gap_m: float = 0.004
    tip_lateral_m: float = 0.002
    # 終わった試行の記録を env.episode_records に残す（評価用。学習では False）
    record_episodes: bool = False
    # 動画用のカメラ（scripts/record_progress.py が使う。学習では None）。環境の外の /World/VideoCamera に 1 台置き、
    # set_world_poses_from_view で動かす。Isaac Lab の viewport のカメラは、毎コマ動かすと描画に遅れて反映されたため
    video_camera_size: tuple[int, int] | None = None
