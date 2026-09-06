"""`unitree_rl_gym` の学習済み 12DoF 歩行ポリシーを MuJoCo で回す。

**中身は公式 `deploy/deploy_mujoco/deploy_mujoco.py` の制御ループそのまま。**
自作したのは以下の 3 点だけで、歩行の中身には一切手を入れていない:

1. 速度指令 `(vx, vy, wz)` を**外から差せる**ようにした（公式は config 固定）
2. 部屋の MJCF を差せるようにした（公式は `scene.xml` 固定）
3. 姿勢と転倒を外から読めるようにした

観測 47 次元の組み立ても、`kps/kds` も、`action_scale` も、位相の周期 0.8s も
公式の `g1.yaml` をそのまま読んでいる。**ここを触ると歩かなくなる。**

手元の Mac（M4 / Python 3.10 / torch / mujoco 3.12）での実測（2026-09-02）:

| 指令 | 5 秒後 |
|---|---|
| 前進 0.5 m/s | +2.23 m（実速度 0.45 m/s） |
| その場旋回 0.5 rad/s | yaw +133°（実 0.46 rad/s） |
| 横移動 0.3 m/s | +1.07 m（実 0.21 m/s） |
| 曲線 (0.3, 0, 0.5) | yaw +129° / 移動 1.10 m |

sim 16 秒が実時間 0.2 秒（約 80 倍速）。指令を 5 区間切り替えても転倒しない。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from nav.protocol import Pose2D
from sim.rooms import DynamicObstacle, Room

ASSET_DIR = Path(__file__).resolve().parent / "assets"
POLICY_PATH = ASSET_DIR / "motion.pt"
CONFIG_PATH = ASSET_DIR / "g1.yaml"

# 歩容の位相の周期[s]。公式 deploy_mujoco.py に直書きされている値。
GAIT_PERIOD_S = 0.8

# 転倒とみなす骨盤の高さ[m]。立っているときは 0.75 前後、転ぶと 0.3 を切る。
# 0.6 は公式のデモにも実測にも出てこない自前の閾値だが、
# 実測で「OK な区間はすべて 0.70 以上、FELL はすべて 0.35 以下」と
# はっきり割れていたので、その間を取った。
FALLEN_HEIGHT_M = 0.6

# 指令の上限。ポリシーの学習時の指令範囲を超えると歩容が壊れる。
#
# 実測で転ばずに歩けたのは前進 0.5 / 横 0.3 / 旋回 0.5（2026-09-02）。
# 前進と旋回はそこに 0.1 の余裕を足す。**横だけ余裕を足さない**のは、
# `sim/slam_service.py` が横移動を一度も指令しない（前進と旋回だけで目標へ向かう）ため。
# 使っていない軸に測っていない値を入れる理由が無い。横を使うようになったら測ってから上げる。
MAX_VX_MPS = 0.6
MAX_VY_MPS = 0.3
MAX_WZ_RPS = 0.6

# Livox Mid-360 の取り付け。**この 3 つの値は実測で決まっている**（2026-09-02）:
# - 骨盤基準の高さ 0.557m ＝ 立位で world z=1.35m。12dof モデルの胴体頂点が
#   z=1.22 なので、これより低いと全レイが自機に当たる
# - 前へ 0.15m。同じ理由
# - 前傾 20°。Mid-360 の縦視野は -7°〜+52°（上向きが主）で、
#   水平のままだと足元の障害物が 1 点も見えない
LIDAR_SITE_NAME = "mid360"
LIDAR_MOUNT_XYZ = (0.15, 0.0, 0.557)
LIDAR_PITCH_DEG = 20.0

# 機体を追うカメラ。`trackcom` なので pos は重心からの world 基準オフセット。
# 向きは pos から重心を見る方向を自動で計算する（`_quat_looking_at_origin_from`）。
# 後ろ 4m・上 2.5m だと機体と行き先の両方が入る（オフスクリーン描画で確認）。
CHASE_CAMERA_NAME = "chase"
CHASE_CAMERA_OFFSET = (-4.0, 0.0, 2.5)
CHASE_CAMERA_FOVY = 60.0

# レイの間引き。1 は 24,000 本（3.2ms）、4 で 6,000 本（4.1ms）。
# 障害物の有無を見るだけなら 4 で十分で、実機 10Hz に対して 24 倍速。
LIDAR_DOWNSAMPLE = 4

# これより遠い点は「当たらなかった」とみなす[m]。Mid-360 の実力は 40m だが、
# 障害物判定に使うのは数 m 以内なので短く切って計算を減らす。
LIDAR_CUTOFF_M = 15.0


@dataclass(frozen=True)
class WalkConfig:
    """公式 `g1.yaml` の中身。**自分で値を決めない。**"""

    simulation_dt: float
    control_decimation: int
    num_actions: int
    num_obs: int
    kps: np.ndarray
    kds: np.ndarray
    default_angles: np.ndarray
    cmd_scale: np.ndarray
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float

    @classmethod
    def load(cls, path: Path = CONFIG_PATH) -> "WalkConfig":
        import yaml

        if not path.exists():
            raise FileNotFoundError(
                f"{path} が無い。先に `bash Navigation/sim/fetch_assets.sh` を実行すること"
            )
        raw = yaml.safe_load(path.read_text())
        return cls(
            simulation_dt=float(raw["simulation_dt"]),
            control_decimation=int(raw["control_decimation"]),
            num_actions=int(raw["num_actions"]),
            num_obs=int(raw["num_obs"]),
            kps=np.array(raw["kps"], np.float32),
            kds=np.array(raw["kds"], np.float32),
            default_angles=np.array(raw["default_angles"], np.float32),
            cmd_scale=np.array(raw["cmd_scale"], np.float32),
            ang_vel_scale=float(raw["ang_vel_scale"]),
            dof_pos_scale=float(raw["dof_pos_scale"]),
            dof_vel_scale=float(raw["dof_vel_scale"]),
            action_scale=float(raw["action_scale"]),
        )


class G1Walker:
    """MuJoCo の中で G1 を歩かせる。速度指令を受け、姿勢を返す。

    `sim/slam_service.py` から見ると「PC1 の運動制御」に相当する層。
    ナビ層はこのクラスを直接には触らない。
    """

    def __init__(self, scene, *, config: WalkConfig | None = None) -> None:
        """`scene` は MJCF のパスか、`build_model()` が返したモデル。"""

        import mujoco
        import torch

        if not POLICY_PATH.exists():
            raise FileNotFoundError(
                f"{POLICY_PATH} が無い。先に `bash Navigation/sim/fetch_assets.sh` を実行すること"
            )
        self._mujoco = mujoco
        self._config = config or WalkConfig.load()
        self._model = (
            mujoco.MjModel.from_xml_path(str(scene))
            if isinstance(scene, (str, Path))
            else scene
        )
        self._model.opt.timestep = self._config.simulation_dt
        self._data = mujoco.MjData(self._model)
        if self._model.nkey > 0:
            # 部屋が置き場所を指定していればそこから始める（`sim/rooms.py` の keyframe）
            mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
        mujoco.mj_forward(self._model, self._data)

        self._policy = torch.jit.load(str(POLICY_PATH))
        self._torch = torch
        self._action = np.zeros(self._config.num_actions, np.float32)
        self._target = self._config.default_angles.copy()
        self._command = np.zeros(3, np.float32)
        self._steps = 0
        self._peak_action = 0.0

    # ------------------------------------------------------------------ 入出力

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        """速度指令[m/s, m/s, rad/s]。学習時の範囲へ丸める。

        丸めるのは、範囲外を渡すと歩容が壊れて転ぶため。
        「指令どおりに動かないこと」は呼び側（`slam_service`）が
        位置のフィードバックで吸収する。
        """

        self._command = np.array(
            [
                _clamp(vx, -MAX_VX_MPS, MAX_VX_MPS),
                _clamp(vy, -MAX_VY_MPS, MAX_VY_MPS),
                _clamp(wz, -MAX_WZ_RPS, MAX_WZ_RPS),
            ],
            np.float32,
        )

    def stop(self) -> None:
        """その場で足踏みさせる。**関節を固めるのではない。**

        指令 0 でもポリシーは動き続け、両足で立ってバランスを取る。
        制御を止めると重力に負けて転ぶ（29DoF で実測済み）。
        """

        self.set_command(0.0, 0.0, 0.0)

    def step(self, seconds: float) -> None:
        """指定秒ぶん進める。公式の制御ループそのまま。"""

        for _ in range(max(1, round(seconds / self._config.simulation_dt))):
            self._step_once()

    @property
    def pose(self) -> Pose2D:
        """骨盤の world 座標での平面姿勢。実機の SLAM が返す `currentPose` に相当する。"""

        x, y = float(self._data.qpos[0]), float(self._data.qpos[1])
        return Pose2D(x, y, _yaw_of(self._data.qpos[3:7]))

    @property
    def height(self) -> float:
        return float(self._data.qpos[2])

    @property
    def has_fallen(self) -> bool:
        return self.height < FALLEN_HEIGHT_M

    @property
    def sim_time(self) -> float:
        return self._steps * self._config.simulation_dt

    @property
    def peak_action(self) -> float:
        """これまでに出たポリシー出力の絶対値の最大。発散の目印。

        正常なら 3 前後。10 を超えたら壊れている（29DoF の失敗時は 2.6e+07 だった）。
        """

        return self._peak_action

    @property
    def model(self):
        """MuJoCo のモデル。ビューアと LiDAR がここを見る。"""

        return self._model

    @property
    def data(self):
        return self._data

    # -------------------------------------------------------------- 制御ループ

    def _step_once(self) -> None:
        config = self._config
        # PD。公式と同じく毎 physics step で計算する
        self._data.ctrl[:] = (
            (self._target - self._data.qpos[7:]) * config.kps - self._data.qvel[6:] * config.kds
        )
        self._mujoco.mj_step(self._model, self._data)
        self._steps += 1
        if self._steps % config.control_decimation:
            return

        phase = (self._steps * config.simulation_dt) % GAIT_PERIOD_S / GAIT_PERIOD_S
        actions = config.num_actions
        obs = np.zeros(config.num_obs, np.float32)
        obs[0:3] = self._data.qvel[3:6] * config.ang_vel_scale
        obs[3:6] = _projected_gravity(self._data.qpos[3:7])
        obs[6:9] = self._command * config.cmd_scale
        obs[9 : 9 + actions] = (self._data.qpos[7:] - config.default_angles) * config.dof_pos_scale
        obs[9 + actions : 9 + 2 * actions] = self._data.qvel[6:] * config.dof_vel_scale
        obs[9 + 2 * actions : 9 + 3 * actions] = self._action
        obs[9 + 3 * actions : 9 + 3 * actions + 2] = [
            math.sin(2 * math.pi * phase),
            math.cos(2 * math.pi * phase),
        ]
        tensor = self._torch.from_numpy(obs).unsqueeze(0)
        self._action = self._policy(tensor).detach().numpy().squeeze()
        self._peak_action = max(self._peak_action, float(np.abs(self._action).max()))
        self._target = self._action * config.action_scale + config.default_angles


def build_model(room: Room, obstacles: tuple[DynamicObstacle, ...] = ()):
    """部屋 + 機体 + LiDAR の site + 動く障害物、を組んだ MuJoCo モデルを返す。

    site と mocap body は **`MjSpec` で後から差し込む**。MJCF の `<include>` は
    トップレベルで併合されるだけで、include 先の body（pelvis）の中には
    要素を足せないため。`g1_12dof.xml` 自体は書き換えない
    （`fetch_assets.sh` が取ってきた上流のファイルに手を入れない）。
    """

    import mujoco

    spec = mujoco.MjSpec.from_file(str(room.write_scene()))

    # 12DoF モデルに torso_link は無い（上半身は pelvis に溶かし込まれている）ので
    # LiDAR は pelvis に付ける。
    site = spec.body("pelvis").add_site()
    site.name = LIDAR_SITE_NAME
    site.pos = list(LIDAR_MOUNT_XYZ)
    half = math.radians(LIDAR_PITCH_DEG) / 2
    site.quat = [math.cos(half), 0.0, math.sin(half), 0.0]  # y 軸まわり＝前傾

    # 歩いている機体を追うカメラ。ビューアで `[` / `]` で切り替えられる。
    # `trackcom` は位置だけ機体に追従し、向きは world 基準のまま。機体の
    # ロール・ピッチに合わせて回らないので、歩容で画面が揺れない。
    chase = spec.body("pelvis").add_camera()
    chase.name = CHASE_CAMERA_NAME
    chase.mode = mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM
    chase.pos = list(CHASE_CAMERA_OFFSET)
    chase.fovy = CHASE_CAMERA_FOVY
    chase.quat = _quat_looking_at_origin_from(CHASE_CAMERA_OFFSET)

    for index, obstacle in enumerate(obstacles):
        body = spec.worldbody.add_body()
        body.name = f"obstacle_{index}"
        body.mocap = True
        body.pos = list(obstacle.position_at(0.0))
        geom = body.add_geom()
        geom.name = f"obstacle_{index}_geom"
        geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        geom.size = [obstacle.radius, obstacle.height / 2, 0.0]
        geom.rgba = [0.85, 0.35, 0.25, 1.0]
    return spec.compile()


class Mid360:
    """`mujoco_lidar` で Livox Mid-360 を再現する。

    **自作の当たり判定は書かない。** 実機と同じレイのパターン（Livox の
    非反復スキャン）で MuJoCo のシーンを撃ち、当たった点を world 座標で返す。
    """

    def __init__(self, model, *, downsample: int = LIDAR_DOWNSAMPLE) -> None:
        from mujoco_lidar.lidar_wrapper import MjLidarWrapper
        from mujoco_lidar.scan_gen import LivoxGenerator

        self._theta, self._phi = LivoxGenerator("mid360").sample_ray_angles(
            downsample=downsample
        )
        # backend は cpu 一択。taichi は Mac で tibvh が無く ImportError、
        # numpy という選択肢は存在しない（cpu/taichi/jax/warp のみ）。
        self._wrapper = MjLidarWrapper(
            model, site_name=LIDAR_SITE_NAME, backend="cpu", cutoff_dist=LIDAR_CUTOFF_M
        )
        self._site_id = _site_id(model, LIDAR_SITE_NAME)

    @property
    def ray_count(self) -> int:
        return len(self._theta)

    def scan(self, data) -> np.ndarray:
        """1 スキャンぶんの当たった点を world 座標 (N,3) で返す。

        当たらなかったレイ（距離 0、または cutoff 超え）は落とす。
        軸に平行なレイで 1/0 が出るため、警告は握りつぶす
        （スラブ法の inf は正常な中間値で、結果には現れない）。
        """

        with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
            self._wrapper.trace_rays(data, self._theta, self._phi, LIDAR_SITE_NAME)
            local = self._wrapper.get_hit_points()
            distance = self._wrapper.get_distances()
            rotation = data.site_xmat[self._site_id].reshape(3, 3)
            world = data.site_xpos[self._site_id] + local @ rotation.T
            hit = (
                (distance > 0.01)
                & (distance < LIDAR_CUTOFF_M - 0.5)
                & np.isfinite(world).all(axis=1)
            )
            result = world[hit]
        # ⚠️ この errstate を抜けても、立った浮動小数の例外フラグは残る。
        # `mujoco_lidar` はスラブ法の交差判定で軸に平行なレイに対して
        # **正当に 0 除算する**。numpy はフラグを立てたまま返し、あとの無関係な
        # 演算がそれを見て警告する。空打ち（スカラも配列も）では消せなかったので、
        # フラグを気にする側（`slam_service.observe_obstacle`）で黙らせている。
        # ここで返す点は上の `isfinite` で有限であることを確かめてある。
        return result


def _site_id(model, name: str) -> int:
    import mujoco

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError(f"site {name!r} がモデルに無い。build_model() を通したか確認すること")
    return site_id


def _quat_looking_at_origin_from(offset) -> list[float]:
    """`offset` の位置から原点を見るカメラの四元数 (w, x, y, z)。

    MuJoCo のカメラは **-z 方向を見て +y が画面の上**。その 3 軸を直接組んで
    `scipy` に四元数へ直させる。手で四元数を掛け合わせると必ず取り違える
    （実際に一度間違えて、カメラが床を向いた）。
    """

    from scipy.spatial.transform import Rotation

    position = np.asarray(offset, dtype=float)
    forward = -position / np.linalg.norm(position)   # 原点へ向かう向き
    z_axis = -forward                                # カメラの +z は視線の逆
    x_axis = np.cross([0.0, 0.0, 1.0], z_axis)       # world の上と直交する「右」
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)                # 画面の「上」
    rotation = Rotation.from_matrix(np.stack([x_axis, y_axis, z_axis], axis=1))
    q_x, q_y, q_z, q_w = rotation.as_quat()
    return [q_w, q_x, q_y, q_z]                      # MuJoCo は (w,x,y,z) 順


def _projected_gravity(quaternion) -> np.ndarray:
    """胴体座標系から見た重力方向。公式 `get_gravity_orientation` と同じ式。"""

    q_w, q_x, q_y, q_z = quaternion
    return np.array(
        [
            2 * (-q_z * q_x + q_w * q_y),
            -2 * (q_z * q_y + q_w * q_x),
            1 - 2 * (q_w * q_w + q_z * q_z),
        ],
        np.float32,
    )


def _yaw_of(quaternion) -> float:
    """MuJoCo の (w,x,y,z) 四元数から yaw[rad]。

    `nav/protocol.py` の `quaternion_to_yaw` は (x,y,z,w) 順の scipy 形式なので
    そのままは使えない。並べ替えるより 1 行で書いたほうが取り違えが起きない。
    """

    q_w, q_x, q_y, q_z = quaternion
    return math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _main() -> int:
    """`python -m sim.g1_walker --selftest` で歩行だけを確かめる。

    ナビも地図も通さずポリシーだけを見る。実測値（このファイルの docstring の表）と
    比べて、資産の取得や環境が壊れていないことを確かめるのに使う。
    """

    import argparse
    import time

    from sim.rooms import EMPTY_ROOM

    parser = argparse.ArgumentParser(description="歩行ポリシーの自己診断")
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument(
        "--command", action="append", default=[], metavar="VX,VY,WZ@SEC",
        help="速度指令とその継続秒。複数指定できる",
    )
    args = parser.parse_args()

    schedule = [_parse_command(text) for text in args.command] or [
        ((0.5, 0.0, 0.0), 5.0),
        ((0.0, 0.0, 0.5), 5.0),
        ((0.0, 0.3, 0.0), 5.0),
        ((0.3, 0.0, 0.5), 5.0),
    ]
    walker = G1Walker(EMPTY_ROOM.write_scene())
    started = time.time()
    print(f"部屋 {EMPTY_ROOM.name} / 出発 ({walker.pose.x:+.2f}, {walker.pose.y:+.2f})")
    for command, seconds in schedule:
        before = walker.pose
        walker.set_command(*command)
        walker.step(seconds)
        after = walker.pose
        moved = math.hypot(after.x - before.x, after.y - before.y)
        turned = math.degrees(_wrap(after.yaw - before.yaw))
        state = "FELL" if walker.has_fallen else "OK"
        print(
            f"  {command!s:18s} {seconds:4.1f}s: 移動 {moved:5.2f}m "
            f"({after.x - before.x:+.2f},{after.y - before.y:+.2f})  "
            f"回頭 {turned:+6.1f}°  高さ {walker.height:.2f}  {state}"
        )
    total = sum(seconds for _, seconds in schedule)
    print(
        f"  sim {total:.0f}s を実時間 {time.time() - started:.1f}s"
        f"（{total / (time.time() - started):.0f} 倍速） / |action|max {walker.peak_action:.2f}"
    )
    return 1 if walker.has_fallen else 0


def _parse_command(text: str) -> tuple[tuple[float, float, float], float]:
    body, _, seconds = text.partition("@")
    values = [float(piece) for piece in body.split(",")]
    if len(values) != 3 or not seconds:
        raise SystemExit(f"--command は VX,VY,WZ@SEC で指定する: {text!r}")
    return (values[0], values[1], values[2]), float(seconds)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


if __name__ == "__main__":
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(_main())
