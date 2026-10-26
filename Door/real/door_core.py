"""段 5: 状態入力の生徒（段 2.5）と HOMIE の脚を実機で回すための計算（numpy だけ。PC2 の Python 3.8 でも動く）。

学習の環境（DoorMan・VIRAL6 の door_open_homie）が 1 制御ステップ（50 Hz）でしている計算を、同じ順に書き直したもの。
正しさは Isaac Sim の評価で取った正解の記録（VIRAL6 の golden_trace.py、パッチ 0050）と突き合わせて確かめる（check_golden.py）。

1 ステップの順番（シミュレータと同じ）:
  1. 観測を作る。前のステップの行動（whole_prev・a_real_prev）と HOMIE への指令（cmd_prev）を使う
  2. 生徒: 観測 151 → 鏡写し（把手が左の扉）→ 正規化 → LSTM 2 層 → MLP → 増分 17（鏡写しの側）→ 映し戻す
  3. HOMIE: 1 ステップ 86 を作り、履歴 5 と合わせた 516 → 歩く／立つの方策 → 脚と腰 15
     ⚠️ HOMIE が見る指令は前のステップのもの（生徒の出力は 1 ステップ遅れて脚に届く）
  4. 増分を積算 D（±15）→ 腕の D を今の角度 ±0.3 rad・可動域の端から 0.05 rad 内側に収める（直し B、10-26）
     → 指令 = clip(0.25·D[0:3], ±0.5)、腕の目標角 = 0.25·D[3:17]
  5. 関節の目標角 = 0.25·[HOMIE 15, D[3:17]] + 既定の角度

落とし穴（spec_actor.md・spec_homie.md）:
  - 観測の項目は設定の並びでなく名前の辞書順につながる（params.layout が持つ）
  - 腕のトルク（arm_dof_torque）はシミュレータでは常に 0 だった（＋雑音）。実機でも 0 を入れる
  - 鏡写しは項目ごとに、尺度を掛けた後・正規化の前。HOMIE の観測は鏡写しにしない
  - 腕の D の切り方（B）と立つ HOMIE の指令の消し方（8）は target_guard.py。学習（VIRAL6、パッチ 0069）と同じファイル。
    10-16 の正解の記録（golden_45150）はどちらも無い評価で取ったので、check_golden.py は両方を切って比べる
"""
from __future__ import annotations

import json
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

import target_guard as tg

NUM_DOF = 29
NUM_ACT = 17  # [前後, 横, 旋回, 左腕 7, 右腕 7]
NUM_HOMIE = 15  # 脚 12＋腰 3（SDK 0〜14）
HOMIE_STEP = 86
HOMIE_HIST = 5
LSTM_LAYERS = 2
LSTM_HIDDEN = 256
ARM_GUARD_DELTA = 0.3  # [rad] 腕の目標は今の角度からここまで（直し B。シミュレータの 1 周期の腕の動きの p99 から決めた）
ARM_GUARD_MARGIN = 0.05  # [rad] 可動域の端からの余裕（door_run_real.py の送る目標の切り方と同じ）
ARM_GUARD = (ARM_GUARD_DELTA, ARM_GUARD_MARGIN)


@dataclass(frozen=True)
class Mlp:
    layers: Tuple[Tuple[np.ndarray, np.ndarray], ...]  # (W [out, in], b [out])
    act: str  # "silu" | "elu"

    def __call__(self, x: np.ndarray) -> np.ndarray:
        y = x
        last = len(self.layers) - 1
        for i, (w, b) in enumerate(self.layers):
            y = w @ y + b
            if i < last:
                y = _silu(y) if self.act == "silu" else _elu(y)
        return y


@dataclass(frozen=True)
class HomieNet:
    encoder: Mlp  # 516 → 256 → 256 → 35（速度 3 ＋ z 32）
    actor: Mlp  # [今の 86, 速度 3, z 32] = 121 → 512 → 256 → 256 → 15

    def __call__(self, obs516: np.ndarray) -> np.ndarray:
        parts = self.encoder(obs516)
        vel, z = parts[:3], parts[3:]
        z = z / max(float(np.linalg.norm(z)), 1e-12)  # F.normalize(p=2)
        return self.actor(np.concatenate([obs516[-HOMIE_STEP:], vel, z]))


@dataclass(frozen=True)
class Params:
    layout: Dict[str, Tuple[int, int]]  # 観測の項目 → [start, stop)（辞書順）
    obs_dim: int
    mirror: bool  # 把手が左の扉なら True（観測と行動を鏡写し）
    obs_mirror_M: np.ndarray  # [151, 151]  y = M x + b
    obs_mirror_b: np.ndarray  # [151]
    act_mirror: np.ndarray  # [17, 17]
    rms_mean: np.ndarray
    rms_var: np.ndarray
    lstm: Tuple[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray], ...]  # 層ごとの (W_ih, W_hh, b_ih, b_hh)
    actor_mlp: Mlp
    homie_walk: HomieNet
    homie_stand: HomieNet
    q_default: np.ndarray  # [29]
    kp: np.ndarray
    kd: np.ndarray
    q_lower: np.ndarray
    q_upper: np.ndarray
    effort_limit: np.ndarray
    c: Dict[str, float]  # 定数（尺度・頭打ち・閾値）。export_door_policy.py が設定から写す

    @property
    def arm_dof(self) -> slice:
        return slice(NUM_HOMIE, NUM_DOF)


@dataclass(frozen=True)
class CoreState:
    h: np.ndarray  # [2, 256]
    cell: np.ndarray  # [2, 256]
    D: np.ndarray  # [17] 積算（実の向き）
    a_real_prev: np.ndarray  # [17] 前のステップの増分（実の向き）
    whole_prev: np.ndarray  # [29] 前のステップの全身の行動（脚と腰は HOMIE の出力、腕は D[3:17]）
    cmd_prev: np.ndarray  # [7] HOMIE への指令 [vx, vy, wz, 高さ, roll, pitch, yaw]（実の単位）
    homie_hist: np.ndarray  # [5, 86] 古い → 新しい
    homie_a_prev: np.ndarray  # [15]


@dataclass(frozen=True)
class Inputs:
    q: np.ndarray  # [29] 関節の角度（SDK 0〜28 の順＝方策の順）
    dq: np.ndarray  # [29]
    gyro: np.ndarray  # [3] pelvis の角速度（pelvis の座標、rad/s）
    gravity: np.ndarray  # [3] pelvis の座標で見た重力の向き（直立で [0, 0, −1]）
    door_angle: np.ndarray  # [3] [LiDAR で測った扉の角度 rad, 最後の測定が成功したか, 古さ/1.0 s]
    door_frame_loc: np.ndarray  # [9] pelvis から見た開口の中心（床、z は pelvis の高さ）の位置・tan・norm（実の向き）
    door_geometry: np.ndarray  # [7] [幅, 高さ, レバーの高さ, バックセット, lr, 1−lr, io]（実の向き）
    complete: float = 0.0  # 扉の先 1.5 m を越えたら 1（以後ずっと 1）
    arm_torque: Optional[np.ndarray] = None  # [14]。None なら 0（学習では常に 0 だった）


@dataclass(frozen=True)
class StepOut:
    q_target: np.ndarray  # [29] 関節の目標角
    obs: np.ndarray  # [151] 生徒に入れた観測（鏡写しの後・正規化の前）
    a_pol: np.ndarray  # [17] 生徒の出力（鏡写しの側）
    a_real: np.ndarray  # [17]
    homie_obs: np.ndarray  # [516]
    homie_a: np.ndarray  # [15]
    walk: bool
    cmd: np.ndarray  # [7] このステップで決まった指令（次のステップの HOMIE が見る）


def _silu(x: np.ndarray) -> np.ndarray:
    return x / (1.0 + np.exp(-x))


def _elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _mlp_from(npz, prefix: str, n: int, act: str) -> Mlp:
    return Mlp(tuple((npz[f"{prefix}{i}_w"], npz[f"{prefix}{i}_b"]) for i in range(n)), act)


def load_params(path: str, mirror: Optional[bool] = None) -> Params:
    """export_door_policy.py が書いた npz を読む。mirror を渡すと設定の値を上書きする"""
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    layout = {k: (int(a), int(b)) for k, (a, b) in meta["layout"].items()}
    lstm = tuple(
        (z[f"lstm{i}_w_ih"], z[f"lstm{i}_w_hh"], z[f"lstm{i}_b_ih"], z[f"lstm{i}_b_hh"]) for i in range(LSTM_LAYERS)
    )

    def homie(name: str) -> HomieNet:
        return HomieNet(_mlp_from(z, f"{name}_enc", 3, "elu"), _mlp_from(z, f"{name}_act", 4, "elu"))

    return Params(
        layout=layout,
        obs_dim=int(meta["obs_dim"]),
        mirror=bool(meta["mirror_left_doors"] if mirror is None else mirror),
        obs_mirror_M=z["obs_mirror_M"],
        obs_mirror_b=z["obs_mirror_b"],
        act_mirror=z["act_mirror"],
        rms_mean=z["rms_mean"],
        rms_var=z["rms_var"],
        lstm=lstm,
        actor_mlp=_mlp_from(z, "actor", 4, "silu"),
        homie_walk=homie("walk"),
        homie_stand=homie("stand"),
        q_default=z["q_default"],
        kp=z["kp"],
        kd=z["kd"],
        q_lower=z["q_lower"],
        q_upper=z["q_upper"],
        effort_limit=z["effort_limit"],
        c={k: float(v) for k, v in meta["constants"].items()},
    )


def default_cmd(p: Params, height: Optional[float] = None) -> np.ndarray:
    h = p.c["homie_default_height"] if height is None else height
    return np.array([0.0, 0.0, 0.0, h, 0.0, 0.0, 0.0])


def initial_state(p: Params, first_height: Optional[float] = None) -> CoreState:
    """回の始め（シミュレータのリセット直後と同じ）。first_height=0.0 は評価のプロセスの最初の観測（正解の記録と比べる用）"""
    return CoreState(
        h=np.zeros((LSTM_LAYERS, LSTM_HIDDEN)),
        cell=np.zeros((LSTM_LAYERS, LSTM_HIDDEN)),
        D=np.zeros(NUM_ACT),
        a_real_prev=np.zeros(NUM_ACT),
        whole_prev=np.zeros(NUM_DOF),
        cmd_prev=default_cmd(p, first_height),
        homie_hist=np.zeros((HOMIE_HIST, HOMIE_STEP)),
        homie_a_prev=np.zeros(NUM_HOMIE),
    )


def reset_actor(p: Params, s: CoreState) -> CoreState:
    """扉の方策を始める直前に生徒の側だけを回の始めに戻す（HOMIE の履歴は続ける）。腕の目標は 0 から始まる"""
    whole = s.whole_prev.copy()
    whole[p.arm_dof] = 0.0
    return replace(
        s,
        h=np.zeros_like(s.h),
        cell=np.zeros_like(s.cell),
        D=np.zeros(NUM_ACT),
        a_real_prev=np.zeros(NUM_ACT),
        whole_prev=whole,
        cmd_prev=default_cmd(p),
    )


def homie_command_obs(p: Params, cmd: np.ndarray) -> np.ndarray:
    """b_homie_commands: [2vx, 2vy, 0.5wz, 高さ, rpy]。最初の 3 つの大きさが 0.1 未満なら 0"""
    b = cmd.astype(float).copy()
    b[0:2] *= p.c["homie_obs_lin_scale"]
    b[2] *= p.c["homie_obs_ang_scale"]
    if np.linalg.norm(b[0:3]) < p.c["homie_cmd_deadzone"]:
        b[0:3] = 0.0
    return b


def build_actor_obs(p: Params, s: CoreState, inp: Inputs) -> np.ndarray:
    """生徒の観測 151（尺度・鏡写しの後、±100 で切る。正規化の前）"""
    arm_torque = np.zeros(NUM_DOF - NUM_HOMIE) if inp.arm_torque is None else inp.arm_torque
    terms = {
        "actions": np.clip(s.whole_prev, -p.c["action_clip"], p.c["action_clip"]),
        "arm_dof_torque": arm_torque * p.c["scale_arm_dof_torque"],
        "b_homie_commands": homie_command_obs(p, s.cmd_prev),
        "base_ang_vel": inp.gyro * p.c["scale_base_ang_vel"],
        "complete": np.array([float(inp.complete)]),
        "delta_actions": s.a_real_prev,
        "dof_pos": inp.q - p.q_default,
        "dof_vel": inp.dq * p.c["scale_dof_vel"],
        "door_angle_lidar": inp.door_angle,
        "door_frame_loc": inp.door_frame_loc,
        "door_geometry": inp.door_geometry,
        "projected_gravity": inp.gravity,
    }
    x = np.zeros(p.obs_dim)
    for name, (a, b) in p.layout.items():
        v = np.asarray(terms[name], dtype=float).reshape(-1)
        if v.shape[0] != b - a:
            raise ValueError(f"観測 {name} の次元が {v.shape[0]}（期待 {b - a}）")
        x[a:b] = v
    if p.mirror:
        x = p.obs_mirror_M @ x + p.obs_mirror_b
    return np.clip(x, -p.c["clip_observations"], p.c["clip_observations"])


def actor_forward(p: Params, obs: np.ndarray, h: np.ndarray, cell: np.ndarray):
    """正規化 → LSTM 2 層（PyTorch の門の順 i, f, g, o）→ MLP。出力は平均の行動（鏡写しの側）"""
    x = np.clip((obs - p.rms_mean) / np.sqrt(p.rms_var + p.c["rms_eps"]), -p.c["rms_clip"], p.c["rms_clip"])
    h_new, c_new = [], []
    for layer, (w_ih, w_hh, b_ih, b_hh) in enumerate(p.lstm):
        g = w_ih @ x + b_ih + w_hh @ h[layer] + b_hh
        i, f, gg, o = np.split(g, 4)
        c_l = _sigmoid(f) * cell[layer] + _sigmoid(i) * np.tanh(gg)
        h_l = _sigmoid(o) * np.tanh(c_l)
        h_new.append(h_l)
        c_new.append(c_l)
        x = h_l
    return p.actor_mlp(x), np.stack(h_new), np.stack(c_new)


def homie_one_step(p: Params, s: CoreState, inp: Inputs) -> np.ndarray:
    """HOMIE の 1 ステップ 86: [指令 7, 角速度×0.5, 重力, q−既定 29, dq×0.05 29, 前の HOMIE の出力 15]"""
    return np.concatenate(
        [
            homie_command_obs(p, s.cmd_prev),
            inp.gyro * p.c["scale_base_ang_vel"],
            inp.gravity,
            inp.q - p.q_default,
            inp.dq * p.c["scale_dof_vel"],
            s.homie_a_prev,
        ]
    )


def homie_forward(p: Params, s: CoreState, one: np.ndarray, zero_stand_cmd: bool = True):
    """歩く／立つを今のステップの指令の大きさ（観測の尺度）で切り替える。

    立つ方策には全フレームの指令の欄を 0 にして渡す（学習と同じ。直し 8、10-26）。
    zero_stand_cmd=False は 10-16 の評価（立つ方策にも指令をそのまま渡していた）と比べる用。返す obs は消す前の値
    """
    obs = np.concatenate([s.homie_hist.reshape(-1), one])
    walk = bool(np.linalg.norm(one[0:3]) > p.c["homie_switch_threshold"])
    if walk:
        return p.homie_walk(obs), obs, walk
    if zero_stand_cmd:
        return p.homie_stand(tg.zero_stand_command(obs, HOMIE_HIST + 1)), obs, walk
    return p.homie_stand(obs), obs, walk


def command_from_D(p: Params, D: np.ndarray) -> np.ndarray:
    cmd = default_cmd(p)
    lim = p.c["homie_cmd_clip"]
    cmd[0:3] = np.clip(p.c["homie_command_scale"] * D[0:3], -lim, lim)
    return cmd


def guard_arm_D(p: Params, D: np.ndarray, q: np.ndarray, arm_guard: Tuple[float, float]) -> np.ndarray:
    """腕の積み上げ D[3:17] を今の角度 ±delta・可動域の端から margin 内側に収めた D を返す（直し B）"""
    delta, margin = arm_guard
    arm = p.arm_dof
    D_arm = tg.guard_arm_accumulator(D[3:], q[arm], p.q_default[arm], p.q_lower[arm], p.q_upper[arm],
                                     p.c["action_scale"], delta, margin)
    return np.concatenate([D[:3], D_arm])


def step(p: Params, s: CoreState, inp: Inputs, mode: str = "door", zero_stand_cmd: bool = True,
         arm_guard: Optional[Tuple[float, float]] = ARM_GUARD):
    """1 制御ステップ。mode: "door"＝生徒と HOMIE／"hold"＝生徒を止める（指令 0・腕は今の目標のまま）

    arm_guard: (delta, margin)。腕の D を積んだ直後に今の角度 ±delta・可動域の端から margin 内側へ収める（B）。
               None は切らない（10-16 の正解の記録と比べる用）
    返り値: (次の状態, StepOut)
    """
    obs = build_actor_obs(p, s, inp)
    if mode == "door":
        a_pol, h, cell = actor_forward(p, obs, s.h, s.cell)
        a_real = p.act_mirror @ a_pol if p.mirror else a_pol
        D = np.clip(s.D + p.c["delta_action_scale"] * a_real, -p.c["delta_action_clip"], p.c["delta_action_clip"])
        if arm_guard is not None:
            D = guard_arm_D(p, D, inp.q, arm_guard)
    elif mode == "hold":
        a_pol = np.zeros(NUM_ACT)
        a_real = np.zeros(NUM_ACT)
        h, cell = s.h, s.cell
        D = s.D.copy()
        D[0:3] = 0.0
    else:
        raise ValueError(f"mode={mode}")

    one = homie_one_step(p, s, inp)
    homie_a, homie_obs, walk = homie_forward(p, s, one, zero_stand_cmd)

    cmd = command_from_D(p, D)
    whole = np.clip(np.concatenate([homie_a, D[3:]]), -p.c["action_clip"], p.c["action_clip"])
    q_target = p.c["action_scale"] * whole + p.q_default

    new = CoreState(
        h=h,
        cell=cell,
        D=D,
        a_real_prev=a_real,
        whole_prev=whole,
        cmd_prev=cmd,
        homie_hist=np.vstack([s.homie_hist[1:], one[None, :]]),
        homie_a_prev=homie_a,
    )
    return new, StepOut(q_target, obs, a_pol, a_real, homie_obs, homie_a, walk, cmd)


def quat_wxyz_to_gravity(q: np.ndarray) -> np.ndarray:
    """IMU の四元数（w, x, y, z、機体→世界）から、機体の座標で見た重力の向き R^T·[0, 0, −1]"""
    w, x, y, z = (float(v) for v in q)
    # R の 3 行目（世界の z 軸を機体の座標で見たもの）に −1 を掛ける
    return -np.array([2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)])


def layout_table(p: Params) -> List[Tuple[str, int, int]]:
    return sorted(((k, a, b) for k, (a, b) in p.layout.items()), key=lambda t: t[1])
