"""段 1（2026-10-26）: HOLD の間に、左右の足が床から受けている鉛直の力を静力学で逆算する（numpy だけ。PC2 の Python 3.8 でも動く）。

10-23 の実機: 綱が体重の 8 割以上を受け、右足がほぼ浮いたまま（右足 約 5 N）A を押して HOMIE に渡し、6.2 秒で脱力した。
A（HOLD → STAND）の前に「両足に体重が乗っている」ことを確かめるための部品。

逆算（10-25 の foot_load.py と同じ式。MuJoCo の代わりに numpy で順運動学・ヤコビアン・自重のトルクを出す）:
  脚 6 関節の釣り合い（静止）  g[脚] = tau[脚] + J_足ᵀ W
    g    その関節より足の側の部品の自重を支えるのに要るトルク（MuJoCo の qfrc_bias の速度 0 のとき）
    tau  rt/lowstate の tau_est（SDK 0〜11）
    W    足（ankle_roll_link の原点）が床から受ける力 [Fx, Fy, Fz, Mx, My, Mz]（世界の向き。鉛直の力 Fz は点に依らない）
  綱は胴に掛かるので脚の式には入らない。両足の Fz の和が体重より小さければ、差を綱が受けている。
  姿勢は pelvis の IMU の四元数（w, x, y, z）。yaw は鉛直の力に効かない。
  モデル（部品の位置・軸・質量）は export_leg_model.py が公式の g1_29dof_rev_1_0.urdf から書き出した npz を読む。

判定（HoldLoadGate）: 1 秒の平均で、両足の合計が体重の 85% 以上・軽い方の足が 35% 以上なら通す。
その 1 秒の間に脚 12 関節のどれかが 0.2 rad/s 以上で動いたら判定しない（綱が揺れていると静止の式が狂う）。線は引数で変える。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Tuple

import numpy as np

NUM_LEG = 12  # SDK 0〜11（左 0〜5・右 6〜11）
MIN_TOTAL = 0.85  # 両足の合計 ÷ 体重（2026-10-26 ユーザー決定）
MIN_EACH = 0.35  # 軽い方の足 ÷ 体重
MAX_LEG_SPEED = 0.2  # [rad/s] 窓の間ずっとこれ未満のときだけ判定する
MAX_TOTAL = 1.3  # 両足の合計 ÷ 体重がこれを越えたら、値が壊れている疑いとして判定しない（10-26 の見直し。綱は引き上げるだけ）
WINDOW_S = 1.0
WINDOW_SLACK_S = 0.05  # 窓がそろったとみなす余裕（50 Hz で 2.5 周期。周期の揺れで 0.98 s になっても判定する）

PASS = "pass"  # 通す
REJECT = "reject"  # 拒む
SKIP = "skip"  # 判定しない（動いている・まだ 1 秒たまっていない・力が計算できない）
SIDE_NAME = ("左", "右")


@dataclass(frozen=True)
class LegModel:
    """脚 2 本 × 6 関節の運動学と質量（配列の先頭の次元は 0 左・1 右、次は股 pitch から足首 roll の順）"""

    joint_names: Tuple[str, ...]  # [12] SDK の順
    body_pos: np.ndarray  # [2, 6, 3] 親の剛体の座標での位置（股 pitch の親は pelvis）
    body_rot: np.ndarray  # [2, 6, 3, 3] 親の剛体の座標での向き（関節の角度 0 のとき）
    jnt_pos: np.ndarray  # [2, 6, 3] 剛体の座標
    jnt_axis: np.ndarray  # [2, 6, 3] 剛体の座標（長さ 1）
    jnt_skew: np.ndarray  # [2, 6, 3, 3] 軸の歪対称行列 K（関節の回転 = I + sinθ·K + (1 − cosθ)·K²）
    jnt_skew2: np.ndarray  # [2, 6, 3, 3] K²
    mass: np.ndarray  # [2, 6]
    com: np.ndarray  # [2, 6, 3] 剛体の座標での重心
    total_mass: float  # [kg] 全身
    gravity: float  # [m/s²]
    meta: dict  # 出典と作り方（export_leg_model.py の make_meta）

    @property
    def weight(self) -> float:
        """体重 [N]（モデルの値。段 5 で実機の値に直すときは HoldLoadGate の weight に渡す）"""
        return self.total_mass * self.gravity


@dataclass(frozen=True)
class FootForces:
    """足（ankle_roll_link の原点）が床から受ける 6 次元の力 [Fx, Fy, Fz, Mx, My, Mz]（N・N·m、世界の向き）"""

    left: np.ndarray
    right: np.ndarray

    @property
    def fz_left(self) -> float:
        return float(self.left[2])

    @property
    def fz_right(self) -> float:
        return float(self.right[2])


def quat_wxyz_to_rot(qt: np.ndarray) -> np.ndarray:
    """四元数（w, x, y, z。長さ 1 でなくてもよい）→ 回転行列 [..., 3, 3]"""
    qt = np.asarray(qt, dtype=float)
    qt = qt / np.linalg.norm(qt, axis=-1, keepdims=True)
    w, x, y, z = qt[..., 0], qt[..., 1], qt[..., 2], qt[..., 3]
    entries = (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
               2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
               2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return np.stack(entries, axis=-1).reshape(qt.shape[:-1] + (3, 3))


def skew(v: np.ndarray) -> np.ndarray:
    """[..., 3] → 歪対称行列 [..., 3, 3]（skew(a) @ b = a × b）"""
    v = np.asarray(v, dtype=float)
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    zero = np.zeros_like(x)
    return np.stack((zero, -z, y, z, zero, -x, -y, x, zero), axis=-1).reshape(v.shape[:-1] + (3, 3))


def load_leg_model(path) -> LegModel:
    """export_leg_model.py の npz を読む（pickle は使わない）"""
    with np.load(Path(path), allow_pickle=False) as z:
        def legs(name: str) -> np.ndarray:
            a = np.asarray(z[name], dtype=float)
            return a.reshape((2, 6) + a.shape[1:])

        axis = legs("jnt_axis")
        axis = axis / np.linalg.norm(axis, axis=-1, keepdims=True)
        k = skew(axis)
        model = LegModel(
            joint_names=tuple(str(s) for s in z["joint_names"]),
            body_pos=legs("body_pos"),
            body_rot=quat_wxyz_to_rot(legs("body_quat")),
            jnt_pos=legs("jnt_pos"),
            jnt_axis=axis,
            jnt_skew=k,
            jnt_skew2=k @ k,
            mass=legs("body_mass"),
            com=legs("body_ipos"),
            total_mass=float(z["total_mass"]),
            gravity=float(-np.asarray(z["gravity"], dtype=float)[2]),
            meta=json.loads(str(z["meta"])),
        )
    if len(model.joint_names) != NUM_LEG or model.total_mass <= 0.0 or model.gravity <= 0.0:
        raise ValueError(f"脚のモデルが壊れている: {path}")
    return model


def leg_statics(model: LegModel, q: np.ndarray, quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """静止の釣り合い g[脚] = tau[脚] + J_足ᵀ W の g（自重のトルク [2, 6]）と J_足ᵀ（[2, 6, 6]）"""
    qa = np.asarray(q, dtype=float)[:NUM_LEG].reshape(2, 6)
    s, c = np.sin(qa)[..., None, None], np.cos(qa)[..., None, None]
    jrot = np.eye(3) + s * model.jnt_skew + (1.0 - c) * model.jnt_skew2  # [2, 6, 3, 3] 関節の回転（剛体の座標）
    rot = np.broadcast_to(quat_wxyz_to_rot(quat), (2, 3, 3))  # pelvis の向き（位置は原点に置く。力には効かない）
    pos = np.zeros((2, 3, 1))
    anchor = np.empty((2, 6, 3))  # 関節の軸の点（世界）
    axis = np.empty((2, 6, 3))  # 関節の軸（世界）
    com = np.empty((2, 6, 3))  # 剛体の重心（世界）
    for k in range(6):
        # 関節の角度 0 の向き → 関節のまわりに回す（MuJoCo の回転関節と同じ: 剛体の座標で jnt_pos を中心に回る）
        rot_pre = rot @ model.body_rot[:, k]
        a = pos + rot @ model.body_pos[:, k, :, None] + rot_pre @ model.jnt_pos[:, k, :, None]
        rot = rot_pre @ jrot[:, k]
        pos = a - rot @ model.jnt_pos[:, k, :, None]
        anchor[:, k] = a[..., 0]
        axis[:, k] = (rot_pre @ model.jnt_axis[:, k, :, None])[..., 0]
        com[:, k] = (pos + rot @ model.com[:, k, :, None])[..., 0]
    foot = pos[..., 0]  # [2, 3] ankle_roll_link の原点
    # 自重のトルク: 関節 k より足の側（k..5）の質量 M_k と、その重心の重み付きの和 S_k
    mcom = model.mass[..., None] * com
    m_dist = np.cumsum(model.mass[:, ::-1], axis=1)[:, ::-1]  # [2, 6]
    s_dist = np.cumsum(mcom[:, ::-1], axis=1)[:, ::-1]  # [2, 6, 3]
    lever = s_dist - m_dist[..., None] * anchor
    g_tau = model.gravity * np.cross(axis, lever)[..., 2]  # [2, 6]
    # J_足ᵀ の行 k: [並進 axis_k × (足 − 軸の点), 回転 axis_k]
    jac_t = np.concatenate([np.cross(axis, foot[:, None, :] - anchor), axis], axis=2)  # [2, 6, 6]
    return g_tau, jac_t


def foot_forces(model: LegModel, q: np.ndarray, tau: np.ndarray, quat: np.ndarray) -> FootForces:
    """関節の角度 q・トルク tau（どちらも SDK の順。先頭 12 を使う）と pelvis の IMU の四元数（w, x, y, z）から、
    左右の足が床から受けている 6 次元の力を出す（静止の近似。速度・加速度の項は入れない）"""
    g_tau, jac_t = leg_statics(model, q, quat)
    ta = np.asarray(tau, dtype=float)[:NUM_LEG].reshape(2, 6)
    wrench = np.linalg.solve(jac_t, (g_tau - ta)[..., None])[..., 0]  # b を [..., 6, 1] で渡す（numpy 1・2 で同じ意味）
    return FootForces(left=wrench[0], right=wrench[1])


def leg_torques_for_wrenches(model: LegModel, q: np.ndarray, quat: np.ndarray,
                             w_left: np.ndarray, w_right: np.ndarray) -> np.ndarray:
    """foot_forces の逆: 足が受ける力（世界の向き、ankle_roll_link の原点）から、静止で釣り合う脚 12 関節のトルク（SDK の順）。
    実機では使わない（試験の作り物の機体と、シムの吊り具の確かめ用）"""
    g_tau, jac_t = leg_statics(model, q, quat)
    w = np.stack([np.asarray(w_left, dtype=float), np.asarray(w_right, dtype=float)])
    return (g_tau - (jac_t @ w[..., None])[..., 0]).reshape(NUM_LEG)


@dataclass(frozen=True)
class LoadSample:
    t: float  # [s]
    fz_left: float  # [N]
    fz_right: float
    leg_speed: float  # [rad/s] 脚 12 関節の速さの最大
    fast_joint: int  # その関節（SDK の番号）


@dataclass(frozen=True)
class GateVerdict:
    decision: str  # PASS・REJECT・SKIP
    reason: str  # 人が読む理由の文
    fz_left: float = math.nan  # [N] 窓の平均
    fz_right: float = math.nan
    total_ratio: float = math.nan  # 両足の合計 ÷ 体重
    light_ratio: float = math.nan  # 軽い方の足 ÷ 体重

    @property
    def passed(self) -> bool:
        return self.decision == PASS


def make_sample(t: float, forces: FootForces, dq: np.ndarray) -> LoadSample:
    legs = np.abs(np.asarray(dq, dtype=float)[:NUM_LEG])
    bad = ~np.isfinite(legs)
    if bad.any():  # 読めない速さは「速い」として扱う（判定しない）
        j = int(np.argmax(bad))
        return LoadSample(t=float(t), fz_left=forces.fz_left, fz_right=forces.fz_right, leg_speed=math.inf, fast_joint=j)
    j = int(np.argmax(legs))
    return LoadSample(t=float(t), fz_left=forces.fz_left, fz_right=forces.fz_right, leg_speed=float(legs[j]), fast_joint=j)


def format_load(fz_left: float, fz_right: float, weight: float) -> str:
    """画面に出す形「足 L ○ N・R ○ N（体重の ○%）」"""
    return f"足 L {fz_left:.0f} N・R {fz_right:.0f} N（体重の {100 * (fz_left + fz_right) / weight:.0f}%）"


def judge_window(samples: Tuple[LoadSample, ...], weight: float, min_total: float = MIN_TOTAL,
                 min_each: float = MIN_EACH, max_leg_speed: float = MAX_LEG_SPEED,
                 window_s: float = WINDOW_S) -> GateVerdict:
    """窓の中の周期の並び（古い順）→ 通す・拒む・判定しない と理由"""
    span = samples[-1].t - samples[0].t if samples else 0.0
    if span < window_s - WINDOW_SLACK_S:
        return GateVerdict(SKIP, f"測り始めて {span:.1f} 秒（{window_s:.1f} 秒そろうまで判定しない）")
    fast = max(samples, key=lambda s: s.leg_speed)
    if math.isinf(fast.leg_speed):
        return GateVerdict(SKIP, f"脚の関節 {fast.fast_joint} の速さが読めない（NaN）")
    if not fast.leg_speed < max_leg_speed:
        return GateVerdict(SKIP, f"脚の関節 {fast.fast_joint} が {fast.leg_speed:.2f} rad/s で動いている"
                                 f"（{max_leg_speed:.2f} rad/s 未満が {window_s:.1f} 秒続くまで判定しない）")
    fz = np.array([(s.fz_left, s.fz_right) for s in samples])
    if not np.isfinite(fz).all():
        return GateVerdict(SKIP, "足の力が計算できない（NaN）。関節の角度・トルク・IMU を確かめる")
    left, right = (float(v) for v in fz.mean(axis=0))
    total = (left + right) / weight
    if not total <= MAX_TOTAL:
        return GateVerdict(SKIP, f"逆算した両足の合計 {left + right:.0f} N が体重の {100 * total:.0f}%（{100 * MAX_TOTAL:.0f}% を越える。"
                                 "トルク・IMU の値か体重の分母を確かめる）", fz_left=left, fz_right=right, total_ratio=total)
    light_side = 0 if left < right else 1
    light_n = min(left, right)
    light = light_n / weight
    numbers = dict(fz_left=left, fz_right=right, total_ratio=total, light_ratio=light)
    short = []
    if not total >= min_total:
        short.append(f"両足の合計 {left + right:.0f} N（体重の {100 * total:.0f}%）が線 {100 * min_total:.0f}% に届かない")
    if not light >= min_each:
        short.append(f"{SIDE_NAME[light_side]}足 {light_n:.0f} N（体重の {100 * light:.0f}%）が線 {100 * min_each:.0f}% に届かない")
    if short:
        return GateVerdict(REJECT, "・".join(short), **numbers)
    return GateVerdict(PASS, f"{format_load(left, right, weight)}・軽い方 {100 * light:.0f}%"
                             f"（線 合計 {100 * min_total:.0f}%・片足 {100 * min_each:.0f}%）", **numbers)


@dataclass(frozen=True)
class HoldLoadGate:
    """HOLD の間に毎周期 push して、A を押されたら verdict を見る。push は新しい判定器を返す（自分は変えない）"""

    weight: float  # [N] 体重（モデルの値か、段 5 で実機から推定した値）
    min_total: float = MIN_TOTAL
    min_each: float = MIN_EACH
    max_leg_speed: float = MAX_LEG_SPEED
    window_s: float = WINDOW_S
    samples: Tuple[LoadSample, ...] = ()

    def __post_init__(self) -> None:
        for name in ("weight", "window_s", "max_leg_speed"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0.0):
                raise ValueError(f"{name} は正の数（{v}）")
        for name in ("min_total", "min_each"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v >= 0.0):
                raise ValueError(f"{name} は 0 以上（{v}）")

    def push(self, t: float, forces: FootForces, dq: np.ndarray) -> "HoldLoadGate":
        """この周期の時刻 [s]・足の力・関節の速さ（SDK の順。先頭 12 を見る）を足し、窓より古い周期を捨てた判定器"""
        s = make_sample(t, forces, dq)
        kept = tuple(x for x in self.samples if x.t >= s.t - self.window_s - 1e-9) + (s,)
        return replace(self, samples=kept)

    def verdict(self) -> GateVerdict:
        return judge_window(self.samples, self.weight, self.min_total, self.min_each, self.max_leg_speed, self.window_s)
