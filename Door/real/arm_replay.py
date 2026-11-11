"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-decision.md の R1・R4、道具 T1・T2）: 実機の走らせ役（door_run_stock.py）が使う
numpy だけの部品。

R1 腕の動きの再生: 記録（door_stock_*.npz）の段 2 の腕の送り値 arm_sent を、同じ時刻（倍率つき）で流す。扉から離れた何も無い所で
  流して、体が回るか（反動が主因か）を見る。記録の時刻の間隔は 20〜40 ms と揺れるので、行の番号でなく時刻で補間する。
  走らせ役の腕の積み上げの守り（今の角度 ±δ・door_core.ARM_GUARD）は再生を通らないので、ここで今の角度 ±δ に切り直す。
R4 腰 yaw の打ち消し: 打ち消しを始めた時からの pelvis の回り Δ（IMU の yaw）の逆を、腰 yaw の目標（arm_sdk で保つ値）に足す。
  −倍率·Δ を ±上限で切り、変化の速さにも上限を付ける（腰の kp 250 で急に回さない）。止めると 0 へ戻る。
  方策に渡す扉の値は、打ち消した胴の向き（pelvis の yaw＋打ち消し）から見た値にし、観測の腰 yaw から打ち消しの分を引く
  （シムのパッチ 0105 の door_obs=torso・hide_obs と同じ。扉の値を pelvis のまま渡すとシムで効き目が消えた）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from door_frame import PlanarPose

SEGMENTS = ("policy", "stage2")
WAIST_YAW = 12  # 関節の並び（SDK 0〜28）の腰 yaw


@dataclass(frozen=True)
class ReplayTrack:
    t: np.ndarray  # [T] 区間の始めからの時刻 [s]（倍率を掛けた後）
    arm: np.ndarray  # [T, 14] 腕の送り値 [rad]
    source: str
    segment: str
    speed: float

    @property
    def duration(self) -> float:
        return float(self.t[-1])


def load_replay(path: "str | Path", segment: str = "policy", speed: float = 1.0) -> ReplayTrack:
    """記録から再生する区間を取り出す。segment: policy＝段 2 の方策の区間・stage2＝腕の構え（2.5 s）を含む段 2 の全部。
    speed: 1.0 で記録の速さ・0.5 で半分の速さ（時間は 2 倍）"""
    if segment not in SEGMENTS:
        raise ValueError(f"segment は {SEGMENTS} のどれか: {segment}")
    if not (speed > 0.0):
        raise ValueError(f"speed は正: {speed}")
    z = np.load(path, allow_pickle=True)
    st, t, arm = z["staged"], z["t"], z["arm_sent"]
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2)
    pick = in2 & (st[:, 2] == 2) if segment == "policy" else in2 & np.isfinite(st[:, 2])
    pick &= np.isfinite(arm).all(axis=1)
    k = np.flatnonzero(pick)
    if len(k) < 2:
        raise ValueError(f"{path}: 再生できる行が {len(k)} 行しかない（segment={segment}）")
    tt = (t[k] - t[k[0]]) / speed
    return ReplayTrack(t=tt, arm=arm[k].astype(float), source=str(path), segment=segment, speed=float(speed))


def track_at(track: ReplayTrack, t: float) -> np.ndarray:
    """時刻 t [s] の腕の送り値（区間の外は端の値）"""
    return np.array([np.interp(t, track.t, track.arm[:, j]) for j in range(track.arm.shape[1])])


def guard_lead(target: np.ndarray, q_arm: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """送る腕の目標を、今の角度 ±delta に収めた新しい配列（走らせ役の積み上げの守りと同じ幅）"""
    return np.clip(target, q_arm - delta, q_arm + delta)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class WaistCompConfig:
    gain: float  # 打ち消しの倍率（0 で切）
    max_rad: float  # 打ち消しの上限 [rad]
    rate: float  # 打ち消しの変化の速さの上限 [rad/s]


@dataclass(frozen=True)
class WaistCompState:
    yaw0: Optional[float] = None  # 打ち消しを始めた時の IMU の yaw（None は始めていない）
    off: float = 0.0  # 今の打ち消し [rad]（腰 yaw の目標に足す）


def waist_comp_step(st: WaistCompState, yaw: float, active: bool, dt: float,
                    cfg: WaistCompConfig) -> Tuple[WaistCompState, float, float]:
    """1 周期進める。(新しい状態, 打ち消し [rad], 打ち消しの変化の速さ [rad/s]) を返す。
    active の最初の周期で基準の yaw を取り、以後は −倍率·Δ（±上限）へ速さの上限で寄せる。active でなければ 0 へ寄せて基準を捨てる"""
    if not (math.isfinite(cfg.gain) and math.isfinite(cfg.max_rad) and math.isfinite(cfg.rate)):
        raise ValueError(f"腰 yaw の打ち消しの設定は有限の値: {cfg}")
    if cfg.gain <= 0.0:
        return WaistCompState(), 0.0, 0.0
    if not math.isfinite(yaw):  # IMU の値が壊れた周期は打ち消しを止めて 0 へ戻す（腰へ NaN を送らない・ねじれたまま固めない）
        active = False
    if active:
        yaw0 = yaw if st.yaw0 is None else st.yaw0
        goal = max(-cfg.max_rad, min(cfg.max_rad, -cfg.gain * wrap_angle(yaw - yaw0)))
    else:
        yaw0, goal = None, 0.0
    step = cfg.rate * dt
    off = st.off + max(-step, min(step, goal - st.off))
    return replace(st, yaw0=yaw0, off=off), off, (off - st.off) / dt


def torso_pose(pose: PlanarPose, off: float) -> PlanarPose:
    """打ち消した胴の向きの姿勢（扉の座標での yaw に打ち消しを足す）。方策の扉の値（door_frame_loc）をこれで作る"""
    return PlanarPose(pose.x, pose.y, pose.yaw + off)


def waist_with_yaw(waist_hold: np.ndarray, off: float, yaw_lower: float, yaw_upper: float) -> np.ndarray:
    """送る腰 3 関節の目標: 借りた時の角度の yaw にだけ打ち消しを足して可動域で切る（roll・pitch は借りた時のまま）。新しい配列"""
    out = np.array(waist_hold, dtype=float)
    out[0] = min(max(out[0] + off, yaw_lower), yaw_upper)
    return out


def hide_waist(q: np.ndarray, dq: np.ndarray, off: float, off_rate: float) -> Tuple[np.ndarray, np.ndarray]:
    """方策に渡す関節の角度・速さから、腰 yaw の打ち消しの分を引いた写し（段の部品・順運動学には元の q を渡す）"""
    q2, dq2 = q.copy(), dq.copy()
    q2[WAIST_YAW] = q[WAIST_YAW] - off
    dq2[WAIST_YAW] = dq[WAIST_YAW] - off_rate
    return q2, dq2
