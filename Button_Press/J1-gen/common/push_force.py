"""押している間に、指先がボタンをどれだけの力で押したかを、関節の推定トルク（lowstate の tau_est）から推定する。

G1 の腕には力のセンサーが無い。モーターの電流から推定したトルク（tau_est）は、押す前（手前で止まっているとき）は
重力などを支えるトルクで、指先が何かを押すと、その反力を支える分だけ変わる。腕が動くと重力のトルクも変わるので
（MuJoCo で、触れる前の 6 cm の押し込みだけで 3.7 N に見えた。2026-11-04）、重力のモデル（common/arm/gravity.py）で
計算した重力のトルクを引いた残りを使う。止まっていれば
    残りの変化 Δτ (7) = Jᵀ f
（J は指先の位置のヤコビアン (3, 7)、f は指先がボタンを押す力 (3)、pelvis 座標）。7 つの式から 3 つの未知数を
最小二乗で解く。押す向きの成分（f · 押す向き）を「押す力」、残りを「横の力」とする。

使い方（common/elevator_press.py）:
    mon = PushForceMonitor(kin)
    mon = PushForceMonitor(kin, gravity)   gravity は GravityModel（None なら重力の分を引かない）
    毎周期 mon.remember(q, tau, imu)          押す前の値を、直近 baseline_window 周期だけ持つ
    押し込みを始めた周期 mon.start(n)         持っていた値の平均を基準にする
    押し込み・保持の周期 mon.measure(phase, q, tau, imu)
    保持が終わったら mon.finish()      まとめ（PushForceSummary）を返す

⚠️ 推定の限界:
- 位置の PD 制御なので、トルクの変化は「指令と実際の角度の差 × kp」とほぼ同じ情報（関節の遅れとは独立でない）。
  役に立つのは、押した力が N の値で残ることと、押す向き・横の向きに分けられること
- 腕が動いている間は、加速度・摩擦の分も入る。重力のモデルの誤差（腕の重さ・重心）も残る
- 実機の tau_est には摩擦・ノイズが乗る。大きさは未確認（実機の日に、止まっているときのばらつきを測る。
  real/press_force.py の「見る」段階の値）
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass

import numpy as np

from .arm.gravity import GravityModel
from .kinematics import ArmKinematics


def push_force(kin: ArmKinematics, q: np.ndarray, dtau_arm: np.ndarray) -> np.ndarray:
    """腕の 7 関節のトルクの変化（押す前からの増え方 [Nm]）から、指先が押している力 (3,) [N]（pelvis 座標）。"""
    J = kin.jacobian_pos(q)
    f, *_ = np.linalg.lstsq(J.T, np.asarray(dtau_arm, dtype=float), rcond=None)
    return f


def split_force(f: np.ndarray, push_dir: np.ndarray) -> tuple[float, float]:
    """力を、押す向きの成分 [N]（正がボタンを押す向き）と、横の大きさ [N] に分ける。"""
    n = np.asarray(push_dir, dtype=float)
    along = float(f @ n)
    return along, float(np.linalg.norm(f - along * n))


@dataclass
class PushForceSummary:
    """1 回の押し込み（押し直しは別の回）の、推定した押す力のまとめ [N]。"""

    attempt: int
    press_max_n: float        # 押し込み中の、押す向きの力の最大
    press_side_n: float       # そのときの横の力
    hold_mean_n: float | None  # 保持中の、押す向きの力の平均（保持の周期が無ければ None）
    hold_max_n: float | None
    baseline_samples: int     # 基準にした、押す前の周期の数

    def text(self) -> str:
        hold = (f"、保持中の平均 {self.hold_mean_n:.1f} N（最大 {self.hold_max_n:.1f} N）"
                if self.hold_mean_n is not None and self.hold_max_n is not None else "")
        return (f"押す力（推定、tau_est から）: 押し込み中の最大 {self.press_max_n:.1f} N（そのとき横に "
                f"{self.press_side_n:.1f} N）{hold}")

    def to_dict(self) -> dict[str, float | int | None]:
        return asdict(self)


class PushForceMonitor:
    def __init__(self, kin: ArmKinematics, gravity: GravityModel | None = None, baseline_window: int = 10) -> None:
        self.kin = kin
        self.gravity = gravity
        self.arm = kin.arm_idx
        self._recent: deque[np.ndarray] = deque(maxlen=int(baseline_window))
        self.attempt = 0
        self.last_n: float | None = None  # 最後に測った押す向きの力（記録の列に使う）
        self._n_base = 0
        self._reset_attempt()

    def _reset_attempt(self) -> None:
        self.baseline: np.ndarray | None = None
        self.push_dir: np.ndarray | None = None
        self._press: list[tuple[float, float]] = []  # (押す向き, 横)
        self._hold: list[float] = []

    def _residual(self, q: np.ndarray, tau: np.ndarray, imu_quat: np.ndarray | None) -> np.ndarray:
        """腕の 7 関節の推定トルクから、重力のトルク（モデルで計算）を引いた残り。"""
        r = np.asarray(tau, dtype=float)[self.arm].copy()
        if self.gravity is not None:
            r -= self.gravity.torques(q, imu_quat)[self.arm]
        return r

    def remember(self, q: np.ndarray, tau: np.ndarray | None, imu_quat: np.ndarray | None = None) -> None:
        """押す前の値として持つ（毎周期呼ぶ。基準は start() の直前の baseline_window 周期の平均）。"""
        if tau is not None:
            self._recent.append(self._residual(q, tau, imu_quat))

    def start(self, push_dir: np.ndarray) -> None:
        """押し込みを始める。それまで持っていた値の平均を基準にする（値が無ければ測らない）。"""
        self._reset_attempt()
        self.attempt += 1
        self.push_dir = np.asarray(push_dir, dtype=float)
        if self._recent:
            self.baseline = np.mean(np.array(self._recent), axis=0)
        self._n_base = len(self._recent)

    def measure(self, phase: str, q: np.ndarray, tau: np.ndarray | None,
                imu_quat: np.ndarray | None = None) -> float | None:
        """押し込み（phase="press"）・保持（"hold"）の周期の、押す向きの力 [N]。測れなければ None。"""
        if tau is None or self.baseline is None or self.push_dir is None:
            self.last_n = None
            return None
        dtau = self._residual(q, tau, imu_quat) - self.baseline
        along, side = split_force(push_force(self.kin, q, dtau), self.push_dir)
        if phase == "press":
            self._press.append((along, side))
        else:
            self._hold.append(along)
        self.last_n = along
        return along

    def finish(self) -> PushForceSummary | None:
        """その回のまとめ。測れなかった（tau_est が無い）なら None。"""
        if not self._press:
            self._reset_attempt()
            return None
        k = int(np.argmax([a for a, _ in self._press]))
        hold = np.array(self._hold) if self._hold else None
        out = PushForceSummary(
            attempt=self.attempt, press_max_n=self._press[k][0], press_side_n=self._press[k][1],
            hold_mean_n=None if hold is None else float(hold.mean()),
            hold_max_n=None if hold is None else float(hold.max()), baseline_samples=self._n_base)
        self._reset_attempt()
        return out


def replay(kin: ArmKinematics, gravity: GravityModel | None, phases: list[str], q: np.ndarray, tau: np.ndarray,
           imu: np.ndarray, push_dir: np.ndarray,
           baseline_window: int = 10) -> tuple[np.ndarray, list[PushForceSummary]]:
    """記録（エージェントの body_log.csv）の行ごとの段階・関節角 (N, 29)・推定トルク (N, 29)・IMU (N, 4) から、
    エージェントの中と同じ手順で押す力を出し直す。行ごとの押す向きの力（押し込み・保持の行だけ。ほかは nan）と、
    回ごとのまとめを返す。
    body_log の段階はその周期の step のあとの段階なので、押し込みに移った周期の分だけ、エージェントの中より 1 周期早く
    測り始める（値は少し違う。MuJoCo の種 0 で、押し込み中の最大がエージェントの中 10.1 N、出し直し 9.8 N）。"""
    mon = PushForceMonitor(kin, gravity, baseline_window)
    force = np.full(len(phases), np.nan)
    out: list[PushForceSummary] = []
    prev = ""
    for i, ph in enumerate(phases):
        if ph == "press" and prev != "press":
            if prev == "hold":  # 保持から直接押し込みに戻ることは無いが、念のため前の回を閉じる
                s = mon.finish()
                out += [s] if s is not None else []
            mon.start(push_dir)
        if prev == "hold" and ph != "hold":
            s = mon.finish()
            out += [s] if s is not None else []
        if ph in ("press", "hold"):
            v = mon.measure(ph, q[i], tau[i], imu[i])
            force[i] = np.nan if v is None else v
        mon.remember(q[i], tau[i], imu[i])
        prev = ph
    if prev == "hold":
        s = mon.finish()
        out += [s] if s is not None else []
    return force, out


def still_noise(tau_arm: np.ndarray) -> np.ndarray:
    """腕が止まっている間（エレベーターでは look の段階）の推定トルクのばらつき（関節ごとの標準偏差 [Nm]）。"""
    return np.asarray(tau_arm, dtype=float).std(axis=0)
