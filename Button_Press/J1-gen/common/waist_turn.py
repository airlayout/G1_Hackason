"""腰 yaw を小さい角度からゆっくり回して確かめるための、動きの計画・止める判定・記録のまとめ。実機に依存しない部品。

使い先は real/waist_turn_check.py。ここには DDS も ArmCommander も出てこない（単体テストできる）。

なぜ要るか: 「見つからないとき腰 yaw を ±10〜20° 回して探す」機能（press.waist_scan）は、MuJoCo（骨盤を固定）でしか
動かしていない。実機では、ひねりの反動と、足側の歩行制御（立っている間も動いている）との干渉が見える。
腕は動かさず、腰 yaw だけを小さい角度から回して、そこを確かめる。

動き（WaistTurnPlan）: 始めの角度（測った値）から、指定した角度（始めの角度からの差）の並びへ順に smoothstep で回し、
止めて hold_s 秒保ち、次へ。最後に始めの角度へ戻して end_hold_s 秒保つ。腕・腰 roll / pitch は始めの角度のまま。
- speed_deg_s は「平均の速さ」。smoothstep の最大は平均の 1.5 倍なので、最大の速さ（1.5 × speed_deg_s）が
  MAX_SPEED_DEG_S（20°/s）を超える設定は ValueError にする（超えるのは危ないほう側で数えた）
- 角度の上限 max_deg（既定 5°。これを超える角度の並びは、max_deg を明示して上げない限り ValueError）。
  max_deg の絶対の上限は ABS_MAX_DEG（20°）。

途中で止める判定（WaistGuard、毎周期）:
- 腰 yaw の指令と実測のずれ、骨盤の傾き（IMU の roll / pitch の始めからの変化）、腰 roll / pitch・腕の関節の始めからの
  ずれ（押されている・当たっている疑い）、lowstate の止まり
- ⚠️ しきい値の既定（3°・3°・3°・0.2 秒）は「最初に小さく試すための仮の値」。根拠になる実機の記録は無い。
  分かっているのは 2026-11-03 の実機で、立っているだけのときの腰 yaw のずれ（指令 − 実測）が最大 0.17〜0.65° だったことだけ
  （3° はその約 5 倍。傾き・ほかの関節の 3° は、当時の real/auto_press.py の MAX_WAIST_DEG = 3° に合わせた。どちらも始めからの変化）。
  実機の記録が取れたら見直すこと
- 当たったときの止め方は、既存の ArmCommander と同じ考え方に合わせる: 腕の位置が分かるなら、その場で止めて
  「始めの角度へゆっくり戻って」から weight を下げる（weight を下げると腰は内蔵コントローラの姿勢へ戻るので、
  ひねったまま下げると急に動くため）。lowstate が途切れて位置が分からないとき（stale）、値が壊れているときだけは
  戻らず、その場で保持して weight を下げる（ArmCommander も StateTimeoutError では戻らない）。
  戻る途中は判定をゆるめて（2 倍）続け、それも超えたら戻るのをやめて、その場で weight を下げる
  （ArmCommander の RetreatGuardError と同じ）
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .robot_model import WAIST_IDX

WAIST_YAW_IDX = WAIST_IDX[0]  # motor 12
ABS_MAX_DEG = 20.0  # max_deg の絶対の上限（これを超える max_deg は受け付けない）
DEFAULT_MAX_DEG = 5.0
MAX_SPEED_DEG_S = 20.0  # 指令の最大の速さの上限（平均の速さ × 1.5 で数える）
ABORT_SPEED_DEG_S = 5.0  # 止めて戻すときの平均の速さ（行きより遅い）
ABORT_MIN_S = 0.5

# 腰 roll / pitch と両腕（motor 13〜28）。腰 yaw 以外はすべて始めの角度のまま保つ
OTHER_IDX = tuple(i for i in range(13, 29))


def smoothstep(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def roll_pitch_deg(quat_wxyz: np.ndarray) -> tuple[float, float]:
    """IMU の四元数 (w, x, y, z) → 骨盤の roll・pitch [deg]。"""
    w, x, y, z = (float(v) for v in quat_wxyz)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(np.degrees(roll)), float(np.degrees(pitch))


def _wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def tilt_change_deg(quat_now: np.ndarray, quat_base: np.ndarray) -> float:
    """骨盤の傾き（roll・pitch）の、始めの値からの変化の大きいほう [deg]。"""
    r0, p0 = roll_pitch_deg(quat_base)
    r1, p1 = roll_pitch_deg(quat_now)
    return max(abs(_wrap_deg(r1 - r0)), abs(_wrap_deg(p1 - p0)))


# ---- 動きの計画 -------------------------------------------------------------------


@dataclass(frozen=True)
class Segment:
    label: str  # 記録の phase に書く名前
    kind: str  # pre / move / hold / return / end
    index: int  # 何番目の角度か（pre・return・end は -1）
    t0: float
    t1: float
    a0: float  # 始めの角度からの差 [rad]
    a1: float


def validate_limits(angles_deg: list[float], max_deg: float, speed_deg_s: float, hold_s: float) -> None:
    """設定の確認。通らなければ ValueError。"""
    if not (0.0 < max_deg <= ABS_MAX_DEG):
        raise ValueError(f"max_deg は 0 より大きく {ABS_MAX_DEG:.0f}° 以下: {max_deg}（絶対の上限 {ABS_MAX_DEG:.0f}°）")
    if not angles_deg:
        raise ValueError("角度の並びが空")
    for a in angles_deg:
        if not np.isfinite(a):
            raise ValueError(f"角度が有限でない: {a}")
        if abs(a) > max_deg + 1e-9:
            raise ValueError(f"角度 {a}° が max_deg {max_deg}° を超える（上げるなら --max-deg を明示する。絶対の上限 {ABS_MAX_DEG:.0f}°）")
    if not (speed_deg_s > 0.0) or 1.5 * speed_deg_s > MAX_SPEED_DEG_S + 1e-9:
        raise ValueError(f"speed_deg_s は 0 より大きく、最大の速さ（1.5 倍）が {MAX_SPEED_DEG_S:.0f}°/s 以下"
                         f"（つまり {MAX_SPEED_DEG_S / 1.5:.1f} 以下）: {speed_deg_s}")
    if hold_s < 0.0:
        raise ValueError(f"hold_s は 0 以上: {hold_s}")


class WaistTurnPlan:
    """時刻 t [s] → 腰 yaw の指令 [rad]（絶対の角度）。角度は始めの角度からの差 [deg]。"""

    def __init__(self, start_yaw: float, angles_deg: list[float], max_deg: float = DEFAULT_MAX_DEG,
                 speed_deg_s: float = 10.0, hold_s: float = 3.0, pre_hold_s: float = 1.0,
                 end_hold_s: float = 1.0) -> None:
        validate_limits(list(angles_deg), max_deg, speed_deg_s, hold_s)
        if not np.isfinite(start_yaw):
            raise ValueError("始めの腰 yaw が有限でない")
        self.start_yaw = float(start_yaw)
        self.angles_deg = [float(a) for a in angles_deg]
        self.max_deg = float(max_deg)
        self.speed_deg_s = float(speed_deg_s)
        self.hold_s = float(hold_s)
        self.segments: list[Segment] = []
        t = 0.0
        prev = 0.0

        def add(label: str, kind: str, idx: int, dur: float, a0: float, a1: float) -> None:
            nonlocal t
            if dur > 0.0:
                self.segments.append(Segment(label, kind, idx, t, t + dur, a0, a1))
                t += dur

        add("pre", "pre", -1, pre_hold_s, 0.0, 0.0)
        for k, deg in enumerate(self.angles_deg):
            a = float(np.radians(deg))
            dist = abs(a - prev)
            if dist > 1e-9:
                add(f"move{k}", "move", k, np.degrees(dist) / self.speed_deg_s, prev, a)
            add(f"hold{k}", "hold", k, hold_s, a, a)
            prev = a
        if abs(prev) > 1e-9:
            add("return", "return", -1, np.degrees(abs(prev)) / self.speed_deg_s, prev, 0.0)
        add("end", "end", -1, end_hold_s, 0.0, 0.0)
        self.duration = t

    def segment_at(self, t: float) -> Segment:
        for s in self.segments:
            if t < s.t1:
                return s
        return self.segments[-1]

    def offset_at(self, t: float) -> float:
        """始めの角度からの差 [rad]。"""
        s = self.segment_at(t)
        if s.t1 <= s.t0:
            return s.a1
        return s.a0 + (s.a1 - s.a0) * smoothstep((t - s.t0) / (s.t1 - s.t0))

    def at(self, t: float) -> float:
        """腰 yaw の指令 [rad]（絶対）。"""
        return self.start_yaw + self.offset_at(t)

    def command29(self, q_start: np.ndarray, t: float) -> np.ndarray:
        """29 関節の指令。腰 yaw 以外（腰 roll / pitch、腕、脚）は始めに測った角度のまま。"""
        q = np.asarray(q_start, dtype=float).copy()
        q[WAIST_YAW_IDX] = self.at(t)
        return q

    def describe(self) -> list[str]:
        out = []
        for s in self.segments:
            a0, a1 = np.degrees(s.a0), np.degrees(s.a1)
            move = f"{a0:+.1f}° → {a1:+.1f}°" if s.kind in ("move", "return") else f"{a1:+.1f}° で保つ"
            out.append(f"{s.t0:6.1f}〜{s.t1:6.1f} s  {s.label:<7} {move}")
        return out


def smooth_return_path(yaw_from: float, yaw_to: float, dt: float,
                       speed_deg_s: float = ABORT_SPEED_DEG_S, min_s: float = ABORT_MIN_S) -> np.ndarray:
    """止めたときに、今の指令から始めの角度へ戻す指令の列（smoothstep。今の値は含めず、最後は yaw_to ちょうど）。"""
    dist = abs(yaw_to - yaw_from)
    if dist < 1e-9:
        return np.array([yaw_to])
    dur = max(min_s, float(np.degrees(dist)) / speed_deg_s)
    n = max(1, int(np.ceil(dur / dt)))
    u = np.array([smoothstep(k / n) for k in range(1, n + 1)])
    path = yaw_from + (yaw_to - yaw_from) * u
    path[-1] = yaw_to
    return path


# ---- 途中で止める判定 -------------------------------------------------------------


@dataclass
class GuardLimits:
    max_track_err_deg: float = 3.0
    max_tilt_change_deg: float = 3.0
    max_other_dev_deg: float = 3.0
    stale_s: float = 0.2


@dataclass(frozen=True)
class StopReason:
    kind: str  # track / tilt / other / stale / invalid
    message: str

    @property
    def hold_in_place(self) -> bool:
        """True なら戻らず、その場で保持して weight を下げる（腰・骨盤の位置が分からないとき）。"""
        return self.kind in ("stale", "invalid")


class WaistGuard:
    """毎周期の判定。q_base（始めに測った 29 関節）・imu_base（そのときの IMU）からの変化を見る。"""

    def __init__(self, q_base: np.ndarray, imu_base: np.ndarray, limits: GuardLimits | None = None) -> None:
        self.q_base = np.asarray(q_base, dtype=float).copy()
        self.imu_base = np.asarray(imu_base, dtype=float).copy()
        self.limits = limits or GuardLimits()

    def check(self, q: np.ndarray, imu_quat: np.ndarray, yaw_cmd: float, age_s: float,
              scale: float = 1.0) -> StopReason | None:
        """どれかに当たれば StopReason、無ければ None。scale は戻る途中のゆるめ（2.0 で 2 倍）。"""
        lim = self.limits
        q = np.asarray(q, dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(imu_quat)) or not np.isfinite(yaw_cmd):
            return StopReason("invalid", "lowstate の値が有限でない")
        if age_s > lim.stale_s:
            return StopReason("stale", f"lowstate が {age_s:.2f} 秒更新されない（しきい値 {lim.stale_s:.2f}）")
        err = float(np.degrees(abs(yaw_cmd - q[WAIST_YAW_IDX])))
        if err > lim.max_track_err_deg * scale:
            return StopReason("track", f"腰 yaw の指令と実測のずれ {err:.2f}°（しきい値 {lim.max_track_err_deg * scale:.1f}°）")
        tilt = tilt_change_deg(imu_quat, self.imu_base)
        if tilt > lim.max_tilt_change_deg * scale:
            return StopReason("tilt", f"骨盤の傾き（IMU roll/pitch）が始めから {tilt:.2f}° 変わった"
                              f"（しきい値 {lim.max_tilt_change_deg * scale:.1f}°）")
        idx = list(OTHER_IDX)
        dev = np.degrees(np.abs(q[idx] - self.q_base[idx]))
        j = int(np.argmax(dev))
        if float(dev[j]) > lim.max_other_dev_deg * scale:
            return StopReason("other", f"motor {idx[j]}（腰 roll/pitch・腕）が始めから {dev[j]:.2f}° ずれた"
                              f"（しきい値 {lim.max_other_dev_deg * scale:.1f}°。押されている・当たっている疑い）")
        return None


# ---- 記録のまとめ -----------------------------------------------------------------


@dataclass
class _Sample:
    t: float
    kind: str
    index: int
    yaw_cmd: float
    yaw: float
    quat: np.ndarray


@dataclass
class WaistTurnRecorder:
    """毎周期の (時刻, 段階, 指令, 実測, IMU) をためて、角度ごとにまとめる。"""

    samples: list[_Sample] = field(default_factory=list)

    def add(self, t: float, kind: str, index: int, yaw_cmd: float, q: np.ndarray, imu_quat: np.ndarray) -> None:
        self.samples.append(_Sample(t, kind, index, float(yaw_cmd), float(q[WAIST_YAW_IDX]),
                                    np.asarray(imu_quat, dtype=float).copy()))

    def summarize(self, start_yaw: float, angles_deg: list[float], hold_settle_s: float = 0.5) -> dict[str, Any]:
        """角度ごとの指令と実測のずれの最大・保つ間の実測の平均と揺れ・骨盤の傾きの変化の最大、戻したあとの差。

        保つ間の平均と揺れは、保ち始めの hold_settle_s 秒（保つ時間の 1/3 を超えない）を除いて数える（動いた直後の整定を除く）。
        傾きの変化の基準は、最初の段階（pre。動かす前）の IMU の平均に近い値として pre の最初の 1 点にする。
        """
        if not self.samples:
            raise ValueError("記録が空")
        pre = [s for s in self.samples if s.kind == "pre"] or self.samples[:1]
        base_quat = pre[0].quat
        rows = []
        for k, deg in enumerate(angles_deg):
            seg = [s for s in self.samples if s.index == k and s.kind in ("move", "hold")]
            hold = [s for s in self.samples if s.index == k and s.kind == "hold"]
            row: dict[str, Any] = {"index": k, "target_deg": float(deg), "samples": len(seg)}
            if not seg:
                rows.append(row)
                continue
            row["max_track_err_deg"] = float(np.degrees(max(abs(s.yaw_cmd - s.yaw) for s in seg)))
            row["max_tilt_change_deg"] = float(max(tilt_change_deg(s.quat, base_quat) for s in seg))
            if hold:
                dur = hold[-1].t - hold[0].t
                skip = min(hold_settle_s, dur / 3.0)
                use = [s for s in hold if s.t - hold[0].t >= skip] or hold
                ys = np.degrees(np.array([s.yaw - start_yaw for s in use]))
                row["hold_mean_deg"] = float(np.mean(ys))
                row["hold_std_deg"] = float(np.std(ys))
                row["hold_mean_err_deg"] = float(np.mean(ys) - deg)
            rows.append(row)
        end = [s for s in self.samples if s.kind == "end"]
        out: dict[str, Any] = {"angles": rows,
                               "max_tilt_change_deg": float(max(tilt_change_deg(s.quat, base_quat) for s in self.samples))}
        if end:
            tail = end[len(end) // 2:]  # 戻した直後を除き、後ろ半分
            out["return_diff_deg"] = float(np.degrees(np.mean([s.yaw for s in tail]) - start_yaw))
        else:
            out["return_diff_deg"] = None  # 最後まで動かせなかった（途中で止めた）
        return out


def summary_text(summary: dict[str, Any]) -> str:
    """summarize の結果の表（文字列）。"""
    lines = ["角度[deg] | 指令-実測 最大[deg] | 保つ間の実測 平均[deg] / 揺れ std[deg] / 平均-目標[deg] | 傾き変化 最大[deg]"]
    f = lambda v, p=2: "-" if v is None else f"{v:+.{p}f}"  # noqa: E731
    for r in summary["angles"]:
        if "max_track_err_deg" not in r:
            lines.append(f"{r['target_deg']:+7.1f}   | （動かしていない）")
            continue
        lines.append(f"{r['target_deg']:+7.1f}   | {r['max_track_err_deg']:7.2f}           | "
                     f"{f(r.get('hold_mean_deg'))} / {r.get('hold_std_deg', float('nan')):.2f} / {f(r.get('hold_mean_err_deg'))}"
                     f"            | {r['max_tilt_change_deg']:.2f}")
    rd = summary.get("return_diff_deg")
    lines.append(f"全体: 骨盤の傾き変化の最大 {summary['max_tilt_change_deg']:.2f}°、戻したあとの始めの角度との差 "
                 + ("（最後まで動かせず）" if rd is None else f"{rd:+.2f}°"))
    return "\n".join(lines)
