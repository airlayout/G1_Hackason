"""ボタンを押す力が適正だったかを測る（シミュレーターに依存しない部分）。

ランナー（contest/runner.py）と模擬 G1（sim/mujoco/g1_sim_server.py）が、試行ごとに 1 つ作って使う。
力は「ロボットが目標のボタンを押す力」（ボタンの面との接触力の大きさ [N]）。壁や柱との接触力は含まない。

configs/contest.yaml の press_force:
- ok_range_n   適正な範囲 [最小, 最大]。押したときの力の最大がこの中なら「適正」
- max_n        これを超えたら「押しすぎ」（実機のボタンと手を傷める恐れ）
- post_press_s 点灯したあとも続けて測る時間（押す力の最大と、指を離したかを見るため。成否の判定は変えない）
- release_n    点灯のあとの時間の終わりに、力がこれより小さければ「指を離した」

⚠️ 値はエレベーターの規格（EN 81-70: 押す力 2.5〜5 N）からの目安。本番のボタンでは測っていない。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PressForceSpec:
    ok_lo: float
    ok_hi: float
    max_n: float
    post_press_s: float
    release_n: float

    @staticmethod
    def from_cfg(cfg: dict[str, Any] | None) -> PressForceSpec:
        """configs/contest.yaml の press_force から作る（無ければ既定の値）。"""
        c = cfg or {}
        lo, hi = c.get("ok_range_n", (3.0, 8.0))
        return PressForceSpec(ok_lo=float(lo), ok_hi=float(hi), max_n=float(c.get("max_n", 20.0)),
                              post_press_s=float(c.get("post_press_s", 0.5)), release_n=float(c.get("release_n", 0.3)))


class PressForceMeter:
    """1 試行の、目標のボタンを押す力の記録。

    update() に、周期ごとの力の最大を渡す（物理の 1 ステップごとでもよい）。点灯したら mark_lit()、
    点灯のあとの時間の最後の周期で mark_end() を呼ぶ。
    """

    def __init__(self, spec: PressForceSpec):
        self.spec = spec
        self.measured = False
        self.max_force = 0.0
        self.max_force_at_lit: float | None = None
        self.lit = False
        self.end_force: float | None = None

    def update(self, force: float | None) -> None:
        if force is None:
            return
        self.measured = True
        self.max_force = max(self.max_force, float(force))

    def mark_lit(self) -> None:
        """目標のボタンが点灯した（点灯までの力の最大を残す）。"""
        if not self.lit:
            self.lit = True
            self.max_force_at_lit = self.max_force

    def mark_end(self, force: float | None) -> None:
        """点灯のあとの時間の終わりの力（指を離したかの判定に使う）。"""
        if force is not None:
            self.end_force = float(force)

    def summary(self) -> dict[str, Any]:
        """記録の要約。測れなかった（力を測れないロボット）ときは値を None にする。

        max_press_force_n     試行全体での、押す力の最大
        press_force_in_range  押す力の最大が適正な範囲か（目標のボタンが点灯したときだけ。点灯しなければ None）
        press_force_over_max  押しすぎたか
        released_after_press  点灯のあとの時間の終わりに、指を離していたか（点灯しなければ None）
        """
        if not self.measured:
            return {"max_press_force_n": None, "press_force_in_range": None, "press_force_over_max": None,
                    "released_after_press": None}
        s = self.spec
        out: dict[str, Any] = {
            "max_press_force_n": round(self.max_force, 2),
            "press_force_over_max": self.max_force > s.max_n,
            "press_force_in_range": None,
            "released_after_press": None,
        }
        if self.lit:
            out["press_force_in_range"] = s.ok_lo <= self.max_force <= s.ok_hi
            if self.end_force is not None:
                out["released_after_press"] = self.end_force < s.release_n
        return out


def summarize_press(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """試行の結果（EpisodeResult.to_dict() か、模擬 G1 の結果の JSON）の一覧から、押す力の集計を作る。

    measured: 押す力を測れた試行の数。in_range_rate / released_rate は success の試行のうちの割合。
    """
    meas = [r for r in rows if r.get("max_press_force_n") is not None]
    ok = [r for r in meas if r.get("outcome") == "success"]
    if not meas:
        return {"measured": 0}
    forces = [float(r["max_press_force_n"]) for r in ok]
    rel = [r for r in ok if r.get("released_after_press") is not None]
    return {
        "measured": len(meas),
        "max_press_force_n_success_mean": round(sum(forces) / len(forces), 2) if forces else None,
        "max_press_force_n_success_max": round(max(forces), 2) if forces else None,
        "in_range_rate": round(sum(bool(r.get("press_force_in_range")) for r in ok) / len(ok), 3) if ok else None,
        "over_max_trials": sum(bool(r.get("press_force_over_max")) for r in meas),
        "released_rate": round(sum(bool(r["released_after_press"]) for r in rel) / len(rel), 3) if rel else None,
    }
