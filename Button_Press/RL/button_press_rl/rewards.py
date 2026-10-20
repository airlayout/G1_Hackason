"""教師の報酬（torch。シミュレーターに依存しないので、tests/test_rewards.py で単体で確かめられる）。

考え方:
- 毎周期の項はすべて「減点」（0 以下）。＋は出来事（点灯・指を離した）のときだけ。
  毎周期 ＋ の項があると、押して試行を終えるより、ボタンの前に居続けるほうが得になるため
- 段階の項（手前で横をそろえる → 面に近づける → 沈める）は、その段階に入る前は最大の減点にしておく。
  入ったときだけ減点を足す形にすると、段階に入った瞬間に報酬が下がり、近づかなくなるため（tests/test_rewards.py）
- 押す力は 3 つで見る: 試行全体（点灯のあとの時間も含む）の力の最大が適正な範囲か（試行の終わりに、範囲なら ＋、
  外なら −。評価環境の press_force_in_range と同じ量）、上限を超えた分（毎周期、2 乗で減点）、当たる直前の速さ（衝撃）
- 沈める項は、点灯の深さではなく行き止まり（travel）まで押させる。ばね 1000 N/m × 3 mm = 3 N が適正な範囲の下限なので、
  点灯（2 mm）で止めると 2〜3 N で下限を割る（2026-10-20 の評価で 23%）
- 点灯のあと post_press_s の間は、指を離すように減点する（実物の呼びボタンと同じく、点灯したら離す）

位置はすべてロボットの腰（pelvis）の座標。axis は押す向き（ボタンが沈む向き。単位ベクトル）。
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import torch


@dataclass
class RewardWeights:
    # 近づく（ボタンの手前 approach_m の点まで）。遠くは距離に比例（reach_far_m で頭打ち）、近くは tanh の 2 つを半分ずつ。
    # tanh だけだと、手の初期位置（約 0.6 m 先）で tanh が 1 に張りついて、近づいても報酬が変わらず、
    # 学習が進まなかった（2026-10-19）
    reach: float = 2.0
    reach_far_m: float = 0.80
    reach_scale_near_m: float = 0.05
    approach_m: float = 0.03
    # 手前に来たら、押す向きに対する横のずれを小さく
    align: float = 1.0
    align_scale_m: float = 0.01
    near_m: float = 0.06  # 面からこの距離より近いと「手前に来た」
    # 横がそろったら、面まで近づける
    close: float = 1.0
    close_scale_m: float = 0.02
    lateral_ok_m: float = 0.012  # 横のずれがこれより小さいと「そろった」（ボタンの半径の最小 15 mm より内側）
    # 沈める（行き止まりまで）
    depth: float = 2.0
    # 押す力: 適正な範囲の上限を超えた分（[上限で割った値]^2）、当たる直前の速さ [m/s] を超えた分の 2 乗
    force_over: float = 3.0
    impact: float = 5.0
    impact_speed: float = 0.10
    impact_zone_m: float = 0.02
    touch_n: float = 0.2  # これより大きい力で「触れている」
    # 点灯のあとも押している（release_n 以上）
    hold_after: float = 1.0
    # 指先以外が乗り場や体に当たる力 [N] / 10（最大 1）
    unwanted_contact: float = 0.5
    # 滑らかさ: 行動の変化、関節の速さ、目標が可動範囲の端に張りついた関節の数
    action_rate: float = 0.01
    joint_vel: float = 1.0e-3
    joint_limit: float = 0.1
    # 出来事
    success: float = 30.0
    success_quality: float = 20.0  # 試行の終わりに、押す力の最大が適正な範囲なら ＋、外なら −
    not_fingertip: float = 20.0  # 中指の先以外（手のひら・ほかの指など）で押して点灯させた（点灯のボーナスは与えない）
    released: float = 10.0
    wrong_button: float = 50.0
    wrong_lit: float = 5.0  # 違うボタンが点灯している間の、毎周期の減点（打ち切らないので、押し得にならないように）
    force_abort: float = 20.0

    def to_dict(self) -> dict[str, float]:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass
class RewardInputs:
    """1 周期分の値（どれも先頭の次元が環境の数 N）。"""

    tip: torch.Tensor  # (N, 3) 指先の球の中心 [m]
    tip_vel: torch.Tensor  # (N, 3) 指先の速度 [m/s]
    face: torch.Tensor  # (N, 3) 目標のボタンの面の中心（押していないとき）[m]
    axis: torch.Tensor  # (N, 3) 押す向き
    tip_radius: float  # 指先の球の半径 [m]
    depth: torch.Tensor  # (N,) 目標のボタンの沈み [m]
    travel: float  # 行き止まりまでの沈み [m]
    force: torch.Tensor  # (N,) 目標のボタンを押す力（周期の中の最大）[N]
    max_force_ep: torch.Tensor  # (N,) 試行の中の押す力の最大 [N]
    unwanted_force: torch.Tensor  # (N,) 指先以外の接触力 [N]
    in_post: torch.Tensor  # (N,) bool 点灯のあとの時間か
    lit_event: torch.Tensor  # (N,) bool この周期で目標のボタンが点灯した
    wrong_event: torch.Tensor  # (N,) bool この周期で目標以外のボタンが点灯した
    released_event: torch.Tensor  # (N,) bool 点灯のあとの時間の終わりに指を離していた
    end_event: torch.Tensor  # (N,) bool 点灯のあとの時間の終わり（押す力の良し悪しを決める）
    force_abort: torch.Tensor  # (N,) bool 押す力が大きすぎた（上限の倍を超えた）
    wrong_lit: torch.Tensor  # (N,) bool 目標以外のボタンが点灯している
    tip_press: torch.Tensor  # (N,) bool 中指の先がボタンの面に触れる位置にある（点灯した周期に見る）
    action: torch.Tensor  # (N, 7)
    last_action: torch.Tensor  # (N, 7)
    dq: torch.Tensor  # (N, 7) 右腕の関節の速度 [rad/s]
    at_limit: torch.Tensor  # (N,) 目標が可動範囲の端に張りついた関節の数
    press_ok: tuple[float, float]  # 適正な範囲 [N]
    release_n: float


def geometry(x: RewardInputs) -> tuple[torch.Tensor, torch.Tensor]:
    """(押す向きの距離 along, 横のずれ lateral)。along は指先の球の中心から面までで、手前が負 [m]。"""
    rel = x.tip - x.face
    along = (rel * x.axis).sum(-1)
    lateral = torch.linalg.norm(rel - along.unsqueeze(-1) * x.axis, dim=-1)
    return along, lateral


def _stage(entered: torch.Tensor, cost: torch.Tensor) -> torch.Tensor:
    """段階の減点（0〜1）。段階に入る前は 1（最大）、入ったら cost。"""
    return entered * cost + (1.0 - entered)


def compute_rewards(x: RewardInputs, w: RewardWeights) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    """(1 周期の報酬 (N,), 項ごとの値 {名前: (N,)})。項ごとの値は重みを掛けたもの。"""
    along, lateral = geometry(x)
    approach = x.face - w.approach_m * x.axis
    d_app = torch.linalg.norm(x.tip - approach, dim=-1)
    # 球の表面と面のすき間（触れていれば 0）
    gap = torch.clamp(-along - x.tip_radius, min=0.0)
    near = (along > -w.near_m).float()
    aligned = (lateral < w.lateral_ok_m).float()
    not_post = (~x.in_post).float()
    touching = x.force > w.touch_n
    lo, hi = x.press_ok

    v_along = (x.tip_vel * x.axis).sum(-1)
    in_zone = ((gap < w.impact_zone_m) & ~touching).float()

    t: dict[str, torch.Tensor] = {}
    t["reach"] = -w.reach * 0.5 * (torch.clamp(d_app / w.reach_far_m, max=1.0)
                                    + torch.tanh(d_app / w.reach_scale_near_m)) * not_post
    t["align"] = -w.align * _stage(near, torch.tanh(lateral / w.align_scale_m)) * not_post
    t["close"] = -w.close * _stage(aligned, torch.tanh(gap / w.close_scale_m)) * not_post
    t["depth"] = -w.depth * _stage(aligned, 1.0 - torch.clamp(x.depth / x.travel, 0.0, 1.0)) * not_post
    t["force_over"] = -w.force_over * (torch.clamp(x.force - hi, min=0.0) / hi) ** 2
    t["impact"] = -w.impact * in_zone * torch.clamp(v_along - w.impact_speed, min=0.0) ** 2
    t["hold_after"] = -w.hold_after * (x.in_post & (x.force >= x.release_n)).float()
    t["unwanted_contact"] = -w.unwanted_contact * torch.clamp(x.unwanted_force / 10.0, max=1.0)
    t["action_rate"] = -w.action_rate * ((x.action - x.last_action) ** 2).sum(-1)
    t["joint_vel"] = -w.joint_vel * (x.dq ** 2).sum(-1)
    t["joint_limit"] = -w.joint_limit * x.at_limit
    in_band = ((x.max_force_ep >= lo) & (x.max_force_ep <= hi)).float()
    by_tip = (x.lit_event & x.tip_press).float()
    t["success"] = by_tip * w.success
    t["force_quality"] = w.success_quality * x.end_event.float() * (2.0 * in_band - 1.0)
    t["not_fingertip"] = -w.not_fingertip * (x.lit_event & ~x.tip_press).float()
    t["released"] = w.released * x.released_event.float()
    t["wrong_button"] = -w.wrong_button * x.wrong_event.float()
    t["wrong_lit"] = -w.wrong_lit * x.wrong_lit.float()
    t["force_abort"] = -w.force_abort * x.force_abort.float()
    total = torch.stack(list(t.values()), 0).sum(0)
    return total, t
