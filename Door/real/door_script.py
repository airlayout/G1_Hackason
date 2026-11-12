"""11-12 台本の計画（docs/plan/2026-11-12-g1-door-scripted-stage123.md・S2・S3・S5）: 段 2（レバーを下げる）と
段 3（扉を押し開ける）を、学習した方策を使わず手で書いた状態機械で進める。numpy だけ。

走らせ役（door_run_staged.StagedDoor）が毎周期（50 Hz）step を呼ぶ。ここは扉の座標の上の「手のひらの行き先（goal）」と
脚への指令だけを決める。goal を関節の角度にするのは呼ぶ側（g1_arm_ik.ArmIK）。機体・逆運動学から切り離してあるので、
偽の手のひらで状態の移りを試験できる。

  小段  2a 待つ → 2b 構える（経由点 → レバーの上）→ 2c 下ろして触れる → 2d 押し下げる
        → 3a 掛け金を外す（下げたまま前へ・扉 3°）→ 3c 押しながら歩く（扉 30°）→ done
        体が回ったら retreat（手を上げて 2a からやり直し・1 回だけ）。だめな時は failed（呼ぶ側が立ったまま止める）
  座標  door_frame.py と同じ（原点＝開口の中心の床・+x＝押して入る向き・把手が左ならレバーは +y）
  定数  script_params.json（load_params）。回と回の間はこのファイルだけ直す

⚠️ 定数の初めの値は 11-11 の記録と机の上の届きの確かめから置いた。実機で止める所を奥へ進めながら合わせる
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

import door_frame as dfr
import door_staged as ds
import hand_door_angle as hda

Walk = Tuple[bool, float, float, float]
STAND: Walk = (False, 0.0, 0.0, 0.0)
SUBS = ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed")
STOP_AFTER = ("", "2a", "2b", "2c", "2d", "3a")
DT = 0.02
ROW_LEN = 12


@dataclass(frozen=True)
class ScriptParams:
    # 構えの位置（pelvis・扉の座標）。y はレバーの当てる点からの差（把手が左の扉の向き。右の扉は符号を返す）
    stance_x: float = -0.40
    stance_y_off: float = -0.22    # 手が肩より 8 cm 外に来る（内だと肘が胸の前へ回り込む・11-12 机の上の確かめ）
    stance_yaw_deg: float = 0.0
    stance_tol: Tuple[float, float, float] = (0.04, 0.04, 6.0)  # 前後 m・左右 m・向き °
    # 逆運動学の基準の姿勢（左腕 7 軸）。肩 roll を外へ開いた形（11-11 の s2f の角度 −0.8,0.17,… だと肘が胸の前へ回り込み、
    # 肩 roll が体の側へ −0.5〜−0.8 rad 押す＝11-03 に熱を持った型。11-12 机の上の通り道の確かめで決めた）
    posture: Tuple[float, ...] = (-0.8, 1.1, -0.25, 0.4, 1.4, -1.0, 0.55)
    # 11-12 実機 2・3 本目: 手の先が扉へまっすぐ向いていると、手のひらをレバーの上に置いた時に手の先（原点の 10〜12 cm 先）が板に届く。
    # 手の先を水平のまま蝶番の側へこの角度だけ振る（0＝扉へまっすぐ・90＝棒に沿って真横）。負の値で向きを縛らない
    hand_dir_deg: float = 60.0
    # 2a 待つ
    settle_s: float = 1.5
    yaw_still_deg: float = 1.5     # 直近 0.5 秒の向きの幅がこれ未満なら止まっている
    wait_max_s: float = 5.0
    # 2b 構える（レバーの当てる点からの差 [前, 横, 上]。横は把手の側が正）
    # 11-12 実機 1 本目: レバーの手前 12 cm をまっすぐ上げたら、手の先（手のひらの原点の約 10 cm 先）が棒の下に掛かり、
    # 肩 pitch 17 N·m で押し上げ続けて 5 秒で 53→76 °C・機体が後ろへ 23 cm 押された。棒の無い把手の側（付け根の外）で上げてから横へ入る
    # 道筋: 把手の側・板から遠い所でレバーより 12 cm 高く上げる → その高さのまま横へ → 前へ出てレバーの 10 cm 上 → 2c で上から下ろす
    via: Tuple[Tuple[float, float, float], ...] = ((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.06, 0.12))
    # 11-12 実機 2 本目: 前 0 だと手の先（手のひらの原点の 10〜12 cm 先）が板に当たり、機体が後ろへ 10 cm 押された → 7 cm 手前
    # 手を 60° 振った時: 手のひらの原点を棒の 3 cm 手前・6 cm 付け根寄りに置くと、手の中ほど（原点から 7 cm）が当てる点の上を斜めに渡り、
    # 手の先は板から約 5 cm 手前に残る
    above: Tuple[float, float, float] = (-0.03, 0.06, 0.10)
    raise_speed: float = 0.15
    raise_max_s: float = 10.0
    via_tol: float = 0.04
    reach_tol: float = 0.02
    base_s: float = 0.3            # レバーの上で止まって空中の値（遅れ・力）を取る長さ
    raise_stall_s: float = 1.2     # 2b: 行き先が先行の上限に張り付いたまま、手がこの間 1 cm も進まなければ止まる（何かに当たっている）
    pelvis_move_max: float = 0.08  # 2b〜3a（脚は止めている）: pelvis が 2a の終わりからこれより動いたら止まる（押されて後ずさりしている）
    # 2c 下ろして触れる
    touch_speed: float = 0.03
    touch_floor: float = -0.09     # 当てる点よりこれだけ下まで触れなければ空振り（手は先が上がっているので、手のひらは棒より下まで下がる）
    # 11-12 実機 3 本目: 「空中の遅れからの差」は、着いた直後の垂れ（−1.2 cm）が下ろす間に消えるだけで立った（偽の触れた）。
    # 「行き先が手のひらよりこれだけ下にあり、かつ手のひらが直近 touch_still_s に touch_still_m も下がっていない」で見る
    lag_line: float = 0.015
    touch_still_s: float = 0.3
    touch_still_m: float = 0.003
    tau_line: float = 6.0          # 肩 pitch の力の見積もりの変化 [N·m]（空中の値から。姿勢で 3 N·m は動く）
    touch_hold: int = 3
    # 11-12 実機 4 本目: このレバーのばねは軽く、手は止まらずにレバーごと下がった（肩 roll の力が +4 N·m 動いただけ）＝止まりでは触れたと分からない。
    # 手のひらが当てる点の高さ＋touch_at まで下りたら触れたとみなす（止まり・力で先に分かればそちらで）
    touch_by_height: bool = True
    touch_at: float = 0.0
    # 2d 押し下げる
    press_speed: float = 0.04
    press_depth: float = 0.04      # 触れた高さからの下がり（当日レバーの回る量を測って決める）
    press_min: float = 0.02        # 下がりが止まった時、これ以上下がっていれば下げたとみなす
    press_max_s: float = 3.0
    stall_s: float = 0.6
    hold_extra: float = 0.01       # 3c（掛け金が外れた後）は行き先を下げた時の高さよりこれだけ下に置く
    lead_max: float = 0.05         # 行き先が手のひらより先へ行ける上限（押し付けの力の頭打ち）
    # 3a 掛け金を外す
    unlatch_speed: float = 0.05
    unlatch_deg: float = 3.0
    unlatch_stall_s: float = 3.0
    nudge_vx: float = 0.2
    nudge_s: float = 0.5
    # 3c 押しながら歩く
    push_lead_deg: float = 4.0
    push_dz: float = 0.0           # 3c の行き先の高さの足し（0＝下げた時の高さ − hold_extra）
    push_dr: float = 0.0           # 行き先を蝶番から遠ざける足し [m]（負で蝶番寄り＝板の上へ移す時）
    goal_deg: float = 30.0
    push_max_s: float = 20.0
    push_stall_s: float = 3.0
    reach_hi: float = 0.37         # 手のひらが pelvis の前へこれだけ出たら脚を前へ（腕の届きは肩の高さで約 0.41 m）
    reach_lo: float = 0.30         # ここまで戻ったら脚を止める
    walk_vx: float = 0.2
    walk_vy: float = 0.0
    walk_wz: float = 0.0
    pulse_max_s: float = 1.0
    pulse_gap_s: float = 0.6
    # 守り
    tau_max: float = 20.0          # 肩 pitch の力の見積もりの上限 [N·m]
    tau_max_s: float = 2.0
    yaw_abort_deg: float = 8.0     # 2c〜3a で体がこれだけ回ったら手を上げてやり直す
    retries: int = 1
    waist_comp_from: str = "2c"    # この小段から腰 yaw の打ち消しを頼む（走らせ役の --waist-comp-gain が 0 なら効かない）。"" で頼まない
    done_hold_s: float = 1.0
    stop_after: str = ""           # この小段が終わったら立ったまま止まる（"" は最後まで）
    pose_tau_s: float = 0.2        # 腕の行き先に使う pelvis の姿勢のなまし
    note: str = ""


def load_params(path: Optional[str] = None, **override) -> ScriptParams:
    """定数ファイル（JSON）を読む。知らない名前・型の違う値は起動で落とす（当日の打ち間違いを機体の前で見つける）"""
    raw = {}
    if path:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    raw = {**raw, **{k: v for k, v in override.items() if v is not None}}
    known = {f.name: f for f in fields(ScriptParams)}
    bad = sorted(set(raw) - set(known))
    if bad:
        raise ValueError(f"台本の定数に知らない名前: {bad}")
    out = {}
    for k, v in raw.items():
        d = getattr(ScriptParams(), k)
        if isinstance(d, tuple):
            v = tuple(tuple(float(x) for x in e) if isinstance(e, (list, tuple)) else float(e) for e in v)
        elif isinstance(d, bool) or isinstance(d, str):
            if not isinstance(v, type(d)):
                raise ValueError(f"台本の定数 {k} の型が違う: {v!r}")
        elif isinstance(d, int):
            v = int(v)
        else:
            v = float(v)
        out[k] = v
    p = replace(ScriptParams(), **out)
    check_params(p)
    return p


def check_params(p: ScriptParams) -> None:
    if len(p.posture) != 7 or len(p.above) != 3 or any(len(v) != 3 for v in p.via) or len(p.stance_tol) != 3:
        raise ValueError("台本の定数: posture は 7 つ・above と via の各点は 3 つ・stance_tol は 3 つ")
    if p.stop_after not in STOP_AFTER:
        raise ValueError(f"stop_after は {STOP_AFTER} のどれか: {p.stop_after!r}")
    if p.waist_comp_from not in ("",) + SUBS[:6]:
        raise ValueError(f"waist_comp_from は小段の名前か空: {p.waist_comp_from!r}")
    nums = [getattr(p, f.name) for f in fields(p) if isinstance(getattr(p, f.name), (int, float))]
    if not all(math.isfinite(float(v)) for v in nums):
        raise ValueError("台本の定数に有限でない値がある")
    pos = ("settle_s", "raise_speed", "touch_speed", "press_speed", "press_depth", "lead_max", "unlatch_speed", "goal_deg",
           "reach_tol", "via_tol", "tau_max", "pulse_max_s")
    for k in pos:
        if not getattr(p, k) > 0:
            raise ValueError(f"台本の定数 {k} は正: {getattr(p, k)}")
    if not (0 < p.press_min <= p.press_depth and p.reach_lo < p.reach_hi and p.unlatch_deg < p.goal_deg):
        raise ValueError("台本の定数: press_min ≤ press_depth・reach_lo < reach_hi・unlatch_deg < goal_deg")
    if p.walk_vx and abs(p.walk_vx) < ds.V_MIN:
        raise ValueError(f"walk_vx は 0 か {ds.V_MIN} m/s 以上（純正は 0.1〜0.15 m/s では動かない）")
    if not (-0.60 <= p.stance_x <= ds.NEAR_STOP_X - 0.02):
        raise ValueError(f"stance_x は −0.60〜{ds.NEAR_STOP_X - 0.02:.2f} m（近すぎると段 1 が止まる）: {p.stance_x}")


def script_stance(p: ScriptParams, width: float, backset: float, lever_height: float, lever_left: bool = True) -> ds.Stance:
    """台本の構えの位置（段 1 の寄りの行き先）"""
    lr = 1.0 if lever_left else -1.0
    ref = ds.lever_ref_point(width, backset, lever_height, lever_left)
    return ds.Stance(dfr.PlanarPose(p.stance_x, float(ref[1]) + lr * p.stance_y_off, lr * math.radians(p.stance_yaw_deg)),
                     p.stance_tol[0], p.stance_tol[1], math.radians(p.stance_tol[2]))


class PoseFilter:
    """pelvis の姿勢を 1 次でなます（FAST-LIO の 10 Hz の段と歩みの揺れを腕の行き先へそのまま流さない）"""

    def __init__(self, tau_s: float) -> None:
        self.tau, self.pose = tau_s, None

    def step(self, pose: dfr.PlanarPose, dt: float = DT) -> dfr.PlanarPose:
        if self.pose is None or self.tau <= 0:
            self.pose = pose
            return pose
        a = min(1.0, dt / self.tau)
        o = self.pose
        self.pose = dfr.PlanarPose(o.x + a * (pose.x - o.x), o.y + a * (pose.y - o.y), o.yaw + a * ds.wrap(pose.yaw - o.yaw))
        return self.pose


@dataclass(frozen=True)
class ScriptOut:
    sub: str
    goal: Optional[np.ndarray]     # 扉の座標での手のひらの行き先（None は腕を下ろしたまま）
    walk: Walk
    direction: Optional[np.ndarray]  # 扉の座標での手の先の向き（単位ベクトル。None は縛らない）
    angle_obs: np.ndarray          # door_angle_lidar の形 [角度 rad, 測れたか, 古さ/1.0 s]
    waist_comp: bool
    stop: str = ""                 # 空でなければ立ったまま止まる理由
    success: bool = False
    info: str = ""


def _toward(g: np.ndarray, wp: np.ndarray, step: float) -> np.ndarray:
    d = wp - g
    n = float(np.linalg.norm(d))
    return wp.copy() if n <= step else g + d * (step / n)


class ScriptDoor:
    """段 2・3 の台本。step には、時刻・なました pelvis の姿勢・扉の座標の手のひら・肩 pitch の力の見積もり・
    pelvis から見た手のひらの前への出（m）を渡す"""

    def __init__(self, p: ScriptParams, width: float, backset: float, lever_height: float, lever_left: bool = True) -> None:
        check_params(p)
        self.p, self.lr = p, (1.0 if lever_left else -1.0)
        self.ref = ds.lever_ref_point(width, backset, lever_height, lever_left)
        self.line = hda.door_line(width, lever_left)
        self.sub = "2a"
        self.t_sub: Optional[float] = None
        self.t_prev: Optional[float] = None
        self.goal: Optional[np.ndarray] = None
        self.wp_i = 0
        self.yaws: List[Tuple[float, float]] = []
        self.yaw0: Optional[float] = None
        self.base: List[Tuple[float, float]] = []   # レバーの上で止まっている間の（遅れ, 力）
        self.lag0 = self.tau0 = 0.0
        self.touch_n = 0
        self.z_hist: List[Tuple[float, float]] = []      # 2c: 直近の（時刻, 手のひらの高さ）
        self.z_contact: Optional[float] = None
        self.z_min = math.inf
        self.t_progress = 0.0
        self.palm_ref: Optional[np.ndarray] = None
        self.angle = 0.0
        self.angle_best = 0.0
        self.nudged = False
        self.nudge_until: Optional[float] = None
        self.walking_until: Optional[float] = None
        self.walk_rest_until = 0.0
        self.tau_over_t: Optional[float] = None
        self.retries_left = p.retries
        self.xy0: Optional[Tuple[float, float]] = None   # 2a の終わりの pelvis の位置
        self.raise_best = math.inf                       # 2b: 今の経由点までの残りの最小
        self.raise_t = 0.0
        self.reason = ""
        self.success = False
        self.lag = self.tau = self.fwd = math.nan
        self.palm = np.full(3, math.nan)

    # --- 部品 ---
    def _off(self, d: Sequence[float]) -> np.ndarray:
        return self.ref + np.array([d[0], self.lr * d[1], d[2]])

    def _go(self, sub: str, t: float) -> None:
        print(f"[script] {self.sub} → {sub}（{self._where()}）", flush=True)
        self.sub, self.t_sub = sub, t

    def _where(self) -> str:
        d = self.palm - self.ref
        return (f"手のひら−当てる点 前 {100 * d[0]:+.1f} 横 {100 * d[1]:+.1f} 上 {100 * d[2]:+.1f} cm・"
                f"扉 {math.degrees(self.angle):.1f}°")

    def _fail(self, why: str, t: float) -> None:
        self.reason = f"台本 {self.sub}: {why}"
        self._go("failed", t)

    def _angle_obs(self) -> np.ndarray:
        if self.palm_ref is None:
            return dfr.door_closed_angle_obs()
        return np.array([self.angle, 1.0, hda.LATENCY_S / hda.MAX_AGE_S])

    def _lead_cap(self, g: np.ndarray, palm: np.ndarray) -> np.ndarray:
        """行き先を手のひらから lead_max の内へ（上下と水平を別々に。下へ押さえたままでも前へ進められるように）"""
        out = np.array(g, dtype=float, copy=True)
        out[2] = palm[2] + max(-self.p.lead_max, min(self.p.lead_max, out[2] - palm[2]))
        d = out[:2] - palm[:2]
        n = float(np.linalg.norm(d))
        if n > self.p.lead_max:
            out[:2] = palm[:2] + d * (self.p.lead_max / n)
        return out

    def _stop_after(self, done_sub: str) -> bool:
        if self.p.stop_after == done_sub:
            self.reason = f"台本: {done_sub} が終わった（止める所・{self._where()}）"
            return True
        return False

    def row(self) -> np.ndarray:
        """記録の 1 行: [小段の番号, 行き先 xyz, 手のひら xyz, 扉の角度 °, 遅れ m, 肩 pitch の力, 手の前への出 m, 歩きの指令を出しているか]"""
        g = self.goal if self.goal is not None else np.full(3, math.nan)
        return np.array([SUBS.index(self.sub), *g, *self.palm, math.degrees(self.angle), self.lag, self.tau, self.fwd,
                         float(self.walking_until is not None or self.nudge_until is not None)])

    # --- 毎周期 ---
    def step(self, t: float, pose: dfr.PlanarPose, palm: np.ndarray, tau_pitch: float, palm_fwd: float) -> ScriptOut:  # noqa: C901
        p = self.p
        if self.t_sub is None:
            self.t_sub = t
        dt = DT if self.t_prev is None else min(0.1, max(0.0, t - self.t_prev))
        self.t_prev = t
        palm = np.asarray(palm, dtype=float)
        self.palm, self.tau, self.fwd = palm, float(tau_pitch), float(palm_fwd)
        if self.sub not in ("failed", "done") and not (np.isfinite(palm).all() and math.isfinite(pose.yaw)):
            self._fail("手のひらか姿勢が有限でない", t)
        if self.sub in ("2c", "2d", "3a", "3c") and self.goal is not None:
            self.lag = float(palm[2] - self.goal[2])
        if self.palm_ref is not None and np.isfinite(palm).all():
            self.angle = hda.door_angle(self.line, self.palm_ref, palm, "hinge")
        self.yaws.append((t, pose.yaw))
        self.yaws = [e for e in self.yaws if t - e[0] <= 0.5]
        walk: Walk = STAND
        stop = ""

        # 守り: 肩 pitch の力・体の回り
        if self.sub in ("2b", "2c", "2d", "3a") and self.xy0 is not None and self.nudge_until is None and not self.nudged:
            moved = math.hypot(pose.x - self.xy0[0], pose.y - self.xy0[1])
            if moved > p.pelvis_move_max:
                self._fail(f"脚を止めているのに pelvis が {100 * moved:.0f} cm 動いた（押されて後ずさりしている）", t)
        if self.sub in ("2b", "2c", "2d", "3a", "3c"):
            if math.isfinite(tau_pitch) and abs(tau_pitch) > p.tau_max:
                self.tau_over_t = t if self.tau_over_t is None else self.tau_over_t
                if t - self.tau_over_t > p.tau_max_s:
                    self._fail(f"肩 pitch の力 {tau_pitch:+.1f} N·m が {p.tau_max_s:.0f} 秒 上限 {p.tau_max:.0f} を越えた", t)
            else:
                self.tau_over_t = None
        if self.sub in ("2c", "2d", "3a") and self.yaw0 is not None:
            turned = math.degrees(ds.wrap(pose.yaw - self.yaw0))
            if abs(turned) > p.yaw_abort_deg:
                if self.retries_left > 0:
                    self.retries_left -= 1
                    print(f"[script] 体が {turned:+.1f}° 回った: 手を上げてやり直す（残り {self.retries_left} 回）", flush=True)
                    self.palm_ref, self.angle, self.z_contact = None, 0.0, None
                    self._go("retreat", t)
                else:
                    self._fail(f"体が {turned:+.1f}° 回った（やり直しは使い切った）", t)

        if self.sub == "2a":
            still = len(self.yaws) > 5 and math.degrees(max(y for _, y in self.yaws) - min(y for _, y in self.yaws)) < p.yaw_still_deg
            if t - self.t_sub >= p.settle_s and still:
                self.yaw0, self.xy0 = pose.yaw, (pose.x, pose.y)
                self.raise_best, self.raise_t = math.inf, t
                if self.goal is None:  # 初めの 2a。やり直しの 2a は手を上げたまま待っているので行き先と経由点の番号を保つ
                    self.goal, self.wp_i = palm.copy(), 0
                if self._stop_after("2a"):
                    stop = self.reason
                else:
                    self._go("2b", t)
            elif t - self.t_sub > p.wait_max_s:
                self._fail(f"{p.wait_max_s:.0f} 秒待っても体が止まらない", t)
        elif self.sub == "2b":
            wps = [self._off(v) for v in p.via] + [self._off(p.above)]
            wp = wps[self.wp_i]
            self.goal = self._lead_cap(_toward(self.goal, wp, p.raise_speed * dt), palm)  # 当たったら行き先は 5 cm 先で止まる
            at_goal = float(np.linalg.norm(self.goal - wp)) < 1e-9
            last = self.wp_i == len(wps) - 1
            left = float(np.linalg.norm(palm - wp))
            if left < self.raise_best - 0.01:
                self.raise_best, self.raise_t = left, t
            if at_goal and not last and float(np.linalg.norm(palm - wp)) < p.via_tol:
                self.wp_i += 1
                self.base = []
                self.raise_best, self.raise_t = math.inf, t
            elif at_goal and last and float(np.linalg.norm(palm - wp)) < p.reach_tol:
                self.base.append((float(palm[2] - self.goal[2]), float(tau_pitch)))
                if len(self.base) * DT >= p.base_s:
                    self.lag0 = float(np.mean([b[0] for b in self.base]))
                    self.tau0 = float(np.nanmean([b[1] for b in self.base])) if np.isfinite([b[1] for b in self.base]).any() else 0.0
                    self.touch_n, self.z_hist = 0, []
                    if self._stop_after("2b"):
                        stop = self.reason
                    else:
                        self._go("2c", t)
            else:
                self.base = []
            if self.sub == "2b" and not stop and left > p.via_tol and t - self.raise_t > p.raise_stall_s:
                self._fail(f"手が {p.raise_stall_s} 秒 進まない（何かに当たっている。経由点 {self.wp_i}・残り {100 * left:.1f} cm・"
                           f"肩 pitch の力 {tau_pitch:+.1f} N·m）", t)
            if self.sub == "2b" and not stop and t - self.t_sub > p.raise_max_s:
                self._fail(f"{p.raise_max_s:.0f} 秒で手がレバーの上に着かない（経由点 {self.wp_i}・残り "
                           f"{100 * float(np.linalg.norm(palm - wp)):.1f} cm）", t)
        elif self.sub == "2c":
            g = self.goal.copy()
            g[2] = max(g[2] - p.touch_speed * dt, self.ref[2] + p.touch_floor - p.lead_max)
            self.goal = self._lead_cap(g, palm)
            lag = float(palm[2] - self.goal[2])
            self.z_hist.append((t, float(palm[2])))
            self.z_hist = [e for e in self.z_hist if t - e[0] <= p.touch_still_s]
            still = (t - self.z_hist[0][0] >= 0.8 * p.touch_still_s
                     and self.z_hist[0][1] - float(palm[2]) < p.touch_still_m)
            dtau = abs(tau_pitch - self.tau0) if math.isfinite(tau_pitch) else 0.0
            self.touch_n = self.touch_n + 1 if ((lag > p.lag_line and still) or dtau > p.tau_line) else 0
            by_height = p.touch_by_height and palm[2] <= self.ref[2] + p.touch_at
            if self.touch_n >= p.touch_hold or by_height:
                self.z_contact, self.z_min, self.t_progress = float(palm[2]), float(palm[2]), t
                print(f"[script] 触れた（{'高さで' if by_height and self.touch_n < p.touch_hold else '止まり・力で'}）: 高さ {palm[2]:.3f} m（当てる点 {self.ref[2]:.3f}）・遅れ {100 * lag:.1f} cm・力の変化 {dtau:.1f} N·m", flush=True)
                if self._stop_after("2c"):
                    stop = self.reason
                else:
                    self._go("2d", t)
            elif palm[2] <= self.ref[2] + p.touch_floor:
                self._fail(f"当てる点の {-100 * p.touch_floor:.0f} cm 下まで下ろしても触れない（空振り）", t)
        elif self.sub == "2d":
            g = self.goal.copy()  # 行き先は手のひらの lead_max 先まで下げ続ける（ばねのあるレバーを押し切る。力は lead_max で頭打ち）
            g[2] = g[2] - p.press_speed * dt
            self.goal = self._lead_cap(g, palm)
            if palm[2] < self.z_min - 0.001:
                self.z_min, self.t_progress = float(palm[2]), t
            depth = self.z_contact - float(palm[2])
            stalled = t - self.t_progress > p.stall_s or t - self.t_sub > p.press_max_s
            if depth >= p.press_depth or (stalled and depth >= p.press_min):
                self.palm_ref, self.angle, self.angle_best, self.t_progress = palm.copy(), 0.0, 0.0, t
                self.goal = np.array([palm[0], palm[1], self.goal[2]])  # 押さえる深さはそのまま（掛け金が外れるまで）
                print(f"[script] 下げた: 下がり {100 * depth:.1f} cm・肩 pitch の力 {tau_pitch:+.1f} N·m", flush=True)
                if self._stop_after("2d"):
                    stop = self.reason
                else:
                    self._go("3a", t)
            elif stalled:
                self._fail(f"レバーが下がらない（下がり {100 * depth:.1f} cm・線 {100 * p.press_min:.0f} cm・力 {tau_pitch:+.1f} N·m）", t)
        elif self.sub == "3a":
            g = self.goal.copy()
            g[0] += p.unlatch_speed * dt
            self.goal = self._lead_cap(g, palm)
            deg = math.degrees(self.angle)
            if deg > self.angle_best + 0.5:
                self.angle_best, self.t_progress = deg, t
            if self.nudge_until is not None:
                if t < self.nudge_until:
                    walk = (True, p.nudge_vx, 0.0, 0.0)
                else:
                    self.nudge_until, self.t_progress = None, t
            if deg >= p.unlatch_deg:
                self.nudge_until = None
                self.t_progress, self.angle_best = t, deg
                if self._stop_after("3a"):
                    stop = self.reason
                else:
                    self._go("3c", t)
            elif self.nudge_until is None and t - self.t_progress > p.unlatch_stall_s:
                if not self.nudged:
                    self.nudged, self.nudge_until = True, t + p.nudge_s
                    print(f"[script] 扉が {p.unlatch_stall_s:.0f} 秒動かない: 脚を前へ {p.nudge_vx} m/s で {p.nudge_s} 秒", flush=True)
                else:
                    self._fail(f"掛け金が外れない（扉 {deg:.1f}°・脚で 1 回押した後）", t)
        elif self.sub == "3c":
            deg = math.degrees(self.angle)
            tgt = math.radians(min(deg + p.push_lead_deg, p.goal_deg + p.push_lead_deg))
            v = self.palm_ref[:2] - self.line.hinge_xy
            r = float(np.linalg.norm(v))
            v = v * ((r + p.push_dr) / r)
            a = self.line.open_sign * tgt
            c, s = math.cos(a), math.sin(a)
            xy = self.line.hinge_xy + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
            # 掛け金が外れた後は強く押さえない: 高さは下げた時の手のひら − hold_extra（＋ push_dz）
            self.goal = self._lead_cap(np.array([xy[0], xy[1], self.palm_ref[2] - p.hold_extra + p.push_dz]), palm)
            if deg > self.angle_best + 1.0:
                self.angle_best, self.t_progress = deg, t
            # 脚: 手が前へ出すぎたら短く前へ（出し続けない。止まって腕で押し、また出す）
            if self.walking_until is not None:
                if t >= self.walking_until or palm_fwd < p.reach_lo:
                    self.walking_until, self.walk_rest_until = None, t + p.pulse_gap_s
                else:
                    walk = (True, p.walk_vx, p.walk_vy, p.walk_wz)
            elif palm_fwd > p.reach_hi and t >= self.walk_rest_until and p.walk_vx:
                self.walking_until = t + p.pulse_max_s
                walk = (True, p.walk_vx, p.walk_vy, p.walk_wz)
            if deg >= p.goal_deg:
                self.walking_until = None
                walk = STAND
                self.success = True
                self.goal = palm.copy()
                self._go("done", t)
            elif t - self.t_progress > p.push_stall_s:
                self._fail(f"扉が {p.push_stall_s:.0f} 秒 開いていかない（{deg:.1f}°・手の前への出 {palm_fwd:.2f} m）", t)
            elif t - self.t_sub > p.push_max_s:
                self._fail(f"{p.push_max_s:.0f} 秒で {p.goal_deg:.0f}° に届かない（{deg:.1f}°）", t)
        elif self.sub == "retreat":
            up = np.array([palm[0], palm[1], self.ref[2] + p.above[2]]) if self.goal is None else np.array(
                [self.goal[0], self.goal[1], self.ref[2] + p.above[2]])
            self.goal = _toward(self.goal if self.goal is not None else palm.copy(), up, p.raise_speed * dt)
            if abs(palm[2] - up[2]) < p.via_tol or t - self.t_sub > 3.0:
                self.yaws = []
                self.xy0 = None
                self.wp_i = len(p.via)  # 経由点は飛ばしてレバーの上へ戻る
                self._go("2a", t)
        elif self.sub == "done":
            if t - self.t_sub >= p.done_hold_s:
                stop = f"台本: 扉 {math.degrees(self.angle):.0f}° まで開けた（成功）"
        if self.sub == "failed":
            stop = self.reason
        wc = bool(p.waist_comp_from) and self.sub in SUBS[SUBS.index(p.waist_comp_from):5]  # 脚を止めている 3a まで
        goal = self.goal.copy() if self.goal is not None else None
        direction = None
        if p.hand_dir_deg >= 0.0:
            a = math.radians(p.hand_dir_deg)
            direction = np.array([math.cos(a), -self.lr * math.sin(a), 0.0])
        return ScriptOut(self.sub, goal, walk, direction, self._angle_obs(), wc, stop, self.success and self.sub == "done", self._where())
