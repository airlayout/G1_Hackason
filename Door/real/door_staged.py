"""11-10 手順 L4（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.3・9.10）: 段に分けた走らせ役の中身。numpy だけ。
走らせ役 door_run_staged.py の輪から毎周期（50 Hz）呼ぶ。機体・記録の出どころとは切り離してあり、記録を流して確かめられる。

  段 0 Stage0     扉の正面 0.8〜1.3 m で止まり、目の網（c2）の扉の値を 1.5 秒溜めた中央値で測る → FAST-LIO の口へ REF
                  → まっすぐ 0.2 m 前へ → もう一度網で測り、FAST-LIO で運んだ値と比べる（8 cm・5°）→ 合えば平均で REF し直す
  段 1 Approach   手で書いた制御で構えの位置（s2f が学んだ範囲の内側）へ。先に回る → 斜めに寄る。指令は 0.2 m/s 以上か 0
                  （純正は 0.1〜0.15 m/s では動かない）。残りが小さい時は短く出して止まり、測り直す
  段 2 ArmStage   腕だけの方策 s2f。歩きの積み上げは 0 に止め（zero_walk_accumulator）、レバーを下げたら（0100 と同じ引き金）脚を放す。
                  扉の角度は手のひらの動きから（hand_door_angle・L2b）。3° で教師 f3t3 へ引き継ぐ
  持ち替え        handover_state: 積み上げ D・HOMIE の履歴は持ち越し、記憶（h・cell）だけ 0（シムの 0101 の zero と同じ）
  段 3〜4 OdomCarry 引き継いだ時の扉の座標の姿勢を /dog_odom で運ぶ（ライダーは使わない＝ユーザー 9.6）
扉の座標は door_frame.py と同じ（原点＝開口の中心の床・+x＝押して入る向き・把手が左ならレバーは +y）。
⚠️ 段 1 の指令の線・倍率は 11-06 の純正の応え方の実測から置いた値。L5 のシムと L7 の機体で確かめる
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

import door_frame as dfr
import hand_door_angle as hda

Cmd = Tuple[float, float, float]
ZERO: Cmd = (0.0, 0.0, 0.0)

# ── 段 0 ───────────────────────────────────────────────────
S0_SAMPLE_S = 1.5            # 網の扉の値を溜める長さ（10 Hz で 15 枚）
S0_MIN_SAMPLES = 8
S0_NET_WAIT_S = 3.0          # この間に網の値がそろわなければ失敗
S0_X_RANGE = (-1.3, -0.8)    # 段 0 の立ち位置（扉の正面 0.8〜1.3 m。遠いと深度が抜けて網が外れる・9.5）
S0_Y_MAX = 0.3
S0_YAW_MAX_DEG = 10.0
S0_STEP_VX = 0.2             # 2 回目の前にまっすぐ前へ出る指令
S0_STEP_M = 0.2
S0_SETTLE_S = 1.0
S0_STEP_MIN_M = 0.08          # 2 回目の時に FAST-LIO で運んだ値がこれより動いていなければ失敗（前へ出ていない＝運びの確かめが空振り）
S0_ANCHOR_WAIT_S = 8.0       # FAST-LIO の口が段 0 を作り終えるのを待つ上限（1 秒＋2.5 秒＋余裕）
WATCH_POS_M = 0.08           # 段 0 の見張りの線（9.2・ユーザー了承）
WATCH_YAW_DEG = 5.0
# 純正が指令に応える倍率（前後・左右・旋回）。左右・旋回は 11-06 の実測。前後は 11-11 の機体: 0.2 m/s を 1.92 秒で 32 cm 進んだ
# （巻き尺 76 → 44 cm・FAST-LIO 31 cm）＝ 0.8 倍。11-06 の 0.52 は方策の小刻みな指令での値だった
STOCK_GAIN = (0.8, 0.8, 0.61)  # 左右も 11-11 の機体で 0.2〜0.3 m/s に約 0.8 倍（3 秒で 0.5 m）

# ── 構えの位置（s2f の start_range と同じ決まり: 前 x・付け根からの横（レバーの側が正）・向き（レバーの側が正）） ──
S2F_START_RANGE = (-0.50, -0.37, -0.20, 0.11, -65.0, -25.0)
STANCE_X, STANCE_Y_OFF, STANCE_YAW_DEG = -0.435, -0.08, -45.0   # 計画書 9.3 の段 1 の終わり（扉から約 0.43 m・付け根の右・−45°）
# 入ったとみなす幅（s2f の範囲の内側いっぱい）。11-11 の機体: 純正は一度歩くと 15〜20 cm 進み、止まっても ±3〜5 cm 揺れる。
# ±4 cm・±6 cm・±6° では入らずに短く歩き直して 19 cm 行き過ぎ、扉の 0.3 m の止めに掛かった
STANCE_TOL = (0.06, 0.10, 13.0)
# 閉じたレバーの当てる点（扉の座標・把手が左の時）: 板の中央の面からの前・付け根からの横・レバーの高さからの上。
# シムの grasp_target（11-10 の記録・229 台の中央値。付け根からの横は台で 8.9〜13 cm）
LEVER_REF_OFFSET = (-0.078, -0.105, 0.03)

# ── 段 1 ───────────────────────────────────────────────────
V_MIN = 0.2                  # 並進の指令 [m/s]（0 かこの値。純正は 0.1〜0.15 m/s では動かない）
WZ_MIN, WZ_MAX = 0.3, 0.5    # 旋回の指令 [rad/s]
K_W = 1.2
TURN_DONE_DEG = 5.0          # 回るのを終える向きの残り
AXIS_TOL_M = 0.02            # 体の座標の各軸でこれより近ければその軸は 0
LEG_S = (0.3, 2.0)           # 1 回に並進を出す長さ [s]（最小・最大）
COAST_M = 0.06               # 指令を 0 にした後に流れる分（11-11 の機体で約 8 cm。控えめに引く）
SETTLE_S = 1.5               # 止まって待つ長さ
AVG_S = 0.7                  # その後ろのこの長さの姿勢の中央値で測る
MAX_LEGS = 12
APPROACH_MAX_S = 60.0        # 計画書 9.3（60 秒で入らなければ立ったまま止まる）
NEAR_STOP_X = -0.30          # pelvis が扉の 0.3 m より近ければ止まる
POSE_LOST_S = 0.5

# ── 段 2 ───────────────────────────────────────────────────
HANDOVER_DEG = 3.0           # s2f の complete_door_deg
STAGE2_MAX_S = 15.0
# 腕の力が分からない時の引き金の水平の線 [m]。11-11 の機体: 扉が開いた時、手のひら（手の付け根の点）は当てる点から水平 12〜14 cm
# （前 −10 cm＝手の前のほうで押す・横 +7.5 cm）で、0100 の 10 cm に入らず立たなかった。点群の正解で扉は 3° 開いていた
PRESS_RADIUS_FORCELESS_M = 0.16


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ── 網の扉の値 → pelvis の姿勢 ─────────────────────────────────
def pose_from_frame_loc(loc: Sequence[float], roll: float, pitch: float) -> dfr.PlanarPose:
    """door_frame_loc（9・pelvis から見た開口の中心と扉の +x 軸）→ 扉の座標での pelvis の床の上の姿勢（door_frame_loc の逆）"""
    loc = np.asarray(loc, dtype=float)
    T = dfr.tilt_matrix(roll, pitch)
    v = T @ loc[3:6]  # = Rz(−yaw)·e_x
    yaw = math.atan2(-v[1], v[0])
    w = dfr.rot_z(yaw) @ T @ loc[0:3]  # = 開口の中心 − pelvis（扉の座標）
    return dfr.PlanarPose(float(-w[0]), float(-w[1]), yaw)


def median_pose(poses: Sequence[dfr.PlanarPose]) -> dfr.PlanarPose:
    """位置は中央値、向きは平均の向きのまわりの中央値（±π をまたいでも崩れない）"""
    if not poses:
        raise ValueError("姿勢が 0 個")
    yaw = np.array([p.yaw for p in poses])
    ref = math.atan2(float(np.sin(yaw).mean()), float(np.cos(yaw).mean()))
    dev = np.array([wrap(a - ref) for a in yaw])
    return dfr.PlanarPose(float(np.median([p.x for p in poses])), float(np.median([p.y for p in poses])),
                          wrap(ref + float(np.median(dev))))


def pose_diff(a: dfr.PlanarPose, b: dfr.PlanarPose) -> Tuple[float, float]:
    """(位置の差 [m], 向きの差の大きさ [°])"""
    return math.hypot(b.x - a.x, b.y - a.y), abs(math.degrees(wrap(b.yaw - a.yaw)))


def average_pose(a: dfr.PlanarPose, b: dfr.PlanarPose) -> dfr.PlanarPose:
    return dfr.PlanarPose((a.x + b.x) / 2, (a.y + b.y) / 2, wrap(a.yaw + wrap(b.yaw - a.yaw) / 2))


# ── 段 0 ───────────────────────────────────────────────────
TOE_TO_PELVIS_M = 0.145      # 足先 → pelvis（11-10 訂正・memory tape-pelvis-from-toes）


def pose_from_tape(a_left: float, a_right: float, d_left: float, d_right: float, width: float,
                   thickness: float = 0.04, lever_left: bool = True) -> dfr.PlanarPose:
    """巻き尺 4 つ → 扉の座標での pelvis の姿勢（11-11 の機体: 網の横のずれを外して段 2 を確かめる）。
    a_*: 左右のつま先から扉の板の表までの距離・d_*: 左右のつま先の、把手の側の扉の縁（板の端）から扉に沿った距離 [m]。
    つま先の真ん中から体の向きに TOE_TO_PELVIS_M 後ろを pelvis とする（両足が並んで立っている前提）"""
    edge = (width / 2) if lever_left else -(width / 2)
    side = -1.0 if lever_left else 1.0  # 縁から内へ測る向き
    pl = np.array([-(thickness / 2 + a_left), edge + side * d_left])
    pr = np.array([-(thickness / 2 + a_right), edge + side * d_right])
    lv = pl - pr  # 体の左 = (−sin θ, cos θ)
    yaw = math.atan2(-lv[0], lv[1])
    mid = (pl + pr) / 2
    return dfr.PlanarPose(float(mid[0] - TOE_TO_PELVIS_M * math.cos(yaw)), float(mid[1] - TOE_TO_PELVIS_M * math.sin(yaw)), yaw)


@dataclass(frozen=True)
class Stage0Out:
    cmd: Cmd
    ref: Optional[dfr.PlanarPose]  # この周期に FAST-LIO の口へ送る REF（扉の座標での pelvis の姿勢。走らせ役がライダーの姿勢に直す）
    info: str = ""


class Stage0:
    """段 0 の測りと見張り。step には、その周期に新しく届いた網の値（無ければ None）・FAST-LIO の口で運んだ pelvis の姿勢
    （無ければ None）・FAST-LIO の口の準備ができたか（最後の REF の後の電文が来ているか）を渡す"""

    def __init__(self, tape_pose: Optional[dfr.PlanarPose] = None) -> None:
        self.tape_pose = tape_pose  # 渡すと REF は巻き尺の値・網は比べるだけ（まっすぐ 0.2 m と 2 回目は飛ばす）
        self.phase = "measure1"
        self.t_phase: Optional[float] = None
        self.net: List[dfr.PlanarPose] = []
        self.carried: List[dfr.PlanarPose] = []
        self.first: Optional[dfr.PlanarPose] = None
        self.final: Optional[dfr.PlanarPose] = None
        self.reason = ""
        self.check: Tuple[float, float] = (math.nan, math.nan)

    @property
    def done(self) -> bool:
        return self.phase == "done"

    @property
    def failed(self) -> bool:
        return self.phase == "failed"

    def _go(self, phase: str, t: float) -> None:
        self.phase, self.t_phase = phase, t
        self.net, self.carried = [], []

    def _fail(self, why: str) -> Stage0Out:
        self.phase, self.reason = "failed", why
        return Stage0Out(ZERO, None, why)

    def _where_ok(self, p: dfr.PlanarPose) -> str:
        if not (S0_X_RANGE[0] <= p.x <= S0_X_RANGE[1]) or abs(p.y) > S0_Y_MAX or abs(math.degrees(p.yaw)) > S0_YAW_MAX_DEG:
            return (f"段 0 の立ち位置の外（扉の正面 {-S0_X_RANGE[1]:.1f}〜{-S0_X_RANGE[0]:.1f} m・横 ±{S0_Y_MAX} m・"
                    f"向き ±{S0_YAW_MAX_DEG:.0f}°。網の値 x {p.x:+.2f} y {p.y:+.2f} 向き {math.degrees(p.yaw):+.1f}°）")
        return ""

    def _collected(self, t: float) -> Optional[str]:
        """溜め終わったら ""・まだなら None・網が来なければ失敗の理由"""
        if len(self.net) >= S0_MIN_SAMPLES and t - self.t_phase >= S0_SAMPLE_S:
            return ""
        if t - self.t_phase > S0_NET_WAIT_S and len(self.net) < S0_MIN_SAMPLES:
            return f"網の扉の値が {S0_NET_WAIT_S:.0f} 秒で {len(self.net)} 枚しか来ない（{S0_MIN_SAMPLES} 枚要る）"
        return None

    def step(self, t: float, net_pose: Optional[dfr.PlanarPose], carried: Optional[dfr.PlanarPose], ready: bool) -> Stage0Out:
        if self.phase in ("done", "failed"):
            return Stage0Out(ZERO, None, self.reason)
        if self.t_phase is None:
            self.t_phase = t
        if net_pose is not None and self.phase in ("measure1", "measure2"):
            self.net.append(net_pose)
        if carried is not None and self.phase == "measure2":
            self.carried.append(carried)
        if self.phase == "measure1":
            got = self._collected(t)
            if got is None:
                return Stage0Out(ZERO, None, f"1 回目を測っている（{len(self.net)} 枚）")
            if got:
                return self._fail(got)
            m1 = median_pose(self.net)
            if self.tape_pose is not None:  # 巻き尺の値で REF（位置は巻き尺・向きは網。つま先の差からの向きは 1 cm で 4° 動く）
                tp = self.tape_pose
                self.check = pose_diff(tp, m1)
                print(f"[staged] 段 0: 巻き尺の向き {math.degrees(tp.yaw):+.1f}°・網の向き {math.degrees(m1.yaw):+.1f}° → 網の向きを使う", flush=True)
                tp = dfr.PlanarPose(tp.x, tp.y, m1.yaw)
                self.first = self.final = tp
                self._go("anchor2", t)
                return Stage0Out(ZERO, tp, f"巻き尺の値で REF（網との差 前 {100 * (m1.x - tp.x):+.1f} 横 {100 * (m1.y - tp.y):+.1f} cm・"
                                           f"向き {math.degrees(wrap(m1.yaw - tp.yaw)):+.1f}°）")
            why = self._where_ok(m1)
            if why:
                return self._fail(why)
            self.first = m1
            self._go("anchor1", t)
            return Stage0Out(ZERO, m1, "1 回目を測った: FAST-LIO の口へ REF")
        if self.phase in ("anchor1", "anchor2"):
            if ready:
                nxt = "step" if self.phase == "anchor1" else "done"
                self._go(nxt, t)
                return Stage0Out(ZERO, None, "FAST-LIO の口の段 0 ができた")
            if t - self.t_phase > S0_ANCHOR_WAIT_S:
                return self._fail(f"FAST-LIO の口が {S0_ANCHOR_WAIT_S:.0f} 秒で段 0 を作れない（口のログを見る）")
            return Stage0Out(ZERO, None, "FAST-LIO の口の段 0 を待つ")
        if self.phase == "step":
            if t - self.t_phase < S0_STEP_M / (S0_STEP_VX * STOCK_GAIN[0]):
                return Stage0Out((S0_STEP_VX, 0.0, 0.0), None, "まっすぐ前へ")
            self._go("settle", t)
            return Stage0Out(ZERO, None, "止まる")
        if self.phase == "settle":
            if t - self.t_phase >= S0_SETTLE_S:
                self._go("measure2", t)
            return Stage0Out(ZERO, None, "止まる")
        # measure2
        got = self._collected(t)
        if got is None or (got == "" and len(self.carried) < S0_MIN_SAMPLES):
            if got is None and t - self.t_phase <= S0_NET_WAIT_S:
                return Stage0Out(ZERO, None, f"2 回目を測っている（網 {len(self.net)}・運んだ値 {len(self.carried)}）")
            if t - self.t_phase > S0_NET_WAIT_S:
                return self._fail(f"2 回目: 網 {len(self.net)}・FAST-LIO で運んだ値 {len(self.carried)} しか来ない（{S0_MIN_SAMPLES} 要る）")
            return Stage0Out(ZERO, None, "2 回目を測っている")
        if got:
            return self._fail(got)
        m2, c2 = median_pose(self.net), median_pose(self.carried)
        moved = math.hypot(c2.x - self.first.x, c2.y - self.first.y)
        if moved < S0_STEP_MIN_M:
            return self._fail(f"まっすぐ前へ出ていない（FAST-LIO で {100 * moved:.1f} cm・{100 * S0_STEP_M:.0f} cm のはず。"
                              "純正が指令に応えたか・FAST-LIO の口を見る）")
        self.check = pose_diff(c2, m2)
        if self.check[0] > WATCH_POS_M or self.check[1] > WATCH_YAW_DEG:
            return self._fail(f"2 回の網の測りが食い違った（1 回目を FAST-LIO で運んだ値と 2 回目: {100 * self.check[0]:.1f} cm・"
                              f"{self.check[1]:.1f}°。線 {100 * WATCH_POS_M:.0f} cm・{WATCH_YAW_DEG:.0f}°）")
        self.final = average_pose(m2, c2)
        self._go("anchor2", t)
        return Stage0Out(ZERO, self.final, f"2 回目が合った（{100 * self.check[0]:.1f} cm・{self.check[1]:.1f}°）: 平均で REF し直す")


# ── 構えの位置・レバーの点 ────────────────────────────────────
@dataclass(frozen=True)
class Stance:
    target: dfr.PlanarPose
    tol_x: float
    tol_y: float
    tol_yaw: float  # [rad]

    def contains(self, p: dfr.PlanarPose) -> bool:
        return (abs(p.x - self.target.x) <= self.tol_x and abs(p.y - self.target.y) <= self.tol_y
                and abs(wrap(p.yaw - self.target.yaw)) <= self.tol_yaw)


def stance(width: float, backset: float, lever_left: bool = True) -> Stance:
    lr = 1.0 if lever_left else -1.0
    pivot = lr * (width / 2 - backset)
    return Stance(dfr.PlanarPose(STANCE_X, pivot + lr * STANCE_Y_OFF, lr * math.radians(STANCE_YAW_DEG)),
                  STANCE_TOL[0], STANCE_TOL[1], math.radians(STANCE_TOL[2]))


def lever_ref_point(width: float, backset: float, lever_height: float, lever_left: bool = True) -> np.ndarray:
    """閉じたレバーの当てる点（扉の座標）。巻き尺の寸法 ＋ シムの当てる点のずれ（LEVER_REF_OFFSET）"""
    lr = 1.0 if lever_left else -1.0
    dx, dy, dz = LEVER_REF_OFFSET
    return np.array([dx, lr * (width / 2 - backset + dy), lever_height + dz])


# ── 段 1 ───────────────────────────────────────────────────
class Approach:
    """構えの位置へ寄せる（体の座標の指令 vx, vy, wz を返す）。止まるのは呼ぶ側（failed を見る）。
    11-11 の機体: 歩いている間の姿勢は歩みで ±5〜7 cm 揺れ、指令を 0 にしても 8 cm ほど流れ、0.2〜0.3 m/s を出し続けると
    構えの位置を 16 cm 通り越した。そこで「止まって測る（平均）→ 残りから決めた長さだけ 0.2 m/s を出す → 止まる」を繰り返す。
    旋回だけは続けて出す（向きは揺れが小さく、機体で −45° に 3 秒で入った）"""

    def __init__(self, st: Stance) -> None:
        self.st = st
        self.phase = "settle"
        self.t0: Optional[float] = None
        self.t_phase = 0.0
        self.samples: List[Tuple[float, dfr.PlanarPose]] = []
        self.leg_until = (0.0, 0.0)
        self.leg_cmd: Cmd = ZERO
        self.legs = 0
        self.last_pose_t: Optional[float] = None
        self.reason = ""

    @property
    def done(self) -> bool:
        return self.phase == "done"

    @property
    def failed(self) -> bool:
        return self.phase == "failed"

    def _fail(self, why: str) -> Cmd:
        self.phase, self.reason = "failed", why
        return ZERO

    def _settle(self, t: float) -> Cmd:
        self.phase, self.t_phase, self.samples = "settle", t, []
        return ZERO

    def step(self, t: float, pose: Optional[dfr.PlanarPose]) -> Cmd:
        if self.phase in ("done", "failed"):
            return ZERO
        if self.t0 is None:
            self.t0 = self.t_phase = t
        if t - self.t0 > APPROACH_MAX_S:
            return self._fail(f"{APPROACH_MAX_S:.0f} 秒で構えの位置に入らない")
        if pose is None:
            since = self.last_pose_t if self.last_pose_t is not None else self.t0  # 初めから来ない時も同じ線
            if t - since > POSE_LOST_S:
                return self._fail(f"位置が {POSE_LOST_S} 秒来ない")
            return ZERO
        self.last_pose_t = t
        if pose.x > NEAR_STOP_X:
            return self._fail(f"pelvis が扉に近すぎる（x {pose.x:+.2f} m・線 {NEAR_STOP_X} m）")
        e_yaw = wrap(self.st.target.yaw - pose.yaw)
        if self.phase == "turn":
            if abs(e_yaw) <= math.radians(TURN_DONE_DEG):
                return self._settle(t)
            return (0.0, 0.0, math.copysign(min(WZ_MAX, max(WZ_MIN, K_W * abs(e_yaw))), e_yaw))
        if self.phase == "leg":
            dt = t - self.t_phase
            if dt >= max(self.leg_until):
                return self._settle(t)
            return (self.leg_cmd[0] if dt < self.leg_until[0] else 0.0, self.leg_cmd[1] if dt < self.leg_until[1] else 0.0, 0.0)
        # settle: 止まって待ち、後ろの AVG_S の平均で測る
        self.samples.append((t, pose))
        if t - self.t_phase < SETTLE_S:
            return ZERO
        avg = median_pose([p for ts, p in self.samples if ts >= t - AVG_S])
        if self.st.contains(avg):
            self.phase = "done"
            return ZERO
        e_yaw = wrap(self.st.target.yaw - avg.yaw)
        if abs(e_yaw) > math.radians(TURN_DONE_DEG):
            self.phase, self.t_phase = "turn", t
            return ZERO
        c, s = math.cos(avg.yaw), math.sin(avg.yaw)
        dx, dy = self.st.target.x - avg.x, self.st.target.y - avg.y
        body = (c * dx + s * dy, -s * dx + c * dy)  # 体の座標の残り（前・左）
        self.legs += 1
        if self.legs > MAX_LEGS:
            return self._fail(f"{MAX_LEGS} 回出しても構えの位置に入らない（残り 前 {body[0]:+.3f}・左 {body[1]:+.3f} m）")
        durs, cmd = [], []
        for d, g in zip(body, STOCK_GAIN[:2]):
            if abs(d) <= AXIS_TOL_M:
                durs.append(0.0)
                cmd.append(0.0)
            else:  # 流れる分（COAST_M）を引いた長さ。短すぎる時は最小の長さ
                durs.append(min(LEG_S[1], max(LEG_S[0], (abs(d) - COAST_M) / (V_MIN * g))))
                cmd.append(math.copysign(V_MIN, d))
        self.phase, self.t_phase = "leg", t
        self.leg_cmd, self.leg_until = (cmd[0], cmd[1], 0.0), (durs[0], durs[1])
        return (cmd[0] if durs[0] > 0 else 0.0, cmd[1] if durs[1] > 0 else 0.0, 0.0)


# ── 段 2 ───────────────────────────────────────────────────
@dataclass(frozen=True)
class ArmOut:
    legs_free: bool
    angle_obs: np.ndarray  # door_angle_lidar の形
    handover: bool
    failed: bool


class ArmStage:
    """段 2: 脚を放す引き金（0100 と同じ press の線）・手のひらで扉の角度・3° で引き継ぐ。
    force_known=False（腕のトルクから力を出す校正 L2c の前）は力の線を 0 にし、位置と下がりだけで決める
    （角度の基準も press にする: 力の無い contact は近づいただけで立つ）。さらに、手のひらが一度レバーの当てる点の近くで
    当てる点の高さ以上に来てから数え始める（11-10 L4 の輪の試験: 学習の初めの腕の姿勢では、構えの位置で手のひらが
    当てる点から水平 8.8 cm・17.5 cm 下にあり、押していないのに引き金が立った。s2f は棒の上から押す）"""

    def __init__(self, width: float, backset: float, lever_height: float, lever_left: bool = True,
                 force_known: bool = True, press_radius: Optional[float] = None, max_s: float = STAGE2_MAX_S,
                 release_legs: bool = True) -> None:
        self.lever_ref = lever_ref_point(width, backset, lever_height, lever_left)
        self.force_known, self.max_s = force_known, max_s
        self.release_legs = release_legs  # False: 段 2 の間は脚を放さない（11-11 の機体: 脚を止めたままで扉が 3° 開いた回がある）
        min_force = hda.PRESS_FORCE_N if force_known else 0.0
        self.radius = press_radius if press_radius is not None else (hda.PRESS_RADIUS_M if force_known else PRESS_RADIUS_FORCELESS_M)
        self.release = hda.PressTrigger(radius=self.radius, min_force=min_force)
        self.release.set_ref(self.lever_ref)
        self.est = hda.HandDoorAngle(hda.door_line(width, lever_left), ref_mode="contact" if force_known else "press",
                                     radius=self.radius, min_force=min_force)
        self.est.set_lever_ref(self.lever_ref)
        self.t0: Optional[float] = None
        self.armed = force_known  # 力が分かれば初めから数える。分からなければ「上から来た」のを見てから
        self.legs_free = False
        self.handover = False
        self.failed = False
        self.reason = ""

    def step(self, t: float, palm: np.ndarray, force: Optional[float]) -> ArmOut:
        if self.t0 is None:
            self.t0 = t
        f = float(force) if (self.force_known and force is not None) else 0.0
        if self.force_known and force is None:
            f = math.nan  # 力が来ない歩は引き金を数えない
        p = np.asarray(palm, dtype=float)
        if not self.armed and np.all(np.isfinite(p)):
            near = math.hypot(p[0] - self.lever_ref[0], p[1] - self.lever_ref[1]) <= self.radius
            self.armed = near and p[2] >= self.lever_ref[2]
        if not self.armed:
            f = math.nan
        self.legs_free = (self.release.step(palm, f) or self.legs_free) and self.release_legs
        obs = self.est.step(t, palm, f)
        if not self.handover and obs[1] > 0.5 and math.degrees(obs[0]) >= HANDOVER_DEG:
            self.handover = True
        if not self.handover and not self.failed and t - self.t0 > self.max_s:
            self.failed, self.reason = True, (f"段 2 が {self.max_s:.1f} 秒で扉を {HANDOVER_DEG:.0f}° 開けられない（最大 {self.est.max_deg:.1f}°・"
                                              f"脚を{'放した' if self.legs_free else '止めたまま'}）")
        return ArmOut(self.legs_free, obs, self.handover, self.failed)


# ── 持ち替え・運び ────────────────────────────────────────────
def handover_state(s, hidden: str = "zero"):
    """s2f → f3t3 の持ち替え（door_core.CoreState）。積み上げ D・前の行動・HOMIE の履歴は持ち越し、記憶だけ 0。
    shadow（f3t3 の記憶を段 2 の間も影で回す）は走らせ役が f3t3 の状態を別に持つ形になるのでここでは扱わない
    （11-10 の 1 回目の評価で zero と差が無かった）"""
    if hidden != "zero":
        raise ValueError(f"持ち替えの記憶は zero だけ: {hidden}")
    return replace(s, h=np.zeros_like(s.h), cell=np.zeros_like(s.cell))


def zero_walk_accumulator(s):
    """段 2 の脚を放す前: 歩きの積み上げ D[0:3] を 0 に止める（シムの arm_stage の脚を止める台と同じ）"""
    D = np.array(s.D, dtype=float, copy=True)
    D[:3] = 0.0
    return replace(s, D=D)


class OdomCarry:
    """段 3〜4: 引き継いだ時の扉の座標の姿勢を /dog_odom の動きで運ぶ"""

    def __init__(self, start: dfr.PlanarPose, odom_start: dfr.PlanarPose) -> None:
        self.start, self.odom_start = start, odom_start

    def pose(self, odom_now: dfr.PlanarPose) -> dfr.PlanarPose:
        return dfr.compose_planar(self.start, self.odom_start, odom_now)
