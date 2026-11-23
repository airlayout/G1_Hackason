"""11-10 手順 L4（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.3・9.10）: 段に分けた走らせ役の差し込み口。
door_run_stock.py の DOOR の 1 周期を --staged の時に任される（止める判定・腕の送り・記録は door_run_stock.py のまま）。
段の中身は door_staged.py（numpy だけ）。ここはそれを FAST-LIO の受け口（door_lidar_source.LidarDoorSource・
door_fastlio_node.py が送る）・目の網（c2 の特徴 → 扉の頭）・左腕の順運動学につなぎ、毎周期の決め事（Decision）を返す。

  段 0  hold・腕は下ろしたまま（"home"）・網で 2 回測る・FAST-LIO の口へ REF2・まっすぐ 0.2 m
  段 1  hold・腕は下ろしたまま・構えの位置へ寄る（手で書いた指令）。FAST-LIO と odom の食い違い（受け口の見張り）で止まる
  段 2  先に腕を学習の初めの姿勢へ（"goal"・ARM_PREP_S）→ 腕だけの方策（"arm"＝s2f）。脚を放すまでは歩きの積み上げを 0 に止め、
        歩かない。扉の角度は手のひらから。3° で引き継ぐ
  段 3  教師 f3t3（"f3t3"）。状態は持ち替え（記憶だけ 0）。枠は引き継いだ時の値を /dog_odom で運ぶ
⚠️ 段 1 の指令の線・倍率、pelvis の高さ（--pelvis-z）、レバーの当てる点のずれは L5 のシムと L7 の機体で確かめる。
⚠️ 腕の力（L2c）はまだ無いので、既定では力の線を 0 にして位置と下がりだけで脚を放す（force_known=False）

11-12 台本（docs/plan/2026-11-12-g1-door-scripted-stage123.md）: script を渡すと、段 2・3 は学習した方策でなく台本
（door_script.ScriptDoor）が進める。腕は "script"（逆運動学で出した左腕 7 軸の目標）、脚は台本の指令、方策は回さない（"hold"）。
script_start=1 は段 0 を飛ばし、前の回の REF（FAST-LIO の口が持っている扉の座標）のまま段 1 の寄りから始める。
11-12 夜: 同じ走らせ役で段 0 を一度通したら、次の d からは段 0 を自動で飛ばす（d0 で段 0 からやり直す）。
d のたびに台本の定数を読み直す（reload_script。門番 script_gate.py を通った定数だけ）。止まった時は手を扉から離す（ScriptArm.release）
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

import door_frame as dfr
import door_lidar_msg as dlm
import door_script as dscr
import g1_arm_fk as fk
import g1_arm_ik as aik
import hand_force as hfm
import door_staged as ds
import hand_door_angle as hda

Walk = Tuple[bool, float, float, float]
STAND: Walk = (False, 0.0, 0.0, 0.0)
ARM_PREP_S = 2.5        # 段 2 の前に腕を学習の初めの姿勢へ寄せる時間（door_run_stock の ARM_SPEED 1 rad/s で足りる）
ODOM_STALE_S = 0.1      # これより古い odom は使わない（door_run_real.ODOM_STALE_S と同じ）
ODOM_LOST_S = 0.5       # 段 3 で odom がこれだけ来なければ止まる
POSE_LOST_S = 0.5       # 段 2 で FAST-LIO の姿勢がこれだけ来なければ止まる
# 11-13 実機: 段 0 を飛ばした段 1 で、正しい値（x -0.43）の次の 1 回だけ x +9.55 m に跳んで止まった（出どころは未切り分け）
POSE_JUMP_M = 0.3       # 直前に受け入れた姿勢からこれ以上跳んだ値は捨てる（50 Hz の 1 周期で歩きは 1 cm 未満）
POSE_JUMP_DEG = 20.0
POSE_JUMP_WINDOW_S = POSE_LOST_S + 0.1  # 直前の姿勢がこれより古ければ比べない（跳びが続けば先に POSE_LOST_S で止まる）
DEFAULT_PELVIS_Z = 0.75  # [m] 立った pelvis の床からの高さ（⚠️ L7 で巻き尺で測る。レバーを下げたかの判定の高さに効く）
WAIST = slice(12, 15)
SCRIPT_WARM_S = 0.6     # script_start=1: 受け口に関節・odom の履歴が溜まるまで待つ（溜まる前は FAST-LIO の姿勢を今へ運べない）
SCRIPT_Q_LEAD = 0.3     # 逆運動学の内の腕の角度が実の角度より先へ行ける幅 [rad]（当たっている間に溜め込まない）
# 11-13 把手と反対の手を添える: 腕 7 軸（肩 pitch・roll・yaw・肘・手首 roll・pitch・yaw）の左右の鏡写しの符号
MIRROR7 = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
HELPER_DIR = np.array([1.0, 0.0, 0.0])  # 添えた手の向き（扉の座標・板へまっすぐ）
HELPER_SUBS = ("2a", "2b", "2c", "2d", "3a", "3c")
DT = dscr.DT


@dataclass(frozen=True)
class Decision:
    stage: int
    policy: str                    # "arm"（s2f）か "f3t3"
    mode: str                      # door_core.step の mode（"hold" / "door"）
    pose: dfr.PlanarPose           # 扉の座標での pelvis の姿勢（door_frame_loc を作る）
    angle: np.ndarray              # door_angle_lidar（3）
    walk: Optional[Walk]           # None なら方策の指令（door_stock_io.stock_velocity）で歩く
    arm: str                       # 腕の目標の出どころ: "home"（借りる前の角度）・"goal"（学習の初めの姿勢へ寄せる）・"policy"・"script"
    zero_walk: bool = False        # 方策の後に歩きの積み上げを 0 に止める（door_staged.zero_walk_accumulator）
    handover_now: bool = False     # この周期に f3t3 へ持ち替える（door_staged.handover_state）
    stop: str = ""                 # 空でなければ立ったまま止まる理由
    info: str = ""
    arm_left: Optional[np.ndarray] = None   # arm="script": 左腕 7 軸の目標（None は今の目標のまま）
    arm_other: Optional[np.ndarray] = None  # arm="script": 把手と反対の手 7 軸の目標（11-13。None は借りる前の角度）
    waist_comp: bool = False                # arm="script": 腰 yaw の打ち消しを頼む
    waist_twist: float = 0.0                # arm="script": 腰 yaw に足す角度 [rad]（11-13 半身で押す）
    waist_lean: float = 0.0                 # arm="script": 腰 pitch に足す角度 [rad]（11-13 前へ傾ける）
    script_row: Optional[np.ndarray] = None  # 記録の 1 行（door_script.ScriptDoor.row）


ARM_SOURCES = ("home", "goal", "policy", "script")  # 記録の staged 列の番号


class ScriptArm:
    """台本（door_script.ScriptDoor）を機体につなぐ: pelvis の姿勢をなまし、扉の座標の手のひらを出し、台本の行き先を
    逆運動学（g1_arm_ik.ArmIK）で左腕 7 軸の目標にする"""

    def __init__(self, script: dscr.ScriptDoor, ik, palm_fn: Callable, pelvis_z: float) -> None:
        self.script, self.ik, self.palm_fn, self.pelvis_z = script, ik, palm_fn, pelvis_z
        self.filt = dscr.PoseFilter(script.p.pose_tau_s)
        self.posture = np.asarray(script.p.posture, dtype=float)
        self.q_cmd: Optional[np.ndarray] = None
        self.last_pose: Optional[dfr.PlanarPose] = None   # なました姿勢（止まった時に手を離す向きに使う）
        self.release_goal: Optional[np.ndarray] = None    # 止まった時の手の行き先（扉の座標）
        self.release_until: Optional[float] = None
        # 11-13: 把手と反対の手を板に添える（script.p.helper）。arm_other は毎周期の 7 軸の目標（None は借りる前の角度のまま）
        self.helper = aik.ArmIK(ik.chain, "right" if ik.side == "left" else "left", getattr(ik, "urdf", None)) if script.p.helper else None
        self.h_posture = self.posture * MIRROR7
        self.h_q_cmd: Optional[np.ndarray] = None
        self.h_goal: Optional[np.ndarray] = None          # 扉の座標の今の行き先（寄せながら動かす）
        self.h_body: Optional[np.ndarray] = None          # 3c: pelvis の座標に固めた行き先
        self.h_t: Optional[float] = None
        self.h_release_goal: Optional[np.ndarray] = None
        self.h_release_until = 0.0
        self.arm_other: Optional[np.ndarray] = None
        # 11-13: 手が扉・レバーから受ける力の見積もり（腕の関節の力から。扉の座標・N）。2b の間（触れていない）に基準を取る
        try:
            self.hf: Optional[hfm.HandForce] = hfm.HandForce(ik.chain, getattr(ik, "urdf", None) or fk.default_urdf(), ik.side,
                                                             tip_m=hfm.TIP_M)  # 台本は手の先で押す
        except (OSError, ValueError, KeyError) as e:
            print(f"[staged] ⚠️ 手の力の見積もりは使えない（{type(e).__name__}: {e}）", flush=True)
            self.hf = None
        self.hand_f = np.full(3, np.nan)
        self.hf_tick = 0

    def _frame(self, pose: dfr.PlanarPose, quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
        return dfr.rot_z(pose.yaw) @ dfr.tilt_matrix(roll, pitch), np.array([pose.x, pose.y, self.pelvis_z])

    def step(self, now: float, q: np.ndarray, quat: np.ndarray, pose: dfr.PlanarPose,
             tau: Optional[np.ndarray], door_angle: Optional[float] = None) -> Tuple[dscr.ScriptOut, Optional[np.ndarray]]:
        """door_angle [rad]: 外で測った扉の角度（台本の angle_src_3c "ext" の時に 3c・4a で使う。11-16）"""
        fpose = self.filt.step(pose)
        self.last_pose = fpose
        side = "left" if self.ik.side == "left" else "right"
        palm_b = fk.palm_in_pelvis(self.ik.chain, q, side)  # 順運動学は 1 周期に 1 回（11-12 夜）
        R, T = self._frame(fpose, quat)
        palm = T + R @ palm_b
        Rr, Tr = self._frame(pose, quat)
        palm_raw = Tr + Rr @ palm_b                          # 扉の角度は、なましていない姿勢で読む
        fwd = float(palm_b[0])
        q_act = np.asarray(q, dtype=float)[self.ik.idx]
        tau_pitch = float(tau[self.ik.idx[0]]) if tau is not None else math.nan  # 把手の側の肩 pitch
        self._update_hand_force(q, quat, tau, Rr)  # 前の周期の小段で見る（2b は基準・2c から先は力）
        out = self.script.step(now, fpose, palm, tau_pitch, fwd, palm_angle=palm_raw, door_angle_ext=door_angle)
        self.arm_other = self._helper_step(now, q, out.sub, R, T) if self.helper is not None else None
        if out.goal is None or out.stop:
            return out, None
        return out, self._ik_to(q, q_act, out.goal, R, T, out.direction)

    def _update_hand_force(self, q: np.ndarray, quat: np.ndarray, tau: Optional[np.ndarray], R: np.ndarray) -> None:
        """手の力の見積もりを 25 Hz で進める（計算は 1 回 1 ms 前後。周期の半分に間引く）。値は self.hand_f（扉の座標）"""
        if self.hf is None or tau is None:
            return
        self.hf_tick += 1
        if self.hf_tick % 2:
            return
        sub = self.script.sub
        if sub in ("2a", "2b"):
            self.hand_f = np.full(3, np.nan)
            if sub == "2b":
                self.hf.calibrate(q, quat, tau)
        elif sub in ("2c", "2d", "3a", "3c", "4a", "done"):
            F = self.hf.force(q, quat, tau)
            self.hand_f = R @ F if np.isfinite(F).all() else np.full(3, np.nan)

    # --- 把手と反対の手（11-13） ---
    def _helper_palm(self, q: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
        return T + R @ fk.palm_in_pelvis(self.helper.chain, q, self.helper.side)

    def _helper_ik(self, q: np.ndarray, goal: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
        q_act = np.asarray(q, dtype=float)[self.helper.idx]
        if self.h_q_cmd is None:
            self.h_q_cmd = q_act.copy()
        q_cmd = np.clip(self.h_q_cmd, q_act - SCRIPT_Q_LEAD, q_act + SCRIPT_Q_LEAD)
        self.h_q_cmd = self.helper.step(q, q_cmd, R.T @ (goal - T), self.h_posture, R.T @ HELPER_DIR)
        return self.h_q_cmd.copy()

    def _helper_step(self, now: float, q: np.ndarray, sub: str, R: np.ndarray, T: np.ndarray) -> Optional[np.ndarray]:
        """把手と反対の手の 7 軸の目標。helper_from より前は None（借りる前の角度のまま）。
        構え（helper_from〜2d・retreat）: 板の 8 cm 手前 → 3a: 板へ押し当てる → 3c: pelvis に対して固める（歩いて押す）"""
        p = self.script.p
        dt = DT if self.h_t is None else min(max(now - self.h_t, 0.0), 0.1)
        self.h_t = now
        active = HELPER_SUBS.index(sub) >= HELPER_SUBS.index(p.helper_from) if sub in HELPER_SUBS else self.h_goal is not None
        if not active:
            return None
        if sub in ("done", "failed"):  # 走らせ役が POST へ移る周期。今の目標のまま（POST で手を引く）
            return self.h_q_cmd.copy() if self.h_q_cmd is not None else None
        if self.h_goal is None:
            self.h_goal = self._helper_palm(q, R, T)
        y = float(self.script.ref[1]) - self.script.lr * p.helper_dy
        if sub in ("3c", "4a"):  # 11-14: 4a（段 4 の入り）も 3c と同じく体に固める
            if self.h_body is None:  # 3c に入った周期: 今の行き先を pelvis の座標に固め、push_press（helper_push_press）だけ前へ
                press = p.helper_push_press if p.helper_push_press >= 0.0 else p.push_press
                self.h_body = R.T @ (self.h_goal - T) + np.array([press, 0.0, 0.0])
            self.h_goal = T + R @ self._helper_line_body(R)
        else:
            self.h_body = None
            x, speed = (p.helper_x_press, p.helper_press_speed) if sub == "3a" else (p.helper_x_ready, p.helper_speed)
            self.h_goal = dscr._toward(self.h_goal, np.array([x, y, p.helper_z]), speed * dt)
        return self._helper_ik(q, self.h_goal, R, T)

    def _helper_line_body(self, R: np.ndarray) -> np.ndarray:
        """3c・4a の添えた手の行き先（pelvis の座標）。helper_line なら横を 前への出 × tan(φ) に置く。
        φ ＝ −lr × 扉の角度 − 体の向き（扉の座標）＝ pelvis の座標で見た「板の法線（手から体へ）」の逆の向き。
        手がその向きの線の上にあれば、板の押し返し（法線の向き）の力の線は pelvis を通り、体を回さない（11-16 計画 5.2）"""
        p = self.script.p
        if not p.helper_line:
            return self.h_body
        th = math.radians(min(max(self.script.angle_median_deg(), 0.0), p.helper_line_max_deg))
        yaw = math.atan2(R[1, 0], R[0, 0])
        phi = -self.script.lr * th - yaw
        return np.array([self.h_body[0], self.h_body[0] * math.tan(phi), self.h_body[2]])

    def begin_release_other(self, now: float, q: np.ndarray, quat: np.ndarray) -> bool:
        """止まった時（POST）に、添えた手を扉の座標で後ろへ helper_back 引き始める（構えていなければ何もしない）"""
        if self.helper is None or self.h_q_cmd is None or self.last_pose is None:
            return False
        R, T = self._frame(self.last_pose, quat)
        palm = self._helper_palm(q, R, T)
        if not np.isfinite(palm).all():
            return False
        self.h_release_goal = palm - np.array([self.script.p.helper_back, 0.0, 0.0])
        self.h_release_until = now + self.script.p.release_s
        self.h_q_cmd, self.h_goal, self.h_body = None, None, None
        return True

    def release_other(self, now: float, q: np.ndarray, quat: np.ndarray) -> Optional[np.ndarray]:
        """添えた手を引く間の 7 軸の目標（引き終えたら None＝その場で保つ）"""
        if self.h_release_goal is None or now > self.h_release_until or self.last_pose is None:
            return None
        R, T = self._frame(self.last_pose, quat)
        return self._helper_ik(q, self.h_release_goal, R, T)

    def _ik_to(self, q: np.ndarray, q_act: np.ndarray, goal: np.ndarray, R: np.ndarray, T: np.ndarray,
               direction_door: Optional[np.ndarray]) -> np.ndarray:
        if self.q_cmd is None:
            self.q_cmd = q_act.copy()
        target = R.T @ (np.asarray(goal, dtype=float) - T)
        q_cmd = np.clip(self.q_cmd, q_act - SCRIPT_Q_LEAD, q_act + SCRIPT_Q_LEAD)
        direction = R.T @ direction_door if direction_door is not None else None
        self.q_cmd = self.ik.step(q, q_cmd, target, self.posture, direction)
        return self.q_cmd.copy()

    def begin_release(self, now: float, q: np.ndarray, quat: np.ndarray) -> bool:
        """止まった時（POST）に手を扉から離し始める。手が扉・レバーに触れていない時は何もしない（False）"""
        if not self.script.touching() or self.last_pose is None:
            return False
        R, T = self._frame(self.last_pose, quat)
        palm = T + R @ fk.palm_in_pelvis(self.ik.chain, q, "left" if self.ik.side == "left" else "right")
        if not np.isfinite(palm).all():
            return False
        self.release_goal = self.script.release_goal(palm)
        self.release_until = now + self.script.p.release_s
        self.q_cmd = None
        return True

    def release(self, now: float, q: np.ndarray, quat: np.ndarray) -> Optional[np.ndarray]:
        """手を離す間の左腕 7 軸の目標（離し終えたら None＝その場で保つ）。体は止まっている前提で、なました姿勢のまま"""
        if self.release_until is None or now > self.release_until or self.last_pose is None:
            return None
        R, T = self._frame(self.last_pose, quat)
        q_act = np.asarray(q, dtype=float)[self.ik.idx]
        return self._ik_to(q, q_act, self.release_goal, R, T, None)


class NetSampler:
    """目の網（c2 の特徴 → 扉の頭）の新しい 1 枚ごとに、扉の座標での pelvis の姿勢を返す（段 0 の測り）"""

    def __init__(self, p, vp, feats, head_pitch: float) -> None:
        self.p, self.vp, self.feats, self.head_pitch = p, vp, feats, head_pitch
        self.last_seq = None

    def sample(self, now: float, q: np.ndarray, quat: np.ndarray, obs: Optional[np.ndarray]) -> Optional[dfr.PlanarPose]:
        import door_vision_core as dvc
        if obs is None:
            return None
        m = self.feats.latest()
        if m is None or m[0] == self.last_seq or self.feats.why_not(now):
            return None
        self.last_seq = m[0]
        hg = dvc.mirror_head_gravity(self.vp, dvc.head_gravity(quat, q[WAIST], self.head_pitch), self.p.mirror)
        x = dvc.insert_door(self.vp, obs, dvc.door_values(self.vp, obs, m[1], hg))
        if self.p.mirror:  # 鏡写しは 2 回で戻る
            x = self.p.obs_mirror_M @ x + self.p.obs_mirror_b
        f, g = self.vp.door_slices["door_frame_loc"]
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
        return ds.pose_from_frame_loc(x[f:g], roll, pitch)


def make_palm_fn(chain, pelvis_z: float, lever_left: bool) -> Callable:
    """扉の座標での手のひら（把手の側の手）＝ 扉の座標の床の姿勢 ∘ IMU の傾き ∘ 順運動学"""
    side = "left" if lever_left else "right"
    return lambda q, pose, quat: hda.palm_in_door_frame(chain, q, pose, quat, pelvis_z, side)


class StagedDoor:
    """段 0〜3 の決め事。lidar は LidarDoorSource（request_ref・note_state・ready・pose・head_xy）、net は NetSampler、
    palm_fn(q, pose, quat) は扉の座標の手のひら"""

    def __init__(self, width: float, height: float, lever_height: float, backset: float, lever_left: bool,
                 lidar, net, palm_fn: Callable, force_known: bool = False, net_dx: float = 0.0,
                 press_radius: Optional[float] = None, stage2_max_s: float = ds.STAGE2_MAX_S,
                 release_legs: bool = True, tape_pose: Optional[dfr.PlanarPose] = None,
                 script: Optional[dscr.ScriptParams] = None, script_start: int = 0, ik=None,
                 pelvis_z: float = DEFAULT_PELVIS_Z, script_path: Optional[str] = None,
                 script_stop: Optional[str] = None, script_check: Optional[Callable[[], str]] = None) -> None:
        if script is not None and ik is None:
            raise ValueError("台本（script）には逆運動学（ik）が要る")
        if script_start not in (0, 1) or (script_start and script is None):
            raise ValueError(f"script_start は 0 か 1（1 は台本の時だけ）: {script_start}")
        self.script_p, self.script_start, self.ik, self.pelvis_z = script, script_start, ik, pelvis_z
        self.script_path, self.script_stop, self.script_check = script_path, script_stop, script_check
        self.ref_done = False  # この走らせ役で段 0 を一度通した（次の d からは段 0 を飛ばす）
        self.script: Optional[ScriptArm] = None
        self.warm_until: Optional[float] = None
        self.tape_pose = tape_pose
        self.width, self.height, self.lever_height, self.backset = width, height, lever_height, backset
        self.lever_left, self.force_known = lever_left, force_known
        self.lidar, self.net, self.palm_fn = lidar, net, palm_fn
        self.press_radius, self.stage2_max_s, self.release_legs = press_radius, stage2_max_s, release_legs
        self.net_dx = net_dx  # 網の pelvis の前後の値に足す補正 [m]（巻き尺との片寄り。11-11 の機体で網は扉を 4〜5 cm 遠くに出した）
        self.stance = (dscr.script_stance(script, width, backset, lever_height, lever_left) if script is not None
                       else ds.stance(width, backset, lever_left))
        self.stage = -1
        self.stage0: Optional[ds.Stage0] = None
        self.approach: Optional[ds.Approach] = None
        self.arm: Optional[ds.ArmStage] = None
        self.carry: Optional[ds.OdomCarry] = None
        self.prep_until: Optional[float] = None
        self.last_pose = dfr.PlanarPose(-1.0, 0.0, 0.0)
        self.last_pose_t: Optional[float] = None
        self.jumped = False  # 直前の周期に跳んだ姿勢を捨てたか（表示を 1 度だけにする）
        self.last_odom_t: Optional[float] = None
        self.handover_wait_t: Optional[float] = None

    # --- 段の入り口 ---
    def reload_script(self) -> str:
        """d の前に台本の定数を読み直す。返り値は受け付けない理由（空なら読み直した）。
        門番（script_check）が「通っていない」と言えば、古い定数でも走らせない"""
        if self.script_p is None or not self.script_path:
            return ""
        if self.script_check is not None:
            why = self.script_check()
            if why:
                return f"台本の定数: {why}（Mac で push_params.sh、か PC2 で python3 script_gate.py {self.script_path} --door ... を回す）"
        try:
            p = dscr.load_params(self.script_path, stop_after=self.script_stop)
        except (OSError, ValueError) as e:
            return f"台本の定数を読めない: {type(e).__name__}: {e}"
        changed = p != self.script_p
        self.script_p = p
        self.stance = dscr.script_stance(p, self.width, self.backset, self.lever_height, self.lever_left)
        if changed:
            print(f"[staged] 台本の定数を読み直した（{self.script_path}・止める所 {p.stop_after or '無し'}・段 3 {p.push_mode}"
                  + (f"・{p.note}" if p.note else "") + "）", flush=True)
        return ""

    def begin(self, now: float, force_stage0: bool = False) -> None:
        self.stage, self.stage0 = 0, ds.Stage0(self.tape_pose)
        self.approach = self.arm = self.carry = None
        self.prep_until = self.handover_wait_t = None
        self.script, self.warm_until = None, None
        skip0 = self.script_p is not None and (self.script_start >= 1 or self.ref_done) and not force_stage0
        if skip0:  # 段 0 を飛ばす: 前の回の REF のまま、今の FAST-LIO の姿勢から寄り直す
            self.stage, self.approach, self.warm_until = 1, ds.Approach(self.stance), now + SCRIPT_WARM_S
            t = self.stance.target
            print(f"[staged] 台本: 段 0 を飛ばして段 1 から（前の REF のまま）。構えの位置 x {t.x:+.3f} y {t.y:+.3f} 向き "
                  f"{math.degrees(t.yaw):+.0f}°", flush=True)

    def _enter_script(self, now: float) -> None:
        self.stage, self.prep_until = 2, None
        sd = dscr.ScriptDoor(self.script_p, self.width, self.backset, self.lever_height, self.lever_left)
        self.script = ScriptArm(sd, self.ik, self.palm_fn, self.pelvis_z)
        p = self.last_pose
        print(f"[staged] 段 1 が終わった（x {p.x:+.3f} y {p.y:+.3f} 向き {math.degrees(p.yaw):+.1f}°）→ 台本の段 2"
              f"（レバーの当てる点 {np.round(sd.ref, 3).tolist()}・止める所 {self.script_p.stop_after or '無し'}）", flush=True)

    def _enter_stage1(self, now: float) -> None:
        self.ref_done = True
        self.stage, self.approach = 1, ds.Approach(self.stance)
        t = self.stance.target
        print(f"[staged] 段 0 が終わった → 段 1: 構えの位置 x {t.x:+.3f} y {t.y:+.3f} 向き {math.degrees(t.yaw):+.0f}° へ寄る", flush=True)

    def _enter_arm_prep(self, now: float) -> None:
        self.stage, self.prep_until = 2, now + ARM_PREP_S
        p = self.last_pose
        print(f"[staged] 段 1 が終わった（x {p.x:+.3f} y {p.y:+.3f} 向き {math.degrees(p.yaw):+.1f}°）→ 段 2: 腕を {ARM_PREP_S} 秒で構える", flush=True)

    def _enter_stage2(self, now: float) -> None:
        self.stage, self.prep_until = 2, None
        self.arm = ds.ArmStage(self.width, self.backset, self.lever_height, self.lever_left, force_known=self.force_known,
                               press_radius=self.press_radius, max_s=self.stage2_max_s, release_legs=self.release_legs)
        print(f"[staged] 段 2: 腕だけの方策（レバーの当てる点 {np.round(self.arm.lever_ref, 3).tolist()}・力は"
              f"{'使う' if self.force_known else '使わない（上から来てから位置と下がりで）'}）", flush=True)

    def _enter_stage3(self, now: float, pose: dfr.PlanarPose, odom) -> None:
        self.stage = 3
        self.carry = ds.OdomCarry(pose, dfr.PlanarPose(*odom))
        self.last_odom_t = now

    # --- 毎周期 ---
    def decide(self, now: float, q: np.ndarray, quat: np.ndarray, odom, odom_t, obs: Optional[np.ndarray],
               tau: Optional[np.ndarray] = None) -> Decision:
        closed = dfr.door_closed_angle_obs()
        fresh = odom if (odom is not None and odom_t is not None and now - odom_t < ODOM_STALE_S) else None
        q_waist = tuple(float(v) for v in q[WAIST])
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
        if self.stage < 3:
            self.lidar.note_state(now, fresh, q_waist, roll, pitch)
        if self.stage == 0:
            return self._stage0(now, q, quat, q_waist, roll, pitch, fresh, obs, closed)
        if self.stage == 1 and self.warm_until is not None:
            if now < self.warm_until:
                return Decision(1, "arm", "hold", self.last_pose, closed, STAND, "home", info="受け口の履歴を溜めている")
            self.warm_until = None
            if not self.lidar.ready(now):
                return self._stop(1, "段 1: FAST-LIO の口から受け入れた姿勢が来ない（REF が無いなら段 0 から始める）", closed)
        if self.stage == 1:
            lpose, why = self._lidar_pose(now, fresh)
            if why:
                return self._stop(1, "段 1: " + why, closed)
            cmd = self.approach.step(now, lpose)
            self._keep(now, lpose)
            if self.approach.failed:
                return self._stop(1, "段 1: " + self.approach.reason, closed)
            if self.approach.done and self.script_p is not None:
                self._enter_script(now)
                return Decision(2, "arm", "hold", self.last_pose, closed, STAND, "home", info="構えの位置に入った: 台本へ")
            if self.approach.done:
                self._enter_arm_prep(now)
                return Decision(2, "arm", "hold", self.last_pose, closed, STAND, "goal", info="構えの位置に入った: 腕を構える")
            return Decision(1, "arm", "hold", self.last_pose, closed, (any(cmd), *cmd), "home", info=self.approach.phase)
        if self.stage == 2:
            lpose, why = self._lidar_pose(now, fresh)
            if why:
                return self._stop(2, "段 2: " + why, closed)
            self._keep(now, lpose)
            if lpose is None and self.last_pose_t is not None and now - self.last_pose_t > POSE_LOST_S:
                return self._stop(2, f"段 2: FAST-LIO の姿勢が {POSE_LOST_S} 秒来ない", closed)
            if self.script is not None:
                return self._script(now, q, quat, tau)
            if self.prep_until is not None:
                if now < self.prep_until:
                    return Decision(2, "arm", "hold", self.last_pose, closed, STAND, "goal", info="腕を構える")
                self._enter_stage2(now)
            palm = self.palm_fn(q, self.last_pose, quat)
            out = self.arm.step(now, palm, None)
            if out.failed:
                return self._stop(2, "段 2: " + self.arm.reason, out.angle_obs)
            if out.handover and fresh is None:  # 段 3 の枠は odom で運ぶので、新しい odom がある周期に持ち替える
                self.handover_wait_t = now if self.handover_wait_t is None else self.handover_wait_t
                if now - self.handover_wait_t > ODOM_LOST_S:
                    return self._stop(2, f"段 2: 扉 3° に届いたが odom が {ODOM_LOST_S} 秒来ない（段 3 の枠を運べない）", out.angle_obs)
            elif out.handover:
                self._enter_stage3(now, self.last_pose, fresh)
                return Decision(3, "f3t3", "door", self.last_pose, out.angle_obs, None, "policy", handover_now=True,
                                info=f"扉 {math.degrees(out.angle_obs[0]):.1f}°: f3t3 へ引き継ぐ")
            return Decision(2, "arm", "door", self.last_pose, out.angle_obs, None if out.legs_free else STAND, "policy",
                            zero_walk=not out.legs_free, info="脚を放した" if out.legs_free else "脚は止めたまま")
        # 段 3
        if fresh is not None:
            self.last_pose, self.last_odom_t = self.carry.pose(dfr.PlanarPose(*fresh)), now
        elif self.last_odom_t is not None and now - self.last_odom_t > ODOM_LOST_S:
            return self._stop(3, f"段 3: odom が {ODOM_LOST_S} 秒来ない（枠を運べない）", closed)
        palm = self.palm_fn(q, self.last_pose, quat)
        out = self.arm.step(now, palm, None) if self.arm is not None else None
        angle = out.angle_obs if out is not None else closed
        return Decision(3, "f3t3", "door", self.last_pose, angle, None, "policy", info="f3t3")

    def _script(self, now: float, q: np.ndarray, quat: np.ndarray, tau: Optional[np.ndarray]) -> Decision:
        """台本の段 2・3。方策は回さず（"hold"）、腕は逆運動学の目標、脚は台本の指令"""
        out, arm_left = self.script.step(now, q, quat, self.last_pose, tau)
        stage = 3 if out.sub in ("3a", "3c", "4a", "done") else 2  # 11-14: 4a（段 4 の入り）も走らせ役の段 3 のまま
        arm = "home" if (out.goal is None and not out.stop) else "script"
        return Decision(stage, "arm", "hold", self.last_pose, out.angle_obs, out.walk, arm, stop=out.stop,
                        info=f"台本 {out.sub}", arm_left=arm_left, arm_other=self.script.arm_other, waist_comp=out.waist_comp, waist_twist=out.waist_twist, waist_lean=out.waist_lean, script_row=self.script.script.row())

    def _stage0(self, now, q, quat, q_waist, roll, pitch, fresh, obs, closed) -> Decision:
        carried = None
        if self.lidar.ready(now):
            carried, why = self.lidar.pose(now, fresh)
            if why:
                return self._stop(0, "段 0: " + why, closed)
        net_pose = self.net.sample(now, q, quat, obs)
        if net_pose is not None and self.net_dx:
            net_pose = dfr.PlanarPose(net_pose.x + self.net_dx, net_pose.y, net_pose.yaw)
        out = self.stage0.step(now, net_pose, carried, self.lidar.ready(now))
        if out.ref is not None:
            lp = dlm.pelvis_to_lidar(out.ref.x, out.ref.y, out.ref.yaw, q_waist, roll, pitch, tuple(self.lidar.head_xy))
            self.lidar.request_ref(lp)
            self.last_pose = out.ref
            print(f"[staged] 段 0: {out.info}・扉の座標の pelvis x {out.ref.x:+.3f} y {out.ref.y:+.3f} 向き {math.degrees(out.ref.yaw):+.1f}°"
                  f"（ライダー x {lp[0]:+.3f} y {lp[1]:+.3f} 向き {math.degrees(lp[2]):+.1f}°）", flush=True)
        elif carried is not None:
            self.last_pose = carried
        if self.stage0.failed:
            return self._stop(0, "段 0: " + self.stage0.reason, closed)
        if self.stage0.done:
            self._enter_stage1(now)
            return Decision(1, "arm", "hold", self.last_pose, closed, STAND, "home", info="段 0 が終わった")
        return Decision(0, "arm", "hold", self.last_pose, closed, (any(out.cmd), *out.cmd), "home", info=out.info)

    def _keep(self, now: float, pose: Optional[dfr.PlanarPose]) -> None:
        if pose is not None:
            self.last_pose, self.last_pose_t = pose, now

    def _lidar_pose(self, now: float, fresh) -> Tuple[Optional[dfr.PlanarPose], str]:
        """FAST-LIO の姿勢。直前に受け入れた姿勢から跳んだ 1 回は捨てる（None＝短い途切れ。続けば POSE_LOST_S の見張りで止まる）"""
        lpose, why = self.lidar.pose(now, fresh)
        if lpose is None or why or self.last_pose_t is None or now - self.last_pose_t > POSE_JUMP_WINDOW_S:
            self.jumped = False
            return lpose, why
        p = self.last_pose
        jump = math.hypot(lpose.x - p.x, lpose.y - p.y)
        turn = abs(math.degrees(ds.wrap(lpose.yaw - p.yaw)))
        if jump <= POSE_JUMP_M and turn <= POSE_JUMP_DEG:
            self.jumped = False
            return lpose, why
        if not self.jumped:
            print(f"[staged] ⚠️ 姿勢が跳んだので捨てた: ({p.x:+.2f},{p.y:+.2f},{math.degrees(p.yaw):+.0f}°) → "
                  f"({lpose.x:+.2f},{lpose.y:+.2f},{math.degrees(lpose.yaw):+.0f}°)", flush=True)
        self.jumped = True
        return None, ""

    def _stop(self, stage: int, why: str, angle: np.ndarray) -> Decision:
        return Decision(stage, "f3t3" if stage >= 3 else "arm", "hold", self.last_pose, angle, STAND,
                        "policy" if stage >= 2 else "home", stop=why)
