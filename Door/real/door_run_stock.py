"""11-02: 扉の方策を純正の脚で実機の G1 に載せる（計画 docs/plan/2026-11-02-g1-door-real-stock-legs.md の手順 4）。

HOMIE の経路（door_run_real.py）とは別のプロセス。rt/lowcmd は送らない。純正の歩行（通常運転 500/501）のまま起動し、
腕だけを rt/arm_sdk で借り、歩きは生徒の指令を SetVelocity で純正へ渡す（door_stock_io.py）。計算は door_core.py。
生徒の観測の脚と腰の行動には、HOMIE を計算だけ回した出力（影の HOMIE）を入れる（--leg-actions で替えられる）。

流れ（PC で文字を打って Enter。リモコンのボタンは純正の操作と重なるので、進めるのは PC だけ）:
  WAIT      純正が全身を持つ。a で次へ
  ARMS_IN   腕を借りる: 重みを 2 秒で 0 → 1（今の腕の角度を保つ）→ 学習の初めの腕の姿勢へ 1 rad/s で
  READY     腕をその姿勢で保ち、影の HOMIE を回す。--door-source lidar なら推定器へ REF を送る。d で扉の方策へ
            （傾きが --max-stand-tilt-deg 以下・両足に体重が乗っている（--load-check）・ライダーの準備ができているときだけ）
  DOOR      生徒: 腕は rt/arm_sdk・歩きは SetVelocity。--run-seconds 秒・扉の先 1.5 m（通り抜け）・扉が --stop-door-deg で POST へ
  POST      立ったまま止まる: 歩きは 0、腕はその場の目標で保つ（純正が立つ）。r で腕を戻す・a で構え直す（もう一度 READY へ）
  ARMS_OUT  腕を ARMS_IN の前の角度へ 1 rad/s で戻し、重みを 2 秒で 0 へ（純正へ返す）→ WAIT
  PROBE     手順 11（前へ出ないときの切り分け）: POST で p を打つと、腕は止めた姿勢のまま前へ --probe-vx（p0.5 なら 0.5 m/s）を
            --probe-seconds 秒送り、POST へ戻る。扉に手を当てたまま止めたときに、脚が反力で前へ出ないのか方策の側かを分ける
  DAMP      脱力（純正の Damp）

止め方は 2 段（10-16 の約束のまま。ふだんの停止では脱力しない）:
  立ったまま止まる  PC の Enter（a・d・r・q 以外の入力も）・Ctrl+C の 1 回目・ssh の切断（SIGHUP）・リモコンの B か select
                    自動: rt/lowstate が 0.1 s 途切れる・機体の値が壊れる・傾き > --stop-tilt-deg・関節の速さ > --max-dq・
                          周期の遅れ・方策の計算で例外・SetVelocity が続けて失敗・ライダーの見張り・odom の途切れ（tape）・
                          SetVelocity の中継の状態が 0.5 s 来ない（--loco relay）
  脱力              リモコンの L2+B（純正のもの。こちらも Damp を送る）・Ctrl+C の 2 回目・傾き > --max-tilt-deg
  プログラムが落ちたとき: arm_sdk が途切れると純正はその時の形で腕を保ち（10-27 に pkill -9 で確認）、速度は 0.5 秒で切れる
                          （--loco relay では中継の子が、主の電文が 0.3 s 来なければ速度 0 を送り、親が居なくなれば止めて終わる）
  --loco relay（11-04）: SetVelocity を別のプロセス（loco_relay.py）で送る。同じプロセスだと返事が待たされ 3104 で落ちていた
  WAIT と DAMP では q・Enter・Ctrl+C で終わる

例（PC2。機体は吊って綱を緩め、リモコンで通常運転 501 に入れておく）:
  python3 door_run_stock.py --policy door_policy.npz --iface eth0 --door-source lidar --lidar-center-xy 0.047 -0.016 \\
      --door 0.94,2.05,0.99,0.064 --run-seconds 5

--door-source vision（11-05）: 頭のカラーの画像だけの生徒（export_vision_student.py の npz を --policy に渡す）。
  扉の値は画像から当てる（door_vision_core）。特徴は別のプロセス door_vision_node.py が 10 Hz で UDP に送る（door_vision_source）。
  特徴が 0.5 s 来ない・撮ってから 0.6 s より古いと d を受け付けず、DOOR なら立ったまま止まる。
  頭の重力の向きは pelvis の IMU・腰の 3 関節・--head-pitch-deg（その日に測ったカメラの俯角。torso から見て下向き）で作る。
  odom は記録だけ（姿勢は READY に入った所から）。通り抜けの判定と --stop-door-deg は使わない
"""
from __future__ import annotations

import argparse
import gc
import math
import os
import queue
import signal
import stat
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_foot_load as dfl  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_lidar_msg as dlm  # noqa: E402
import door_run_real as drr  # noqa: E402  受け取り（RobotIO）・リモコン・記録・足の力は HOMIE の経路と同じものを使う
import door_stock_io as dsio  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import door_vision_source as dvs  # noqa: E402
import target_guard as tg  # noqa: E402

DT = drr.DT
ARMS = slice(dc.NUM_HOMIE, dc.NUM_DOF)
WAIST = list(dsio.WAIST_IDX)
WEIGHT_RAMP_S = 2.0  # arm_sdk の重みを 0 ↔ 1 にかける時間（armsdk_probe.py・遠隔操作と同じ）
ARM_SPEED = 1.0  # [rad/s] 腕を構える・戻す速さ
ARM_SETTLE_RAD = 0.1  # 構えた腕がこれ以内に落ち着いたら READY
ARM_SETTLE_S = 2.0  # 落ち着かなくてもこれだけ待ったら READY（残りの差を表示する）
VEL_FAIL_TRIP = 5  # SetVelocity が続けてこれだけ失敗したら立ったまま止まる（20 回/秒で 0.25 秒）
DAMP_RELEASE_CYCLES = 5  # 脱力のとき重み 0 を送る周期の数（腕も純正へ返して Damp を効かせる）
ARM_TEMP_STOP_C = 75  # 腕のモーターの温度がこれ以上か状態の札が 0 でなければ立ったまま止まる（11-03 実機: 左肩 roll 84 °C・札 512 で力が抜けた）
ARM_TEMP_HOLD_S = 1.0  # 温度はこれだけ続けて線を越えたら止める（11-04: 読みは腕を押し当てた山で跳ねる。札は待たない）
# 11-06 実機（画像の生徒 d4）: 方策は DOOR の 1.5 秒から左肩 roll を −15° へ送り、腕は −4° で胴に当たって止まり、
# 積み上げの範囲いっぱい（δ 0.2 rad × kp 100＝20 N·m）で 5 秒押し続けて 45 → 88 °C。右肩 roll も 16 N·m で胴へ。
# シミュレータは腕が胴をすり抜ける（10-24 の洗い出し）ので、学習は胴へ押す力に気づかない。送る目標だけを切る
ROLL_PRESS_MAX_NM = 6.0  # 肩 roll が胴の側へ押す力の上限 [N·m]（熱は力の 2 乗なので 20 → 6 で約 1/11）
ARM_ROLL_INWARD = ((1, -1.0), (8, +1.0))  # 腕 14 の中の（肩 roll の位置, 胴へ向かう向き）: 左は負・右は正
# 11-06 実機（d4 を 3 回）: 純正は指令に対し前 約 6 割・横 約 4 割・旋回 約 9 割しか動かず（学習の脚の倍率は 0.8〜1.2）、
# 扉の 0.5 m 手前で方策の前 0.03〜0.2 m/s にほとんど応えず 15 秒うろついた。--vel-gain・--vx-floor で送る指令を持ち上げる（既定は切）
VEL_LIMIT = (0.6, 0.6, 1.0)  # 持ち上げた後の |vx|・|vy|・|wz| の上限（前はさらに --max-vx で切る）
VX_DEAD = 0.03  # これより小さい前後の指令は持ち上げない（止まっていてほしい所を歩かせない）
LIDAR_REF_RETRY_S = 15.0
PHASES = ("WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "DAMP", "PROBE")  # 記録は番号で持つので、足すときは後ろへ
ACTIVE = ("ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "PROBE")


class _NullLoco:
    """--no-send のときの純正の口（何も送らない）"""

    def SetVelocity(self, vx: float, vy: float, wz: float, duration: float) -> int:  # noqa: N802  SDK と同じ名前
        return 0

    def Damp(self) -> None:  # noqa: N802
        return None

    def GetFsmId(self) -> Tuple[int, int]:  # noqa: N802
        return 0, 501


class DoorValues:
    """扉の値（枠の位置・扉の角度）を毎周期つくる。tape: 巻き尺の最初の姿勢＋READY からの odom／lidar: 推定器（door_lidar_source）"""

    def __init__(self, start_pose: dfr.PlanarPose, lidar, lidar_mode: str, lidar_center_xy: Optional[List[float]]) -> None:
        self.start_pose = start_pose
        self.lidar = lidar
        self.lidar_mode = lidar_mode
        self.center_xy = tuple(lidar_center_xy) if lidar_center_xy is not None else None
        self.odom_ref: Optional[dfr.PlanarPose] = None
        self.last_pose = start_pose
        self.odom_lost_t: Optional[float] = None
        self.lpose: Optional[dfr.PlanarPose] = None
        self.head_set = False
        self.ref_t: Optional[float] = None
        self.angle = dfr.door_closed_angle_obs()

    def begin(self, now: float, odom, odom_t) -> str:
        """READY に入った時。odom の基準を取り、ライダーなら REF を送る（止まって 2.5 秒の点群で扉と地図を作る）"""
        fresh = odom is not None and odom_t is not None and now - odom_t < drr.ODOM_STALE_S
        self.odom_ref = dfr.PlanarPose(*odom) if fresh else None
        self.last_pose, self.odom_lost_t, self.lpose, self.head_set = self.start_pose, None, None, False
        if self.lidar is not None:
            self.lidar.request_ref()
            self.ref_t = now
            return "推定器へ REF を送った（扉と地図を作る。止まって待つ）"
        return "odom の基準を取った" if self.odom_ref else "⚠️ odom 無し（枠の位置は最初の姿勢のまま動かない）"

    def update(self, now: float, ready_s: float, odom, odom_t, q: np.ndarray, quat: np.ndarray,
               retry_ref: bool = False) -> Tuple[dfr.PlanarPose, float, str]:
        """(今の pelvis の姿勢, odom が新しいか, 見張りの理由（空なら無事）)。

        retry_ref: 推定器の準備ができないときに REF を送り直してよいか（READY の間だけ真）。
        ⚠️ 11-03 実機: DOOR でも送り直していて、歩き出しで推定が一瞬受け入れられなかった時に REF が飛び、推定器が地図の作り直しに入って止まった
        """
        if self.lidar is None:
            if self.odom_ref is not None and odom is not None and now - odom_t < drr.ODOM_STALE_S:
                self.last_pose = dfr.compose_planar(self.start_pose, self.odom_ref, dfr.PlanarPose(*odom))
                self.odom_lost_t = None
                return self.last_pose, 1.0, ""
            if self.odom_ref is not None and self.odom_lost_t is None:
                self.odom_lost_t = now
            lost = self.odom_lost_t is not None and now - self.odom_lost_t > drr.ODOM_LOST_S
            return self.last_pose, 0.0, (f"odom が {now - self.odom_lost_t:.1f} s 途切れた（枠の位置が分からない）" if lost else "")
        odom_fresh = odom if (odom is not None and now - odom_t < drr.ODOM_STALE_S) else None
        q_waist = tuple(float(v) for v in q[list(dlm.WAIST_IDX)])
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
        self.lidar.note_state(now, odom if (odom is not None and now - odom_t < 0.05) else None, q_waist, roll, pitch)
        if not self.head_set and ready_s > 0.5:
            head = dlm.head_xy_from_center_offset(self.center_xy, q_waist, roll, pitch)
            self.lidar.head_xy = head  # 立った姿勢で取付を合わせる。以後の腰の動きは順運動学で追う
            self.head_set = True
            print(f"[door_stock] ライダーの取付の補正 {head[0]:+.3f} {head[1]:+.3f} m（腰 {np.degrees(q_waist).round(1).tolist()}°）", flush=True)
        if retry_ref and self.ref_t is not None and now - self.ref_t > LIDAR_REF_RETRY_S and not self.lidar.ready(now):
            self.lidar.request_ref()  # 推定器が扉を測れなかった（「REF をもう一度」）ときの送り直し
            self.ref_t = now
            print("[door_stock] ライダーの準備が 15 s でできない: REF を送り直した", flush=True)
        self.lpose, why = self.lidar.pose(now, odom_fresh)
        self.angle = self.lidar.angle_obs(now)
        if self.lpose is not None:
            self.last_pose = self.lpose
        return self.last_pose, (1.0 if odom_fresh is not None else 0.0), why

    def refuse_door(self, now: float) -> str:
        """扉の方策を始められない理由（空なら始めてよい）"""
        if self.lidar is None:
            return ""
        if not self.lidar.ready(now):
            return "ライダーの準備がまだ（推定器のログを見る）"
        if self.lidar_mode == "track" and self.lpose is None:
            return "ライダーの姿勢がまだ無い"
        if self.lidar_mode == "still":
            sp = self.lidar.take_still(now)
            if sp is None:
                return "still: 最初の姿勢が決められない（受け入れた推定・odom が足りない）"
            print(f"[door_stock] still: 最初の姿勢 ({sp.x:+.3f},{sp.y:+.3f},{math.degrees(sp.yaw):+.1f}°)・以後は odom で運ぶ", flush=True)
        return ""

    def door_deg(self) -> float:
        """測れた最後の扉の角度 [°]（測れていなければ nan）"""
        return math.degrees(float(self.angle[0])) if self.lidar is not None and self.angle[1] > 0.5 else float("nan")


def parse_probe(line: str, default_vx: float) -> Optional[float]:
    """押す試しの指示: "p" は default_vx、"p0.5" はその速さ [m/s]（0 より大きく 1 以下）。押す試しの指示でなければ None"""
    if not line.startswith("p"):
        return None
    rest = line[1:].strip()
    if not rest:
        return default_vx
    try:
        v = float(rest)
    except ValueError:
        return None
    return v if 0.0 < v <= 1.0 else None


def cap_roll_press(target: np.ndarray, q_arm: np.ndarray, kp_arm: np.ndarray, max_nm: float) -> np.ndarray:
    """肩 roll の目標を、胴の側へ押す力 kp·(目標 − 角度) が max_nm [N·m] を越えない所まで戻した写し。
    外へ開く側と他の関節は切らない。送る目標だけを切り、方策の積み上げ D と観測には触らない。入力は書き換えない"""
    out = np.array(target, dtype=float)
    for j, inward in ARM_ROLL_INWARD:
        lim = q_arm[j] + inward * max_nm / kp_arm[j]
        out[j] = max(out[j], lim) if inward < 0 else min(out[j], lim)
    return out


def boost_stock_velocity(vx: float, vy: float, wz: float, gain: tuple, vx_floor: float) -> tuple:
    """純正の歩行の鈍さを補う: 各軸に gain を掛け、|vx| が VX_DEAD を越えて vx_floor 未満なら vx_floor へ持ち上げ、VEL_LIMIT で切る"""
    out = [vx * gain[0], vy * gain[1], wz * gain[2]]
    if VX_DEAD < abs(vx) and abs(out[0]) < vx_floor:
        out[0] = math.copysign(vx_floor, vx)
    return tuple(max(-lim, min(lim, v)) for v, lim in zip(out, VEL_LIMIT))


def ready_refusals(tilt: float, max_stand_tilt: float, load: Optional[dfl.GateVerdict], door_why: str) -> List[str]:
    """READY で d を打った時に受け付けない理由（空なら受け付ける）。load は足の確かめの判定（None は見ない）"""
    out = drr.hold_refusals(tilt, max_stand_tilt, load)
    if door_why:
        out.append(door_why)
    return out


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True, help="export_door_policy.py が書いた npz")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--start-x", type=float, help="[m] 扉の板の面から前後（手前は負）。--door-source tape で要る")
    ap.add_argument("--start-y", type=float, help="[m] 開口の中心から左右（扉を向いて左が正）。--door-source tape で要る")
    ap.add_argument("--start-yaw-deg", type=float, default=0.0)
    ap.add_argument("--door", required=True, help="幅,高さ,レバーの高さ,バックセット [m]")
    ap.add_argument("--lever-right", action="store_true", help="把手が右の扉（UIS Room 4 は左なので付けない）")
    ap.add_argument("--run-seconds", type=float, default=5.0)
    ap.add_argument("--stop-door-deg", type=float, default=None, help="扉がこの角度 [°] まで開いたら POST（ライダーのときだけ）")
    ap.add_argument("--arm-temp-stop", type=float, default=ARM_TEMP_STOP_C,
                    help="[°C] 腕のモーターがこれ以上なら立ったまま止まる（純正の保護の札は 84 °C 前後で立つ）")
    ap.add_argument("--arm-temp-hold", type=float, default=ARM_TEMP_HOLD_S,
                    help="[s] 腕の温度がこれだけ続けて --arm-temp-stop を越えたら止める（一瞬の山では止めない。札は待たない）")
    ap.add_argument("--odom", choices=("dog", "relay", "none"), default="dog",
                    help="relay: rt/dog_odom を別のプロセス（odom_relay.py）で受けて UDP で受け取る（11-03。同じプロセスで受けると GIL を食う）")
    ap.add_argument("--clip-margin", type=float, default=0.05, help="腕の目標を可動域の端から内側へ切る余裕 [rad]")
    ap.add_argument("--vel-gain", default="1,1,1", help="純正へ送る vx,vy,wz に掛ける倍率（11-06 実機の応えは 約 0.6,0.4,0.9）")
    ap.add_argument("--vx-floor", type=float, default=0.0, help="[m/s] 前後の小さい指令（|vx| > 0.03）をこの大きさまで持ち上げる。0 で切")
    ap.add_argument("--roll-press-max", type=float, default=ROLL_PRESS_MAX_NM,
                    help="[N·m] DOOR・POST で肩 roll が胴の側へ押す力の上限（送る目標を今の角度から切る）。0 以下で切らない")
    ap.add_argument("--stop-tilt-deg", type=float, default=10.0, help="これより傾いたら立ったまま止まる")
    ap.add_argument("--max-tilt-deg", type=float, default=30.0, help="これより傾いたら脱力（倒れている）")
    ap.add_argument("--max-dq", type=float, default=20.0, help="[rad/s] どれかの関節がこれより速ければ立ったまま止まる")
    ap.add_argument("--max-stand-tilt-deg", type=float, default=8.0, help="READY でこれより傾いていたら d を受け付けない")
    ap.add_argument("--load-check", choices=("on", "off"), default="on",
                    help="READY で両足に体重が乗っているか（綱が体重を受けていないか）を確かめてから d を受け付ける")
    ap.add_argument("--min-load-total", type=float, default=dfl.MIN_TOTAL)
    ap.add_argument("--min-load-each", type=float, default=dfl.MIN_EACH)
    ap.add_argument("--leg-model", default=str(Path(__file__).resolve().parent / "leg_model_g1_29dof_rev_1_0.npz"))
    ap.add_argument("--body-mass-kg", type=float, default=None)
    ap.add_argument("--leg-actions", choices=dsio.LEG_ACTIONS, default="shadow",
                    help="生徒の観測の脚と腰の行動: shadow＝影の HOMIE の出力（既定）／from_q＝関節の角度から逆算／zero＝0")
    ap.add_argument("--max-vx", type=float, default=0.5, help="[m/s] 前への上限（学習の指令の頭打ちは ±0.5）")
    ap.add_argument("--probe-vx", type=float, default=0.3, help="[m/s] 押す試し（POST で p）の前への速さ。p0.5 のように打てば替えられる")
    ap.add_argument("--probe-seconds", type=float, default=2.0, help="押す試しで前への指令を送る秒")
    ap.add_argument("--rpc-timeout", type=float, default=1.0, help="[s] 純正の RPC の応答を待つ上限（送る糸だけが待つ）")
    ap.add_argument("--loco", choices=("inproc", "relay"), default="inproc",
                    help="relay: SetVelocity を別のプロセス（loco_relay.py）で送る（11-04。同じプロセスだと返事が待たされ 3104 で落ちる）")
    ap.add_argument("--no-send", action="store_true", help="arm_sdk も SetVelocity も送らない（読むだけ・計算と記録はする）")
    ap.add_argument("--door-source", choices=("tape", "lidar", "vision"), default="lidar")
    ap.add_argument("--vision-actor", choices=("student", "teacher"), default="student",
                    help="teacher: 扉の値は画像から当て、行動は npz の教師（f3sc）が出す（11-06 の切り分け）")
    ap.add_argument("--vision-port", type=int, default=dvs.PORT, help="door_vision_node.py の特徴を受ける UDP の口")
    ap.add_argument("--head-pitch-deg", type=float, default=None,
                    help="--door-source vision: カメラの俯角 [°]（torso から見て下向き。既定は学習の取付の値＝npz の rot_offset）")
    ap.add_argument("--cam-tilt-deg", type=float, default=None,
                    help="--door-source vision: カメラ D435i の加速度計で測った頭の床からの俯角 [°]（pc2/cam_tilt.py）。"
                         "渡すと起動時の姿勢（pelvis の IMU・腰）から --head-pitch-deg を決める（--head-pitch-deg より優先）")
    ap.add_argument("--lidar-mode", choices=("track", "still"), default="track")
    ap.add_argument("--lidar-port", type=int, default=dlm.PORT)
    ap.add_argument("--lidar-center-xy", type=float, nargs=2, metavar=("前", "左"),
                    help="[m] 立っているときの pelvis から見たライダーの水平の位置（その日に巻き尺で決める）")
    ap.add_argument("--cmd-fifo", default=None,
                    help="PC の指示を読む名前つきパイプ（切り離して起こすとき。pc2/stock_cmd.sh が書く）。標準入力も読み続ける")
    ap.add_argument("--log-dir", default="logs")
    a = ap.parse_args(argv)
    if a.door_source == "tape" and (a.start_x is None or a.start_y is None):
        ap.error("--door-source tape では --start-x と --start-y が要る")
    if a.door_source == "lidar" and a.lidar_center_xy is None:
        a.lidar_center_xy = [0.047, -0.016]
        print("[door_stock] ⚠️ --lidar-center-xy が無い: 10-21 の旋回の値 (0.047, −0.016) を使う。その日に巻き尺で測ってから渡す", flush=True)
    return a


def watch_fifo(path: str, sink: "queue.Queue[str]") -> None:
    """名前つきパイプから 1 行ずつ読む（door_start.sh で切り離して起こすと標準入力が無いので、pc2/stock_cmd.sh がここへ書く）。
    書き手が閉じたら開き直す"""
    fifo = Path(path)
    try:
        if not fifo.exists():
            os.mkfifo(fifo)
        if not stat.S_ISFIFO(fifo.stat().st_mode):
            print(f"[door_stock] ⚠️ {fifo} は名前つきパイプでない: PC の指示は標準入力だけ", flush=True)
            return
        while True:
            with open(fifo, "r", encoding="utf-8") as f:  # 書き手が開くまで待つ
                for line in f:
                    sink.put(line.strip().lower())
    except OSError as e:
        print(f"[door_stock] ⚠️ 指示の口 {fifo} が使えない（{type(e).__name__}: {e}）: PC の指示は標準入力だけ", flush=True)


def check_stock_running(fsm: Optional[int]) -> str:
    """純正が歩ける状態か（fsm は LocoClient で読んだ値。中継のときは子が読んだ値）。だめなら理由（空なら起動してよい）"""
    name = drr.motion_mode_name()
    if not name:
        return "純正の歩行が動いていない（このプログラムは純正の脚で歩く。リモコンで立たせて通常運転 501 に入れる）"
    if fsm not in dsio.WALKING_FSM_IDS:
        return f"FSM={fsm} は通常運転（500/501）ではない（指令が通っても足が動かない・arm_sdk が効かない）。リモコンで通常運転へ"
    return ""


def arm_temps(state) -> np.ndarray:
    """腕 14 関節のモーターの温度 [°C]（2 つの値の高い方）。偽の機体で値が無ければ 0"""
    return np.array([max((float(v) for v in getattr(state.motor_state[j], "temperature", (0,))), default=0.0)
                     for j in range(ARMS.start, ARMS.stop)])


def arm_motor_health(state) -> Tuple[float, int, Optional[int]]:
    """腕 14 関節のモーターの (最高温度 [°C], その関節, 状態の札が 0 でない最初の関節 or None)。偽の機体で値が無ければ 0 と None"""
    temps = arm_temps(state)
    j_flag = next((j for j in range(ARMS.start, ARMS.stop)
                   if int(getattr(state.motor_state[j], "motorstate", 0) or 0) != 0), None)
    return float(temps.max()), ARMS.start + int(temps.argmax()), j_flag


class ArmHeatGate:
    """腕の温度が線を hold_s 続けて越えたら止める。モーターの保護の札は待たずに止める。
    11-04 実機: 温度の読みは力を入れると上がり、抜くと 6 秒で 20 °C 下がる（巻線の見積もり）。触ると 90 °C は無いのに、
    方策が腕を押し当てた山で 90 °C の線を越えて止まった"""

    def __init__(self, limit_c: float, hold_s: float) -> None:
        self.limit_c, self.hold_s = limit_c, hold_s
        self.since: Optional[float] = None  # 線を越え始めた時刻

    def update(self, now: float, temp_c: float, flag_j: Optional[int]) -> bool:
        """止めるなら True（毎周期呼ぶ）"""
        if temp_c < self.limit_c:
            self.since = None
        elif self.since is None:
            self.since = now
        return flag_j is not None or (self.since is not None and now - self.since >= self.hold_s - 1e-9)

    def held_s(self, now: float) -> float:
        return 0.0 if self.since is None else now - self.since


RELAY_WAIT_S = 3.0  # 中継の最初の電文をこれだけ待つ
LOCO_RELAY_WAIT_S = 5.0  # SetVelocity の中継の最初の状態をこれだけ待つ（DDS の初期化・LocoClient の Init 0.5 s・FSM の読み取りを含む）
LOCO_STATUS_STALE_S = 0.5  # 中継の状態がこれより古ければ、方策の間は立ったまま止まる（子は 20 ms ごとに送る）


def start_loco_relay(iface: str, rpc_timeout: float):
    """loco_relay.py を子のプロセスで起こし、(主の側の口, 子が読んだ FSM) を返す。最初の状態が来なければ (None, None)"""
    import subprocess

    import loco_relay as lrl

    # 子が居なくなった時だけ、脱力を主のプロセスから送る（DDS は RobotIO が初期化済み。ふだんは LocoClient を作らない）
    relay = lrl.LocoRelay(lrl.CMD_PORT, lrl.STATUS_PORT, fallback_client=lambda: dsio.make_loco_client(rpc_timeout))
    relay.child = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve().parent / "loco_relay.py"),
                                    "--iface", iface, "--cmd-port", str(lrl.CMD_PORT), "--status-port", str(lrl.STATUS_PORT),
                                    "--rpc-timeout", str(rpc_timeout), "--parent-pid", str(os.getpid())])
    t0 = time.monotonic()
    if not relay.wait_ready(LOCO_RELAY_WAIT_S):
        print(f"[door_stock] 起動しない: SetVelocity の中継（loco_relay.py）から {LOCO_RELAY_WAIT_S:.0f} s 状態が来ない"
              "（前の子が居残ってポートを持っていないか: pgrep -af loco_relay）", flush=True)
        relay.close()
        return None, None
    print(f"[door_stock] SetVelocity は中継から（loco_relay.py pid {relay.child.pid}・{time.monotonic() - t0:.1f} s で届いた）",
          flush=True)
    return relay, relay.fsm


def start_odom_relay(iface: str):
    """odom_relay.py を子のプロセスで起こし、UDP の受け口を返す。(受け口, 子)。最初の電文が来なければ (None, None)"""
    import subprocess

    import odom_relay as orl

    rx = orl.UdpOdom(orl.PORT)
    child = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve().parent / "odom_relay.py"),
                              "--iface", iface, "--port", str(orl.PORT)])
    t0 = time.monotonic()
    while rx.latest()[0] is None:
        if time.monotonic() - t0 > RELAY_WAIT_S or child.poll() is not None:
            print(f"[door_stock] 起動しない: odom の中継（odom_relay.py）から {RELAY_WAIT_S:.0f} s 電文が来ない", flush=True)
            stop_odom_relay(rx, child)
            return None, None
        time.sleep(0.05)
    print(f"[door_stock] odom は中継から（odom_relay.py pid {child.pid}・{time.monotonic() - t0:.1f} s で届いた）", flush=True)
    return rx, child


def stop_odom_relay(rx, child) -> None:
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=2.0)
        except Exception:  # noqa: BLE001  止まらなければ強く止める
            child.kill()
    if rx is not None:
        rx.close()


@dataclass(frozen=True)
class TeacherOnVision(dvc.VisionActor):
    """11-06 実機: 扉の値は画像（頭＋追う段）から当て、行動は npz の教師（f3sc）が出す。
    生徒は扉の 0.5 m 手前で前を止めてうろつき、同じ観測を教師に通すと前へ出し続けた → 生徒の行動の側が実物の画像に迷うのかを切り分ける"""
    teacher: Optional[dc.Params] = None

    def forward(self, obs: np.ndarray, h: np.ndarray, cell: np.ndarray):
        return dc.actor_forward(self.teacher, obs, h, cell)


def vision_policy(vp: dvc.VisionParams, feats: dvs.VisionFeatSource, quat: np.ndarray, q: np.ndarray, head_pitch: float,
                  mirror: bool, teacher: Optional[dc.Params] = None):
    """今の特徴と頭の重力で画像の生徒の policy を作る → (policy, 電文)。特徴がまだ無ければ (None, None)。
    teacher を渡すと扉の値だけ画像から当て、行動は教師が出す（TeacherOnVision）"""
    m = feats.latest()
    if m is None:
        return None, None
    hg = dvc.mirror_head_gravity(vp, dvc.head_gravity(quat, q[WAIST], head_pitch), mirror)
    if teacher is not None:
        return TeacherOnVision(vp, m[1], hg, teacher), m
    return dvc.VisionActor(vp, m[1], hg), m


def vision_row(vp: dvc.VisionParams, out, vmsg, pol, now: float) -> dict:
    """記録の列（毎周期そろえる。無い所は NaN）: 特徴の番号・撮ってからの古さ・網の ms・頭の重力（鏡写しの側）・当てた扉の値 17"""
    nan3, nan17 = np.full(3, np.nan), np.full(dvc.DOOR_DIM, np.nan)
    return dict(vis_seq=vmsg[0] if vmsg else np.nan, vis_age=(now - vmsg[2]) if vmsg else np.nan,
                vis_encode_ms=vmsg[4] if vmsg else np.nan, vis_hg=pol.hg_m if pol is not None else nan3,
                vis_door=dvc.door_from_obs(vp, out.obs) if (out is not None and pol is not None) else nan17)


def vision_text(p: dc.Params, vp: dvc.VisionParams, out, vmsg, pol, now: float) -> str:
    """表示: 画像から当てた扉（実の向き。pelvis から見た開口の中心と扉の角度）・特徴の古さ・頭の床からの俯角。
    頭の俯角は --head-pitch-deg に胴の傾きを足したもの。カメラの加速度計（pc2/cam_tilt.py）の値と比べる（--cam-tilt-deg なら起動時に合わせ済み）"""
    if vmsg is None:
        return "画像 まだ "
    age = f"古さ {1000 * (now - vmsg[2]):.0f} ms "
    if out is None or pol is None:
        return "画像 " + age
    ang, pos = dvc.door_real(p, vp, out.obs)
    tilt = dvc.floor_tilt_deg(pol.hg_m)
    return f"画像の扉 前 {pos[0]:+.2f} 左 {pos[1]:+.2f} m・{math.degrees(ang):.0f}° {age}頭の俯角（床から）{tilt:.1f}° "


def warm_up_vision(p: dc.Params, vp: dvc.VisionParams, state, geometry: np.ndarray, pose: dfr.PlanarPose) -> float:
    """画像の生徒を起動時に空回しする（使い捨ての状態）。返り値はかかった時間 [ms]"""
    t0 = time.monotonic()
    inp = drr.state_inputs(state, pose, geometry)
    pol = dvc.VisionActor(vp, np.zeros(vp.feat_dim), dvc.head_gravity(np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), 0.9))
    s = dc.initial_state(p)
    for mode in ("hold", "door"):
        for _ in range(drr.WARM_UP_STEPS):
            s, _ = dc.step(p, s, inp, mode=mode, policy=pol)
    return (time.monotonic() - t0) * 1000.0


def main(argv=None) -> int:  # noqa: C901  段の進み方を 1 か所で読めるように、輪は 1 つの関数にしている（door_run_real.py と同じ）
    a = parse_args(argv)
    p = dc.load_params(a.policy, mirror=not a.lever_right)
    start_pose = (dfr.PlanarPose(a.start_x, a.start_y, math.radians(a.start_yaw_deg)) if a.door_source == "tape"
                  else dfr.PlanarPose(-1.0, 0.0, 0.0))  # ライダーの推定が来るまでの置き場所（READY では使わない）
    lidar = None
    if a.door_source == "lidar":
        from door_lidar_source import LidarDoorSource

        lidar = LidarDoorSource(a.lidar_port, mode=a.lidar_mode)
    doorv = DoorValues(start_pose, lidar, a.lidar_mode, a.lidar_center_xy)
    vp, feats, head_pitch = None, None, 0.0
    if a.door_source == "vision":
        vp = dvc.load_vision(a.policy, p)
        head_pitch = math.radians(a.head_pitch_deg if a.head_pitch_deg is not None else dvc.camera_pitch_deg(vp))
        feats = dvs.VisionFeatSource(vp.feat_dim, a.vision_port)
    w, h, lh, bs = (float(v) for v in a.door.split(","))
    vel_gain = tuple(float(v) for v in a.vel_gain.split(","))
    geometry = dfr.door_geometry(w, h, lh, bs, lever_left=not a.lever_right)
    leg_model = dfl.load_leg_model(a.leg_model)
    body_weight = (a.body_mass_kg if a.body_mass_kg else leg_model.total_mass) * leg_model.gravity
    use_load_check = a.load_check == "on"
    max_tilt, stop_tilt, max_stand_tilt = (math.radians(v) for v in (a.max_tilt_deg, a.stop_tilt_deg, a.max_stand_tilt_deg))
    kp_arm, kd_arm, q_arm_goal = p.kp[ARMS], p.kd[ARMS], p.q_default[ARMS]
    lo_arm, hi_arm = p.q_lower[ARMS], p.q_upper[ARMS]

    io = drr.RobotIO(a.iface, send=False, use_odom=(a.odom == "dog"))  # rt/lowcmd は送らない（受け取りだけ）
    t_wait = time.monotonic()
    while io.snapshot()[0] is None:
        if time.monotonic() - t_wait > 5.0:
            print("[door_stock] rt/lowstate が来ない（--iface と機体の電源を確かめる）")
            return 1
        time.sleep(0.05)
    odom_rx, relay = (start_odom_relay(a.iface) if a.odom == "relay" else (None, None))
    if a.odom == "relay" and odom_rx is None:
        return 1
    state0 = io.snapshot()[0]
    drr.check_29dof(state0)
    if vp is not None and a.cam_tilt_deg is not None:  # 11-05 夜: 床の AprilTag や手で合わせる代わりに、カメラの加速度計の値に合わせる
        q0 = np.array([state0.motor_state[j].q for j in range(dc.NUM_DOF)])
        quat0 = np.array(state0.imu_state.quaternion, dtype=float)
        given = math.degrees(head_pitch)
        head_pitch = dvc.pitch_for_floor_tilt(quat0, q0[WAIST], a.cam_tilt_deg, head_pitch)
        print(f"[door_stock] 頭の俯角: カメラの加速度計 {a.cam_tilt_deg:.2f}°（床から）に合わせて torso から "
              f"{math.degrees(head_pitch):.2f}°（前の値 {given:.1f}°）", flush=True)
    why = dvc.head_pitch_out_of_range(vp, head_pitch) if vp is not None else None
    if why:
        print(f"[door_stock] 起動しない: {why}", flush=True)
        stop_odom_relay(odom_rx, relay)
        return 1
    if a.loco == "relay" and not a.no_send:
        sender, fsm = start_loco_relay(a.iface, a.rpc_timeout)  # 主のプロセスは LocoClient を持たない
        if sender is None:
            stop_odom_relay(odom_rx, relay)
            return 1
    else:
        loco = _NullLoco() if a.no_send else dsio.make_loco_client(a.rpc_timeout)
        fsm = None if a.no_send else dsio.fsm_id(loco)
        sender = dsio.VelocitySender(loco)
    if not a.no_send:
        why = check_stock_running(fsm)
        if why:
            print(f"[door_stock] 起動しない: {why}")
            sender.close()
            stop_odom_relay(odom_rx, relay)
            return 1
    arm_io = None if a.no_send else dsio.ArmSdkWriter(state0.mode_machine)
    sender.start()
    print(f"[door_stock] 29 関節・送る={'しない' if a.no_send else 'する'}・扉 {geometry.tolist()}・鏡写し {p.mirror}・"
          f"扉の値 {a.door_source}{'（' + a.lidar_mode + '）' if lidar else ''}・脚の観測 {a.leg_actions}・前の上限 {a.max_vx} m/s"
          + (f"・足の確かめ 合計 {100 * a.min_load_total:.0f}%・片足 {100 * a.min_load_each:.0f}%" if use_load_check else "・⚠️ 足の確かめ off"),
          flush=True)
    print(f"[door_stock] 方策の空回し: {drr.warm_up(p, state0, geometry, start_pose):.0f} ms", flush=True)
    if vp is not None:
        print(f"[door_stock] 画像の生徒: 特徴 {vp.feat_dim}・カメラ {vp.meta['camera']['hfov_deg']:.0f}°×{vp.meta['camera']['vfov_deg']:.0f}°・"
              f"俯角 {math.degrees(head_pitch):.1f}°（学習の乱択 {vp.meta['camera']['pitch_range_deg']}）・"
              f"空回し {warm_up_vision(p, vp, state0, geometry, start_pose):.0f} ms", flush=True)
    print("[door_stock] PC で打つ: a＝腕を構える・d＝扉の方策・p＝押す試し（POST のとき。p0.5 で速さを指定）・r＝腕を戻す・"
          "q＝終わる（WAIT のとき）・Enter だけ＝立ったまま止まる", flush=True)

    soft_req = threading.Event()  # 立ったまま止まる（Ctrl+C の 1 回目・SIGHUP）
    hard_req = threading.Event()  # 脱力して終わる（Ctrl+C の 2 回目）
    quit_count = [0]
    lines: "queue.Queue[str]" = queue.Queue()

    def on_quit(*_) -> None:
        quit_count[0] += 1
        (soft_req if quit_count[0] == 1 else hard_req).set()

    signal.signal(signal.SIGINT, on_quit)
    signal.signal(signal.SIGTERM, on_quit)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, lambda *_: soft_req.set())  # ssh が切れても終わらない

    def watch_stdin() -> None:
        while True:
            line = sys.stdin.readline()
            if not line:  # EOF＝標準入力が無い（nohup など）。止めない
                return
            lines.put(line.strip().lower())

    threading.Thread(target=watch_stdin, daemon=True).start()
    if a.cmd_fifo:
        threading.Thread(target=watch_fifo, args=(a.cmd_fifo, lines), daemon=True).start()

    log = drr.Logger()
    log_dir = Path(a.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    meta = dict(args=vars(a), phases=PHASES, program="door_run_stock", vision=vp.meta if vp is not None else None)
    phase, phase_t, reason = "WAIT", time.monotonic(), ""
    keys_prev = drr.parse_keys(state0.wireless_remote)
    s = dc.initial_state(p)
    load_gate = dfl.HoldLoadGate(weight=body_weight, min_total=a.min_load_total, min_each=a.min_load_each)
    heat_gate = ArmHeatGate(a.arm_temp_stop, a.arm_temp_hold)
    weight = 0.0
    arm_target: Optional[np.ndarray] = None  # 送っている腕の目標（None は送っていない）
    arm_home: Optional[np.ndarray] = None  # 借りる前の腕の角度（返すときに戻す）
    arm_hold: Optional[np.ndarray] = None  # READY で保つ腕の角度（構え終えた時に実際に届いた角度。届かない初めの姿勢を押し続けない）
    reached_t: Optional[float] = None  # ARMS_IN で目標に着いた時刻／ARMS_OUT で戻し終えた時刻
    damp_left = 0
    probe_vx = a.probe_vx
    complete = 0.0
    periods: List[float] = []
    next_t = time.monotonic()
    prev_now = None
    last_print = 0.0
    t_exit = None

    def go(new: str, why: str = "") -> None:
        nonlocal phase, phase_t, reason, reached_t, damp_left, arm_target
        if phase == "DAMP" or phase == new:
            return
        print(f"[door_stock] {phase} → {new}" + (f"（{why}）" if why else ""), flush=True)
        phase, phase_t, reached_t = new, time.monotonic(), None
        if new in ("POST", "ARMS_OUT", "DAMP"):
            sender.stop()
        if new == "POST" and arm_target is not None and np.isfinite(q[ARMS]).all():
            # 止めた瞬間の実際の角度で保つ。届かない目標（体・扉に当たった先）を押し続けない（11-03 実機: 左肩 roll が 84 °C）
            arm_target = np.clip(q[ARMS], lo_arm + a.clip_margin, hi_arm - a.clip_margin)
        if new in ("POST", "DAMP"):
            reason = why
        if new == "DAMP":
            sender.damp()
            damp_left = DAMP_RELEASE_CYCLES
            snap = {k: list(v) for k, v in log.rows.items()}  # 電源断でも残るように、脱力に入った時点で一度書く
            threading.Thread(target=drr.Logger.save_rows, args=(snap, log_dir / f"door_stock_{stamp}_damp.npz",
                                                                dict(meta, reason=why)), daemon=True).start()

    def probe_vx_req(cmds: List[str]) -> Optional[float]:
        for c in cmds:
            v = parse_probe(c, a.probe_vx)
            if v is not None:
                return v
        return None

    def soft_stop(why: str, from_pc: bool = False) -> None:
        """立ったまま止まる。腕を借りている段からは POST。WAIT・DAMP では PC からのときだけ終わる"""
        nonlocal t_exit
        if phase in ("ARMS_IN", "READY", "DOOR", "ARMS_OUT", "PROBE"):
            go("POST", why)
        elif from_pc and phase in ("WAIT", "DAMP") and t_exit is None:
            print(f"[door_stock] 終わる（{why}）", flush=True)
            t_exit = time.monotonic() + 0.2

    gc.collect()  # 記録が増えると GC が重くなる（10-16 に 1 周期 66 ms）。この輪は循環参照を作らない
    if hasattr(gc, "freeze"):
        gc.freeze()
    gc.disable()
    try:
        while True:
            now = time.monotonic()
            period = (now - prev_now) if prev_now is not None else DT
            prev_now = now
            periods.append(period)
            del periods[:-drr.PERIOD_AVG_N]
            state, state_t, odom, odom_t = io.snapshot()
            if odom_rx is not None:
                odom, odom_t = odom_rx.latest()
            q = np.array([state.motor_state[j].q for j in range(dc.NUM_DOF)])
            dq = np.array([state.motor_state[j].dq for j in range(dc.NUM_DOF)])
            tau = np.array([state.motor_state[j].tau_est for j in range(dc.NUM_DOF)])
            quat = np.array(state.imu_state.quaternion, dtype=float)  # w, x, y, z
            gyro = np.array(state.imu_state.gyroscope, dtype=float)
            gravity = dc.quat_wxyz_to_gravity(quat)
            tilt = math.acos(max(-1.0, min(1.0, -gravity[2]))) if np.isfinite(gravity).all() else float("nan")
            forces = drr.foot_forces_now(leg_model, q, tau, quat)
            arm_temp, arm_temp_j, arm_flag_j = arm_motor_health(state)
            arm_hot = heat_gate.update(now, arm_temp, arm_flag_j)
            keys = drr.parse_keys(state.wireless_remote)
            edge = keys & ~keys_prev
            keys_prev = keys
            vst = sender.status()

            # --- 止める ---
            if hard_req.is_set():
                go("DAMP", "PC から（Ctrl+C の 2 回目）")
                t_exit = now + 1.0 if t_exit is None else t_exit
            if soft_req.is_set():
                soft_req.clear()
                soft_stop("PC から（Ctrl+C の 1 回目・ssh の切断）", from_pc=True)
            cmds = []
            while not lines.empty():
                cmds.append(lines.get_nowait())
            for c in cmds:
                if c == "q" and phase in ("WAIT", "DAMP"):
                    soft_stop("PC の q", from_pc=True)
                elif c not in ("a", "d", "r") and parse_probe(c, a.probe_vx) is None:
                    soft_stop("PC の Enter", from_pc=True)
            chord = (1 << drr.KEY["L2"]) | (1 << drr.KEY["B"])
            if keys & chord == chord and edge & chord:
                go("DAMP", "リモコンの L2+B")
            elif edge & ((1 << drr.KEY["B"]) | (1 << drr.KEY["select"])):
                soft_stop("リモコンの B／select")
            if phase in ACTIVE:
                qn = float(np.linalg.norm(quat))
                if not (tilt <= max_tilt) and np.isfinite(tilt):
                    go("DAMP", f"傾き {math.degrees(tilt):.1f}°（倒れている）")
                elif now - state_t > drr.STATE_STALE_S:
                    soft_stop(f"rt/lowstate が {now - state_t:.2f} s 途切れた")
                elif not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(gyro).all() and 0.9 < qn < 1.1):
                    soft_stop(f"機体の値が壊れている（四元数の長さ {qn:.3f}）")
                elif not (tilt <= stop_tilt):
                    soft_stop(f"傾き {math.degrees(tilt):.1f}°")
                elif not (np.abs(dq).max() <= a.max_dq):
                    soft_stop(f"関節 {int(np.abs(dq).argmax())} の速さ {np.abs(dq).max():.1f} rad/s")
                elif period > drr.PERIOD_TRIP_S:
                    soft_stop(f"周期が {period * 1000:.0f} ms 空いた")
                elif len(periods) >= drr.PERIOD_AVG_N and sum(periods) / len(periods) > drr.PERIOD_AVG_TRIP_S:
                    soft_stop(f"周期の平均が {1000 * sum(periods) / len(periods):.1f} ms（50 Hz を保てない）")
                elif phase in ("DOOR", "PROBE") and vst.failed_in_row >= VEL_FAIL_TRIP:
                    soft_stop(f"SetVelocity が {vst.failed_in_row} 回続けて失敗（code={vst.last_code} {vst.last_error}）")
                elif phase in ("ARMS_IN", "READY", "DOOR", "PROBE") and vst.age_s > LOCO_STATUS_STALE_S:
                    # 方策の前でも止める（子が居ないまま d で歩かせない）。脱力は子が居なくても主から送る（LocoRelay.damp）
                    soft_stop(f"SetVelocity の中継（loco_relay.py）から状態が {vst.age_s:.1f} s 来ない")
                elif phase in ("ARMS_IN", "READY", "DOOR", "PROBE") and arm_hot:
                    soft_stop(f"腕のモーターが熱い・札が立っている（最高 {arm_temp:.0f} °C は関節 {arm_temp_j}・"
                              f"{a.arm_temp_stop:.0f} °C 以上が {heat_gate.held_s(now):.1f} 秒・札の関節 {arm_flag_j}）。冷えるまで待つ")

            # --- 段ごとの目標 ---
            out = None
            pose = None
            pol, vmsg = None, None
            odom_ok = 0.0
            try:
                if phase == "WAIT" and "a" in cmds:
                    arm_home = q[ARMS].copy()
                    arm_target = arm_home.copy()
                    if arm_io is not None:  # 腰は借りる直前の純正の角度で保つ（書かないと重み 1 で腰が脱力する）
                        arm_io.hold_waist(q[WAIST], p.kp[WAIST], p.kd[WAIST])
                    weight = 0.0
                    go("ARMS_IN", "腕を借りる")
                elif phase == "POST" and probe_vx_req(cmds) is not None:
                    probe_vx = min(probe_vx_req(cmds), a.max_vx)
                    go("PROBE", f"押す試し: 腕はこのまま・前へ {probe_vx:.2f} m/s を {a.probe_seconds:.1f} 秒")
                elif phase == "POST" and "a" in cmds:
                    go("ARMS_IN", "構え直す（腕を学習の初めの姿勢へ。借りる前の角度は覚えたまま）")
                elif phase in ("READY", "POST") and "r" in cmds:
                    go("ARMS_OUT", "腕を戻して純正へ返す")
                if phase == "ARMS_IN":
                    weight = min(1.0, weight + DT / WEIGHT_RAMP_S)
                    if weight >= 1.0:
                        arm_target = dsio.move_toward(arm_target, q_arm_goal, ARM_SPEED * DT)
                        if reached_t is None and np.abs(arm_target - q_arm_goal).max() < 1e-9:
                            reached_t = now
                        resid = float(np.abs(q[ARMS] - q_arm_goal).max())
                        if reached_t is not None and (resid < ARM_SETTLE_RAD or now - reached_t > ARM_SETTLE_S):
                            s = dc.initial_state(p)
                            load_gate = dfl.HoldLoadGate(weight=body_weight, min_total=a.min_load_total, min_each=a.min_load_each)
                            arm_hold = np.clip(q[ARMS], lo_arm + a.clip_margin, hi_arm - a.clip_margin)
                            go("READY", f"腕の差 {resid:.2f} rad（以後は届いた角度で保つ）・" + doorv.begin(now, odom, odom_t))
                elif phase == "ARMS_OUT":
                    arm_target = dsio.move_toward(arm_target, arm_home, ARM_SPEED * DT)
                    if reached_t is None and np.abs(arm_target - arm_home).max() < 1e-9:
                        reached_t = now
                    if reached_t is not None:
                        weight = max(0.0, weight - DT / WEIGHT_RAMP_S)
                        if weight <= 0.0:
                            go("WAIT", "純正へ返した")  # この周期に重み 0 を送り、以後は送らない（下の「送る」）
                if phase == "PROBE":
                    sender.set(True, probe_vx, 0.0, 0.0)  # 腕の目標は止めた時のまま（arm_target を変えない）
                    if now - phase_t >= a.probe_seconds:
                        go("POST", "押す試しの時間")
                if phase in ("READY", "DOOR", "POST", "PROBE"):
                    pose, odom_ok, vwhy = doorv.update(now, now - phase_t if phase == "READY" else 1.0, odom, odom_t, q, quat,
                                                       retry_ref=(phase == "READY"))
                    if vp is not None:
                        vwhy = feats.why_not(now)  # 画像の生徒は扉の値を画像から取る。odom は記録だけ（姿勢は READY からの動き）
                    elif pose.x > 1.5:
                        complete = 1.0
                    if phase == "DOOR" and vwhy:
                        go("POST", vwhy)
                    if phase == "DOOR" and complete:
                        go("POST", "通り抜けた（扉の先 1.5 m）")
                    if phase == "DOOR" and a.stop_door_deg is not None and doorv.door_deg() >= a.stop_door_deg:
                        go("POST", f"扉 {doorv.door_deg():.0f}°（--stop-door-deg {a.stop_door_deg:.0f}）")
                    if phase == "DOOR" and now - phase_t >= a.run_seconds:
                        go("POST", "時間")
                if phase == "READY":
                    load_gate = load_gate.push(now, forces, dq)
                    if "d" in cmds:
                        refuse = ready_refusals(tilt, max_stand_tilt, load_gate.verdict() if use_load_check else None,
                                                doorv.refuse_door(now) if vp is None else feats.why_not(now))
                        if refuse:
                            print("[door_stock] d を受け付けない: " + "・".join(refuse), flush=True)
                        else:
                            s = dc.reset_actor(p, s)
                            complete = 0.0
                            go("DOOR", f"{a.run_seconds:.1f} 秒")
                if phase in ("READY", "DOOR"):
                    inp = dc.Inputs(q=q, dq=dq, gyro=gyro, gravity=gravity, door_angle=doorv.angle,
                                    door_frame_loc=dfr.door_frame_loc(pose, quat), door_geometry=geometry, complete=complete)
                    pol, vmsg = vision_policy(vp, feats, quat, q, head_pitch, p.mirror,
                                                 p if a.vision_actor == "teacher" else None) if vp is not None else (None, None)
                    s, out = dc.step(p, s, inp, mode="door" if phase == "DOOR" else "hold", policy=pol)
                    s = dsio.with_leg_actions(p, s, q, a.leg_actions)
                    if phase == "DOOR":
                        arm_target = tg.limit_sent_target(out.q_target[ARMS], arm_target, lo_arm, hi_arm,
                                                          margin=a.clip_margin, max_step=drr.MAX_TARGET_STEP)
                        walk, vx, vy, wz = dsio.stock_velocity(p, out.cmd)
                        if walk and (vel_gain != (1.0, 1.0, 1.0) or a.vx_floor > 0):
                            vx, vy, wz = boost_stock_velocity(vx, vy, wz, vel_gain, a.vx_floor)
                        sender.set(walk, min(vx, a.max_vx), vy, wz)
                    else:
                        arm_target = (arm_hold if arm_hold is not None else q_arm_goal).copy()
            except Exception as e:  # noqa: BLE001  方策の計算の誤りでは脱力せず、立ったまま腕を保つ
                print(f"[door_stock] 計算で例外: {type(e).__name__}: {e}", flush=True)
                out = None
                if phase in ("ARMS_IN", "READY", "DOOR", "ARMS_OUT", "PROBE"):
                    go("POST", f"例外 {type(e).__name__}")

            # --- 送る（腕だけ。歩きは送る糸が送る） ---
            sent_arm = arm_target
            if phase == "DAMP":
                if damp_left > 0 and arm_io is not None:
                    arm_io.write(q[ARMS] if np.isfinite(q).all() else q_arm_goal, kp_arm, kd_arm, 0.0)  # 腕も純正へ返す
                damp_left = max(0, damp_left - 1)
                weight, arm_target = 0.0, None
            elif arm_target is not None and np.isfinite(arm_target).all():
                if a.roll_press_max > 0 and phase in ("DOOR", "POST") and np.isfinite(q[ARMS]).all():
                    sent_arm = cap_roll_press(arm_target, q[ARMS], kp_arm, a.roll_press_max)
                if arm_io is not None:
                    arm_io.write(sent_arm, kp_arm, kd_arm, weight)
                if phase == "WAIT":
                    arm_target = None  # 重み 0 を送り終えた。以後は純正が腕を持つ
            elif arm_target is not None:
                go("POST", "腕の目標が NaN")

            # --- 記録 ---
            loop_dt = time.monotonic() - now
            row = dict(t=now, phase=PHASES.index(phase), q=q, dq=dq, tau=tau, quat=quat, gyro=gyro, keys=keys,
                       arm_sent=sent_arm if arm_target is not None else np.full(dc.NUM_DOF - dc.NUM_HOMIE, np.nan),
                       arm_weight=weight, loop_dt=loop_dt, period=period,
                       vel_want=np.array([float(vst.want[0]), vst.want[1], vst.want[2], vst.want[3]]),
                       vel_code=np.nan if vst.last_code is None else vst.last_code, vel_rpc_ms=vst.last_rpc_ms,
                       vel_sent=vst.sent, vel_status_age=vst.age_s, odom=np.array(odom) if odom is not None else np.full(3, np.nan),
                       odom_age=(now - odom_t) if odom_t is not None else np.inf, odom_ok=odom_ok,
                       foot_fz=np.array([forces.fz_left, forces.fz_right]), door_angle_obs=doorv.angle,
                       arm_temp=arm_temps(state))  # 11-04: 温度の山か上がり続けかを記録で見分ける
            if vp is not None:
                row.update(vision_row(vp, out, vmsg, pol, now))
            if lidar is not None:
                lm, lrx = lidar.snapshot()
                row.update(lidar_msg=np.array([lm.stamp, lm.x, lm.y, lm.yaw, lm.angle, lm.state, lm.rms_m, lm.tilt_deg, lm.seq])
                           if lm is not None else np.full(9, np.nan), lidar_rx_age=(now - lrx) if lm is not None else np.inf)
            if out is not None:
                row.update(obs=out.obs, a_pol=out.a_pol, D=s.D, cmd=out.cmd, homie_a=out.homie_a, walk=out.walk,
                           q_policy=out.q_target, pose=np.array([pose.x, pose.y, pose.yaw]))
            else:
                row.update(obs=np.full(p.obs_dim, np.nan), a_pol=np.full(dc.NUM_ACT, np.nan), D=np.full(dc.NUM_ACT, np.nan),
                           cmd=np.full(7, np.nan), homie_a=np.full(dc.NUM_HOMIE, np.nan), walk=np.nan,
                           q_policy=np.full(dc.NUM_DOF, np.nan),
                           pose=np.array([pose.x, pose.y, pose.yaw]) if pose is not None else np.full(3, np.nan))
            log.add(**row)

            if now - last_print > 0.5:
                last_print = now
                print(f"[door_stock] {phase:8s} 傾き {math.degrees(tilt):4.1f}° |dq| {np.abs(dq).max():4.1f} "
                      + dfl.format_load(forces.fz_left, forces.fz_right, body_weight)
                      + f" 腕の重み {weight:.2f} 歩き {'歩く' if vst.want[0] else '立つ'} "
                      + f"{np.round(vst.want[1:], 2).tolist()} RPC {vst.last_rpc_ms:.0f} ms code {vst.last_code} "
                      + (f"姿勢 ({pose.x:+.2f},{pose.y:+.2f},{math.degrees(pose.yaw):+.0f}°) " if pose is not None else "")
                      + (f"扉 {doorv.door_deg():.0f}° " if lidar is not None else "")
                      + (vision_text(p, vp, out, vmsg, pol, now) if vp is not None else "")
                      + f"周期 {period * 1000:.1f} ms（計算 {loop_dt * 1000:.1f} ms） 腕 最高 {arm_temp:.0f} °C（関節 {arm_temp_j}）"
                      + (f"・⚠️ 札 関節 {arm_flag_j}" if arm_flag_j is not None else ""), flush=True)
            if t_exit is not None and now >= t_exit:
                break

            next_t += DT
            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()
    finally:
        gc.enable()
        sender.close()
        if feats is not None:
            feats.close()
        stop_odom_relay(odom_rx, relay)
        # 腕を借りたまま輪の外へ出た（輪の外側の例外）ときは、今の目標を保ったまま重みを 2 秒で 0 へ（純正へ返す）
        if arm_io is not None and arm_target is not None and weight > 0.0 and phase != "DAMP":
            print("[door_stock] 腕を借りたまま終わる: 重みを 2 秒で 0 へ戻す", flush=True)
            while weight > 0.0:
                weight = max(0.0, weight - DT / WEIGHT_RAMP_S)
                try:
                    arm_io.write(arm_target, kp_arm, kd_arm, weight)
                except Exception as e:  # noqa: BLE001  返すことを優先する
                    print(f"[door_stock] arm_sdk の送信に失敗: {type(e).__name__}: {e}", flush=True)
                time.sleep(DT)
        try:
            log.save(log_dir / f"door_stock_{stamp}.npz", dict(meta, reason=reason))
        except Exception as e:  # noqa: BLE001
            print(f"[door_stock] 記録を書けなかった: {type(e).__name__}: {e}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
