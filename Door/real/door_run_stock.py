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
                          周期の遅れ・方策の計算で例外・SetVelocity が続けて失敗・ライダーの見張り・odom の途切れ（tape）
  脱力              リモコンの L2+B（純正のもの。こちらも Damp を送る）・Ctrl+C の 2 回目・傾き > --max-tilt-deg
  プログラムが落ちたとき: arm_sdk が途切れると純正はその時の形で腕を保ち（10-27 に pkill -9 で確認）、速度は 0.5 秒で切れる
  WAIT と DAMP では q・Enter・Ctrl+C で終わる

例（PC2。機体は吊って綱を緩め、リモコンで通常運転 501 に入れておく）:
  python3 door_run_stock.py --policy door_policy.npz --iface eth0 --door-source lidar --lidar-center-xy 0.047 -0.016 \\
      --door 0.94,2.05,0.99,0.064 --run-seconds 5
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
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_foot_load as dfl  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_lidar_msg as dlm  # noqa: E402
import door_run_real as drr  # noqa: E402  受け取り（RobotIO）・リモコン・記録・足の力は HOMIE の経路と同じものを使う
import door_stock_io as dsio  # noqa: E402
import target_guard as tg  # noqa: E402

DT = drr.DT
ARMS = slice(dc.NUM_HOMIE, dc.NUM_DOF)
WEIGHT_RAMP_S = 2.0  # arm_sdk の重みを 0 ↔ 1 にかける時間（armsdk_probe.py・遠隔操作と同じ）
ARM_SPEED = 1.0  # [rad/s] 腕を構える・戻す速さ
ARM_SETTLE_RAD = 0.1  # 構えた腕がこれ以内に落ち着いたら READY
ARM_SETTLE_S = 2.0  # 落ち着かなくてもこれだけ待ったら READY（残りの差を表示する）
VEL_FAIL_TRIP = 5  # SetVelocity が続けてこれだけ失敗したら立ったまま止まる（20 回/秒で 0.25 秒）
DAMP_RELEASE_CYCLES = 5  # 脱力のとき重み 0 を送る周期の数（腕も純正へ返して Damp を効かせる）
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

    def update(self, now: float, ready_s: float, odom, odom_t, q: np.ndarray, quat: np.ndarray) -> Tuple[dfr.PlanarPose, float, str]:
        """(今の pelvis の姿勢, odom が新しいか, 見張りの理由（空なら無事）)"""
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
        if self.ref_t is not None and now - self.ref_t > LIDAR_REF_RETRY_S and not self.lidar.ready(now):
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
    ap.add_argument("--odom", choices=("dog", "none"), default="dog")
    ap.add_argument("--clip-margin", type=float, default=0.05, help="腕の目標を可動域の端から内側へ切る余裕 [rad]")
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
    ap.add_argument("--no-send", action="store_true", help="arm_sdk も SetVelocity も送らない（読むだけ・計算と記録はする）")
    ap.add_argument("--door-source", choices=("tape", "lidar"), default="lidar")
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


def check_stock_running(client) -> str:
    """純正が歩ける状態か。だめなら理由（空なら起動してよい）"""
    name = drr.motion_mode_name()
    if not name:
        return "純正の歩行が動いていない（このプログラムは純正の脚で歩く。リモコンで立たせて通常運転 501 に入れる）"
    fsm = dsio.fsm_id(client)
    if fsm not in dsio.WALKING_FSM_IDS:
        return f"FSM={fsm} は通常運転（500/501）ではない（指令が通っても足が動かない・arm_sdk が効かない）。リモコンで通常運転へ"
    return ""


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
    w, h, lh, bs = (float(v) for v in a.door.split(","))
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
    state0 = io.snapshot()[0]
    drr.check_29dof(state0)
    loco = _NullLoco() if a.no_send else dsio.make_loco_client(a.rpc_timeout)
    if not a.no_send:
        why = check_stock_running(loco)
        if why:
            print(f"[door_stock] 起動しない: {why}")
            return 1
    arm_io = None if a.no_send else dsio.ArmSdkWriter(state0.mode_machine)
    sender = dsio.VelocitySender(loco)
    sender.start()
    print(f"[door_stock] 29 関節・送る={'しない' if a.no_send else 'する'}・扉 {geometry.tolist()}・鏡写し {p.mirror}・"
          f"扉の値 {a.door_source}{'（' + a.lidar_mode + '）' if lidar else ''}・脚の観測 {a.leg_actions}・前の上限 {a.max_vx} m/s"
          + (f"・足の確かめ 合計 {100 * a.min_load_total:.0f}%・片足 {100 * a.min_load_each:.0f}%" if use_load_check else "・⚠️ 足の確かめ off"),
          flush=True)
    print(f"[door_stock] 方策の空回し: {drr.warm_up(p, state0, geometry, start_pose):.0f} ms", flush=True)
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
    meta = dict(args=vars(a), phases=PHASES, program="door_run_stock")
    phase, phase_t, reason = "WAIT", time.monotonic(), ""
    keys_prev = drr.parse_keys(state0.wireless_remote)
    s = dc.initial_state(p)
    load_gate = dfl.HoldLoadGate(weight=body_weight, min_total=a.min_load_total, min_each=a.min_load_each)
    weight = 0.0
    arm_target: Optional[np.ndarray] = None  # 送っている腕の目標（None は送っていない）
    arm_home: Optional[np.ndarray] = None  # 借りる前の腕の角度（返すときに戻す）
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
        nonlocal phase, phase_t, reason, reached_t, damp_left
        if phase == "DAMP" or phase == new:
            return
        print(f"[door_stock] {phase} → {new}" + (f"（{why}）" if why else ""), flush=True)
        phase, phase_t, reached_t = new, time.monotonic(), None
        if new in ("POST", "ARMS_OUT", "DAMP"):
            sender.stop()
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
            q = np.array([state.motor_state[j].q for j in range(dc.NUM_DOF)])
            dq = np.array([state.motor_state[j].dq for j in range(dc.NUM_DOF)])
            tau = np.array([state.motor_state[j].tau_est for j in range(dc.NUM_DOF)])
            quat = np.array(state.imu_state.quaternion, dtype=float)  # w, x, y, z
            gyro = np.array(state.imu_state.gyroscope, dtype=float)
            gravity = dc.quat_wxyz_to_gravity(quat)
            tilt = math.acos(max(-1.0, min(1.0, -gravity[2]))) if np.isfinite(gravity).all() else float("nan")
            forces = drr.foot_forces_now(leg_model, q, tau, quat)
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

            # --- 段ごとの目標 ---
            out = None
            pose = None
            odom_ok = 0.0
            try:
                if phase == "WAIT" and "a" in cmds:
                    arm_home = q[ARMS].copy()
                    arm_target = arm_home.copy()
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
                            go("READY", f"腕の差 {resid:.2f} rad・" + doorv.begin(now, odom, odom_t))
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
                    pose, odom_ok, vwhy = doorv.update(now, now - phase_t if phase == "READY" else 1.0, odom, odom_t, q, quat)
                    if pose.x > 1.5:
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
                                                doorv.refuse_door(now))
                        if refuse:
                            print("[door_stock] d を受け付けない: " + "・".join(refuse), flush=True)
                        else:
                            s = dc.reset_actor(p, s)
                            complete = 0.0
                            go("DOOR", f"{a.run_seconds:.1f} 秒")
                if phase in ("READY", "DOOR"):
                    inp = dc.Inputs(q=q, dq=dq, gyro=gyro, gravity=gravity, door_angle=doorv.angle,
                                    door_frame_loc=dfr.door_frame_loc(pose, quat), door_geometry=geometry, complete=complete)
                    s, out = dc.step(p, s, inp, mode="door" if phase == "DOOR" else "hold")
                    s = dsio.with_leg_actions(p, s, q, a.leg_actions)
                    if phase == "DOOR":
                        arm_target = tg.limit_sent_target(out.q_target[ARMS], arm_target, lo_arm, hi_arm,
                                                          margin=a.clip_margin, max_step=drr.MAX_TARGET_STEP)
                        walk, vx, vy, wz = dsio.stock_velocity(p, out.cmd)
                        sender.set(walk, min(vx, a.max_vx), vy, wz)
                    else:
                        arm_target = q_arm_goal.copy()
            except Exception as e:  # noqa: BLE001  方策の計算の誤りでは脱力せず、立ったまま腕を保つ
                print(f"[door_stock] 計算で例外: {type(e).__name__}: {e}", flush=True)
                out = None
                if phase in ("ARMS_IN", "READY", "DOOR", "ARMS_OUT", "PROBE"):
                    go("POST", f"例外 {type(e).__name__}")

            # --- 送る（腕だけ。歩きは送る糸が送る） ---
            if phase == "DAMP":
                if damp_left > 0 and arm_io is not None:
                    arm_io.write(q[ARMS] if np.isfinite(q).all() else q_arm_goal, kp_arm, kd_arm, 0.0)  # 腕も純正へ返す
                damp_left = max(0, damp_left - 1)
                weight, arm_target = 0.0, None
            elif arm_target is not None and np.isfinite(arm_target).all():
                if arm_io is not None:
                    arm_io.write(arm_target, kp_arm, kd_arm, weight)
                if phase == "WAIT":
                    arm_target = None  # 重み 0 を送り終えた。以後は純正が腕を持つ
            elif arm_target is not None:
                go("POST", "腕の目標が NaN")

            # --- 記録 ---
            loop_dt = time.monotonic() - now
            row = dict(t=now, phase=PHASES.index(phase), q=q, dq=dq, tau=tau, quat=quat, gyro=gyro, keys=keys,
                       arm_sent=arm_target if arm_target is not None else np.full(dc.NUM_DOF - dc.NUM_HOMIE, np.nan),
                       arm_weight=weight, loop_dt=loop_dt, period=period,
                       vel_want=np.array([float(vst.want[0]), vst.want[1], vst.want[2], vst.want[3]]),
                       vel_code=np.nan if vst.last_code is None else vst.last_code, vel_rpc_ms=vst.last_rpc_ms,
                       vel_sent=vst.sent, odom=np.array(odom) if odom is not None else np.full(3, np.nan),
                       odom_age=(now - odom_t) if odom_t is not None else np.inf, odom_ok=odom_ok,
                       foot_fz=np.array([forces.fz_left, forces.fz_right]), door_angle_obs=doorv.angle)
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
                      + f"周期 {period * 1000:.1f} ms（計算 {loop_dt * 1000:.1f} ms）", flush=True)
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
