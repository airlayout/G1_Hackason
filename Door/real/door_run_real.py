"""段 5: 実機の G1 で、状態入力の生徒（段 2.5）と HOMIE の脚を回す。最初は 5 秒だけ・ウィンチで吊ったまま。

PC2 で動かす（unitree_sdk2py と numpy だけ。Python 3.8 でも動く）。rt/lowcmd を書くのはこのプロセス 1 本だけ。
計算は door_core.py（Isaac Sim の正解の記録と一致を確かめ済み: check_golden.py）。

流れ（リモコン）:
  WAIT   脱力（kp=0・kd のみ）で待つ。start で次へ
  RAMP   3 秒かけて今の角度から学習の初めの姿勢（脚は少し曲げる・腕は 0＝真下）へ
  HOLD   その姿勢を PD で保つ。ここでウィンチを下ろして足を床に着ける。A で次へ
  STAND  HOMIE だけで立つ（指令 0・腕は 0）。脚だけを見る。2 秒以上たってから A で次へ
  DOOR   生徒と HOMIE（--run-seconds 秒、既定 5）
  POST   生徒を止める。HOMIE は指令 0 で立ったまま、腕はその場
  FREEZE HOMIE が動く前（RAMP・HOLD）に止めたとき。最後の目標角を PD で固く保つ
  DAMP   脱力

止め方は 2 段（2026-10-16 ユーザー判断。脱力で床に崩れたことがあるので、ふだんの停止では脱力しない）:
  立ったまま止まる  リモコンの B か select／PC の Enter／Ctrl+C の 1 回目／ssh の切断（SIGHUP）
                    → STAND・DOOR からは POST（HOMIE で立つ）、RAMP・HOLD からは FREEZE（今の姿勢を保つ）
                    自動: 1 周期が 60 ms 空く・0.5 秒の周期の平均が 25 ms 超・DOOR で odom が 0.5 s 途切れる・方策の計算で例外
  脱力              リモコンの L2+B／Ctrl+C の 2 回目（脱力を 1 秒送って終わる）
                    自動（制御できない・倒れている）: rt/lowstate が 0.1 s 途切れる・機体の値が NaN／四元数が壊れている・
                    傾き > --max-tilt-deg・関節の速さ > --max-dq・目標が NaN
  WAIT と DAMP では Ctrl+C（と Enter）で終わる
  ⚠️ HOMIE 自体が実機で不安定なら「立ったまま止まる」でも立てない（L2+B で脱力し、ウィンチで受ける）。
     電源断・SIGKILL では何も送れない。tmux の中で起こし、最初のうちはウィンチを外さない
start・A は、全部のボタンを離した後の押しだけを受け付ける（起動時に押しっぱなしでも進まない）

扉の値（最初の版）:
  枠の位置  巻き尺で測った最初の pelvis の姿勢（--start-x/-y/-yaw-deg）に、STAND に入ってからの odom（rt/dog_odom）の動きを足す
  扉の角度  閉じている前提の値（角度 0・測れた）。⚠️ 5 秒の試しで扉が開き始めても生徒には 0 のまま（LiDAR の推定は次の版）
  寸法      --door（幅,高さ,レバーの高さ,バックセット [m]）

記録: --log-dir に door_real_<日時>.npz（毎周期の lowstate・観測・行動・目標・扉の値・周期の時間）

例（PC2）:
  python3 door_run_real.py --policy door_policy_it45150.npz --iface eth0 \\
      --start-x=-1.00 --start-y=0.00 --start-yaw-deg=0 --door 0.94,2.05,1.00,0.064
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
if Path("/home/unitree/unitree_sdk2_python").is_dir():  # PC2 の SDK は site-packages に入っていない（record_dds_to_bag.py と同じ）
    sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402

DT = 0.02  # 50 Hz（学習と同じ）
RAMP_S = 3.0
STAND_MIN_S = 2.0
STATE_STALE_S = 0.1
ODOM_STALE_S = 0.2
ODOM_LOST_S = 0.5  # DOOR で odom がこれ以上途切れたら立ったまま止まる（枠の位置が分からない）
STAND_BLEND_S = 0.5  # STAND の初めに既定の姿勢から HOMIE の目標へ移す時間
PERIOD_TRIP_S = 0.06  # 1 周期がこれ以上空いたら立ったまま止まる
PERIOD_AVG_N = 25
PERIOD_AVG_TRIP_S = 0.025  # 0.5 秒の平均がこれを超えたら立ったまま止まる（50 Hz を保てていない）
ARM_CYCLES = 10  # start・A は、全部のボタンが 10 周期離れた後の押しだけ受け付ける
DAMP_KD = 8.0  # unitree_rl_gym の deploy_real と同じ
MAX_TARGET_STEP = 0.7  # [rad/周期] シミュレータの最大 0.63（10-16 の正解の記録）。暴れた出力だけを止める
N_MOTOR_CMD = 35

# リモコン（wireless_remote[2:4] の 16 ビット。unitree_rl_gym の KeyMap と同じ）
KEY = {"R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5, "F1": 6, "F2": 7,
       "A": 8, "B": 9, "X": 10, "Y": 11, "up": 12, "right": 13, "down": 14, "left": 15}


def parse_keys(wireless_remote) -> int:
    return int(wireless_remote[2]) | (int(wireless_remote[3]) << 8)


def pressed(keys: int, name: str) -> bool:
    return bool(keys >> KEY[name] & 1)


class RobotIO:
    """rt/lowstate・rt/dog_odom を受け、rt/lowcmd を送る。送るのはこのクラスだけ"""

    def __init__(self, iface: str, send: bool, use_odom: bool) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(0, iface)
        self.send_enabled = send
        self.crc = CRC()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.state = None
        self.state_t = None
        self.odom = None  # (x, y, yaw)
        self.odom_t = None
        self.odom_count = 0
        self._lock = threading.Lock()
        self.pub = None
        if send:
            self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        self.odom_sub = None
        if use_odom:
            from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

            self.odom_sub = ChannelSubscriber("rt/dog_odom", Odometry_)
            self.odom_sub.Init(self._on_odom, 1)

    def _on_state(self, msg) -> None:
        with self._lock:
            self.state = msg
            self.state_t = time.monotonic()

    def _on_odom(self, msg) -> None:
        self.odom_count += 1
        if self.odom_count % 10:  # 1008 Hz を約 100 Hz に間引く（重い処理はしない）
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = dfr.yaw_from_quat_wxyz(np.array([o.w, o.x, o.y, o.z]))
        with self._lock:
            self.odom = (float(p.x), float(p.y), yaw)
            self.odom_t = time.monotonic()

    def snapshot(self):
        with self._lock:
            return self.state, self.state_t, self.odom, self.odom_t

    def write(self, state, q: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
        if not self.send_enabled:
            return
        self.cmd.mode_pr = 0  # 足首を pitch/roll の関節として扱う（学習と同じ）
        self.cmd.mode_machine = state.mode_machine
        for j in range(N_MOTOR_CMD):
            m = self.cmd.motor_cmd[j]
            m.mode = 1
            m.dq = 0.0
            m.tau = 0.0
            if j < dc.NUM_DOF:
                m.q, m.kp, m.kd = float(q[j]), float(kp[j]), float(kd[j])
            else:
                m.q, m.kp, m.kd = 0.0, 0.0, 0.0
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)


def motion_mode_name() -> str:
    """純正の歩行（モーションコントローラ）の名前。照会に失敗したら例外（黙って「止まっている」とみなさない）"""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, result = msc.CheckMode()
    if code != 0:
        raise RuntimeError(f"MotionSwitcher.CheckMode が失敗（code={code}）")
    return (result or {}).get("name", "") if isinstance(result, dict) else ""


def check_29dof(state) -> None:
    live = [j for j in range(dc.NUM_DOF) if state.motor_state[j].temperature[0] > 0 or state.motor_state[j].vol > 0.1]
    if len(live) != dc.NUM_DOF:
        missing = sorted(set(range(dc.NUM_DOF)) - set(live))
        raise RuntimeError(f"29 関節のうち応答の無いモータ: {missing}（23 関節の機体か電源を確かめる）")


class Logger:
    def __init__(self) -> None:
        self.rows: Dict[str, List[np.ndarray]] = {}

    def add(self, **kw) -> None:
        for k, v in kw.items():
            self.rows.setdefault(k, []).append(np.asarray(v, dtype=float))

    def save(self, path: Path, meta: dict) -> None:
        Logger.save_rows(self.rows, path, meta)

    @staticmethod
    def save_rows(rows: Dict[str, List[np.ndarray]], path: Path, meta: dict) -> None:
        if not rows:
            return
        n = min(len(v) for v in rows.values())
        np.savez_compressed(path, meta=np.array(repr(meta)), **{k: np.stack(v[:n]) for k, v in rows.items()})
        print(f"[door_real] 記録を書いた: {path}（{n} 周期）", flush=True)


PHASES = ("WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "FREEZE", "DAMP")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True, help="export_door_policy.py が書いた npz")
    ap.add_argument("--iface", default="eth0")
    # 扉の座標での pelvis の最初の姿勢（開口の中心の床から。手前は x が負）。負の数は --start-x=-1.0 の形で渡す
    ap.add_argument("--start-x", type=float, required=True, help="[m] 扉の板の面から前後（手前は負）")
    ap.add_argument("--start-y", type=float, required=True, help="[m] 開口の中心から左右（扉を向いて左が正）")
    ap.add_argument("--start-yaw-deg", type=float, default=0.0, help="[deg] 扉を正面に向いて 0、左回りが正")
    ap.add_argument("--door", required=True, help="幅,高さ,レバーの高さ,バックセット [m]")
    ap.add_argument("--lever-right", action="store_true", help="把手が右の扉（UIS Room 4 は左なので付けない）")
    ap.add_argument("--run-seconds", type=float, default=5.0)
    ap.add_argument("--odom", choices=("dog", "none"), default="dog", help="none だと枠の位置は最初の姿勢のまま動かない")
    ap.add_argument("--clip-margin", type=float, default=0.05, help="目標を可動域の端から内側へ切る余裕 [rad]")
    ap.add_argument("--max-tilt-deg", type=float, default=30.0, help="シミュレータの最大は 12.4°")
    ap.add_argument("--max-dq", type=float, default=20.0, help="[rad/s] シミュレータの最大は 11.7")
    ap.add_argument("--no-send", action="store_true", help="rt/lowcmd を送らない（読むだけ。純正の歩行のままでもよい）")
    ap.add_argument("--log-dir", default="logs")
    a = ap.parse_args()

    p = dc.load_params(a.policy, mirror=not a.lever_right)
    start_pose = dfr.PlanarPose(a.start_x, a.start_y, math.radians(a.start_yaw_deg))
    w, h, lh, bs = (float(v) for v in a.door.split(","))
    geometry = dfr.door_geometry(w, h, lh, bs, lever_left=not a.lever_right)
    q_lo, q_hi = p.q_lower + a.clip_margin, p.q_upper - a.clip_margin
    damp_kp, damp_kd = np.zeros(dc.NUM_DOF), np.full(dc.NUM_DOF, DAMP_KD)
    max_tilt = math.radians(a.max_tilt_deg)

    io = RobotIO(a.iface, send=not a.no_send, use_odom=(a.odom == "dog"))
    t_wait = time.monotonic()
    while io.snapshot()[0] is None:
        if time.monotonic() - t_wait > 5.0:
            print("[door_real] rt/lowstate が来ない（--iface と機体の電源を確かめる）")
            return 1
        time.sleep(0.05)
    state0 = io.snapshot()[0]
    check_29dof(state0)
    if not a.no_send:
        name = motion_mode_name()
        if name:
            print(f"[door_real] 純正の歩行 '{name}' が動いている。リモコンで止めてから（L2+B で脱力 → L2+R2 でデバッグ）")
            return 1
    print(f"[door_real] 29 関節・mode_machine={state0.mode_machine}・送る={'する' if not a.no_send else 'しない'}・"
          f"最初の姿勢 {start_pose}・扉 {geometry.tolist()}・鏡写し {p.mirror}")

    soft_req = threading.Event()  # 立ったまま止まる（Enter・Ctrl+C の 1 回目・SIGHUP）
    hard_req = threading.Event()  # 脱力して終わる（Ctrl+C の 2 回目）
    quit_count = [0]

    def on_quit(*_) -> None:
        quit_count[0] += 1
        (soft_req if quit_count[0] == 1 else hard_req).set()

    signal.signal(signal.SIGINT, on_quit)
    signal.signal(signal.SIGTERM, on_quit)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, lambda *_: soft_req.set())  # ssh が切れても終わらない（終わると誰も支えない）

    def watch_enter() -> None:
        while True:
            line = sys.stdin.readline()
            if not line:  # EOF＝標準入力が無い（nohup など）。止めない
                return
            soft_req.set()

    threading.Thread(target=watch_enter, daemon=True).start()

    log = Logger()
    log_dir = Path(a.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    meta = dict(args=vars(a), phases=PHASES)
    phase = "WAIT"
    phase_t = time.monotonic()
    keys_prev = parse_keys(state0.wireless_remote)  # 起動時に押されていたボタンを「押した」と取らない
    released_cycles = 0  # 全部のボタンが離れている周期の数。ARM_CYCLES 続くまで start・A を受け付けない
    q_ramp0 = None
    s = dc.initial_state(p)
    odom_ref: Optional[dfr.PlanarPose] = None
    last_pose: Optional[dfr.PlanarPose] = None
    odom_lost_t: Optional[float] = None
    complete = 0.0
    last_target: Optional[np.ndarray] = None
    freeze_target: Optional[np.ndarray] = None
    periods: List[float] = []
    reason = ""
    next_t = time.monotonic()
    prev_now = None
    last_print = 0.0
    t_exit = None

    def go(new: str, why: str = "") -> None:
        nonlocal phase, phase_t, reason, freeze_target
        if phase == "DAMP" or phase == new:
            return
        print(f"[door_real] {phase} → {new}" + (f"（{why}）" if why else ""), flush=True)
        if new == "FREEZE":
            freeze_target = last_target.copy() if last_target is not None else None
        phase, phase_t = new, time.monotonic()
        if new == "RAMP":
            periods.clear()
        if new in ("POST", "FREEZE", "DAMP"):
            reason = why
        if new == "DAMP":
            # SIGKILL や電源断でも記録が残るように、脱力に入った時点で別スレッドで一度書く
            snap = {k: list(v) for k, v in log.rows.items()}
            threading.Thread(target=Logger.save_rows, args=(snap, log_dir / f"door_real_{stamp}_damp.npz",
                                                            dict(meta, reason=why)), daemon=True).start()

    def soft_stop(why: str, from_pc: bool = False) -> None:
        """立ったまま止まる。HOMIE が動いていれば POST、動く前なら FREEZE。WAIT・DAMP では PC からのときだけ終わる"""
        nonlocal t_exit
        if phase in ("STAND", "DOOR"):
            go("POST", why)
        elif phase in ("RAMP", "HOLD"):
            go("FREEZE", why)
        elif from_pc and phase in ("WAIT", "DAMP") and t_exit is None:
            print(f"[door_real] 終わる（{why}）", flush=True)
            t_exit = time.monotonic() + 0.2

    try:
        while True:
            now = time.monotonic()
            period = (now - prev_now) if prev_now is not None else DT
            prev_now = now
            periods.append(period)
            del periods[:-PERIOD_AVG_N]
            state, state_t, odom, odom_t = io.snapshot()
            q = np.array([state.motor_state[j].q for j in range(dc.NUM_DOF)])
            dq = np.array([state.motor_state[j].dq for j in range(dc.NUM_DOF)])
            tau = np.array([state.motor_state[j].tau_est for j in range(dc.NUM_DOF)])
            quat = np.array(state.imu_state.quaternion, dtype=float)  # w, x, y, z
            gyro = np.array(state.imu_state.gyroscope, dtype=float)
            gravity = dc.quat_wxyz_to_gravity(quat)
            tilt = math.acos(max(-1.0, min(1.0, -gravity[2]))) if np.isfinite(gravity).all() else float("nan")
            keys = parse_keys(state.wireless_remote)
            edge = keys & ~keys_prev
            keys_prev = keys
            ready = released_cycles >= ARM_CYCLES  # この押しの前に、全部のボタンが ARM_CYCLES 周期離れていたか
            released_cycles = released_cycles + 1 if keys == 0 else 0
            go_keys = edge if ready else 0  # start・A は、離した後の押しだけ受け付ける（B・select・L2+B はいつでも）

            # --- 止める ---
            if hard_req.is_set():
                go("DAMP", "PC から（Ctrl+C の 2 回目）")
                t_exit = now + 1.0 if t_exit is None else t_exit
            if soft_req.is_set():
                soft_req.clear()
                soft_stop("PC から（Enter・Ctrl+C の 1 回目・ssh の切断）", from_pc=True)
                if phase in ("POST", "FREEZE"):
                    print("[door_real] 立ったまま止めた。脱力はリモコンの L2+B か、もう一度 Ctrl+C（⚠️ 吊っていないと崩れる）", flush=True)
            chord = (1 << KEY["L2"]) | (1 << KEY["B"])
            if keys & chord == chord and edge & chord:
                go("DAMP", "リモコンの L2+B")
            elif edge & ((1 << KEY["B"]) | (1 << KEY["select"])):
                soft_stop("リモコンの B／select")
            if phase in ("RAMP", "HOLD", "STAND", "DOOR", "POST", "FREEZE"):
                qn = float(np.linalg.norm(quat))
                if now - state_t > STATE_STALE_S:
                    go("DAMP", f"rt/lowstate が {now - state_t:.2f} s 途切れた")
                elif not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(gyro).all() and 0.9 < qn < 1.1):
                    go("DAMP", f"機体の値が壊れている（四元数の長さ {qn:.3f}）")
                elif not (tilt <= max_tilt):
                    go("DAMP", f"傾き {math.degrees(tilt):.1f}°")
                elif not (np.abs(dq).max() <= a.max_dq):
                    go("DAMP", f"関節 {int(np.abs(dq).argmax())} の速さ {np.abs(dq).max():.1f} rad/s")
                elif period > PERIOD_TRIP_S:
                    soft_stop(f"周期が {period * 1000:.0f} ms 空いた")
                elif len(periods) >= PERIOD_AVG_N and sum(periods) / len(periods) > PERIOD_AVG_TRIP_S:
                    soft_stop(f"周期の平均が {1000 * sum(periods) / len(periods):.1f} ms（50 Hz を保てない）")

            # --- 段ごとの目標 ---
            target, kp, kd = None, damp_kp, damp_kd
            out = None
            pose = None
            odom_ok = 0.0
            try:
                if phase == "WAIT" and go_keys & (1 << KEY["start"]):
                    q_ramp0 = q.copy()
                    last_target = q_ramp0.copy()  # 変化の上限を今の角度から効かせる
                    go("RAMP")
                if phase == "RAMP":
                    r = min(1.0, (now - phase_t) / RAMP_S)
                    r = r * r * (3.0 - 2.0 * r)
                    target, kp, kd = (1.0 - r) * q_ramp0 + r * p.q_default, p.kp, p.kd
                    if r >= 1.0:
                        go("HOLD")
                elif phase == "HOLD":
                    target, kp, kd = p.q_default.copy(), p.kp, p.kd
                    if go_keys & (1 << KEY["A"]):
                        s = dc.initial_state(p)
                        fresh = odom is not None and odom_t is not None and now - odom_t < ODOM_STALE_S
                        odom_ref = dfr.PlanarPose(*odom) if fresh else None
                        last_pose, odom_lost_t = start_pose, None
                        go("STAND", "odom の基準を取った" if odom_ref else "⚠️ odom 無し（枠の位置は最初の姿勢のまま動かない）")
                if phase in ("STAND", "DOOR", "POST"):
                    if odom_ref is not None and odom is not None and now - odom_t < ODOM_STALE_S:
                        pose, odom_ok, odom_lost_t = dfr.compose_planar(start_pose, odom_ref, dfr.PlanarPose(*odom)), 1.0, None
                    else:
                        pose = last_pose  # odom が途切れたら最後の姿勢を保つ（最初の姿勢へ跳ばない）
                        if odom_ref is not None and odom_lost_t is None:
                            odom_lost_t = now
                    last_pose = pose
                    if pose.x > 1.5:
                        complete = 1.0
                    inp = dc.Inputs(q=q, dq=dq, gyro=gyro, gravity=gravity, door_angle=dfr.door_closed_angle_obs(),
                                    door_frame_loc=dfr.door_frame_loc(pose, quat), door_geometry=geometry, complete=complete)
                    if phase == "STAND" and go_keys & (1 << KEY["A"]) and now - phase_t >= STAND_MIN_S:
                        s = dc.reset_actor(p, s)
                        go("DOOR", f"{a.run_seconds:.1f} 秒")
                    if phase == "DOOR" and now - phase_t >= a.run_seconds:
                        go("POST", "時間")
                    if phase == "DOOR" and odom_lost_t is not None and now - odom_lost_t > ODOM_LOST_S:
                        go("POST", f"odom が {now - odom_lost_t:.1f} s 途切れた（枠の位置が分からない）")
                    if phase in ("STAND", "DOOR", "POST"):
                        s, out = dc.step(p, s, inp, mode="door" if phase == "DOOR" else "hold")
                        target, kp, kd = out.q_target, p.kp, p.kd
                        if phase == "STAND" and now - phase_t < STAND_BLEND_S:
                            # HOMIE の最初の出力は空の履歴から出るので、0.5 秒かけて既定の姿勢から移す（シミュレータでは 0.07〜0.11 rad）
                            r = (now - phase_t) / STAND_BLEND_S
                            target = (1.0 - r) * p.q_default + r * target
            except Exception as e:  # noqa: BLE001  方策の計算の誤りでは脱力せず、最後の目標を保つ
                print(f"[door_real] 目標の計算で例外: {type(e).__name__}: {e}", flush=True)
                out = None
                if phase in ("RAMP", "HOLD", "STAND", "DOOR", "POST"):
                    go("FREEZE", f"例外 {type(e).__name__}")
                target = None
            if phase == "FREEZE":
                if freeze_target is None:
                    go("DAMP", "保つ目標が無い")
                else:
                    target, kp, kd = freeze_target, p.kp, p.kd

            # --- 送る ---
            if target is not None and phase != "DAMP":
                if not np.isfinite(target).all():
                    go("DAMP", "目標が NaN")
                    target = None
                else:
                    if phase != "RAMP":  # RAMP は今の角度と既定の姿勢の間なので切らない（可動域の外から内へ跳ばない）
                        target = np.clip(target, q_lo, q_hi)
                    if last_target is not None:
                        target = last_target + np.clip(target - last_target, -MAX_TARGET_STEP, MAX_TARGET_STEP)
                    last_target = target
            if phase == "DAMP" or target is None:
                io.write(state, q, damp_kp, damp_kd)
                last_target = None if phase in ("WAIT", "DAMP") else last_target
            else:
                io.write(state, target, kp, kd)

            # --- 記録 ---
            loop_dt = time.monotonic() - now
            row = dict(t=now, phase=PHASES.index(phase), q=q, dq=dq, tau=tau, quat=quat, gyro=gyro, keys=keys,
                       q_sent=target if target is not None else np.full(dc.NUM_DOF, np.nan), loop_dt=loop_dt, period=period,
                       kp_sent=kp if (target is not None and phase != "DAMP") else damp_kp,
                       odom=np.array(odom) if odom is not None else np.full(3, np.nan),
                       odom_age=(now - odom_t) if odom_t is not None else np.inf, odom_ok=odom_ok)
            if out is not None:
                row.update(obs=out.obs, a_pol=out.a_pol, D=s.D, cmd=out.cmd, homie_a=out.homie_a, walk=out.walk,
                           q_policy=out.q_target, pose=np.array([pose.x, pose.y, pose.yaw]))
            else:
                row.update(obs=np.full(p.obs_dim, np.nan), a_pol=np.full(dc.NUM_ACT, np.nan), D=np.full(dc.NUM_ACT, np.nan),
                           cmd=np.full(7, np.nan), homie_a=np.full(dc.NUM_HOMIE, np.nan), walk=np.nan,
                           q_policy=np.full(dc.NUM_DOF, np.nan), pose=np.full(3, np.nan))
            log.add(**row)

            if now - last_print > 0.5:
                last_print = now
                print(f"[door_real] {phase:6s} 傾き {math.degrees(tilt):4.1f}° |dq| {np.abs(dq).max():4.1f} "
                      f"odom {'%.2fs' % (now - odom_t) if odom_t else '無し'} "
                      + (f"姿勢 ({pose.x:+.2f},{pose.y:+.2f},{math.degrees(pose.yaw):+.0f}°) 指令 {out.cmd[:3].round(2).tolist()}" if out is not None else "")
                      + f" 周期 {period * 1000:.1f} ms（計算 {loop_dt * 1000:.1f} ms）"
                      + ("" if ready or phase != "WAIT" else " ボタンを全部離して待つ"), flush=True)
            if t_exit is not None and now >= t_exit:
                break

            # --- 50 Hz ---
            next_t += DT
            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()
    finally:
        # ここに来るのは WAIT・DAMP から終わるとき、Ctrl+C の 2 回目、輪の外側の例外だけ。脱力を 0.5 秒送り続ける
        for _ in range(25):
            try:
                st = io.snapshot()[0]
                if st is not None:
                    io.write(st, np.zeros(dc.NUM_DOF), damp_kp, damp_kd)
            except Exception as e:  # noqa: BLE001  脱力を送り切ることを優先する
                print(f"[door_real] 脱力の送信に失敗: {type(e).__name__}: {e}", flush=True)
            time.sleep(DT)
        try:
            log.save(log_dir / f"door_real_{stamp}.npz", dict(meta, reason=reason))
        except Exception as e:  # noqa: BLE001
            print(f"[door_real] 記録を書けなかった: {type(e).__name__}: {e}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
