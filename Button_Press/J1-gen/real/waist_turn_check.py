"""腰 yaw だけを小さい角度からゆっくり回して、実機で何が起きるかを確かめる（腕は動かさない）。ラボ PC から DDS で送る。

    # まず dry-run（既定）: lowstate を読んで、今の腰・骨盤の傾きと、これから送る動きを表示するだけ。何も送らない
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/waist_turn_check.py
    # ロボットにつながずに、計画の表示だけ
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/waist_turn_check.py --no-robot
    # ±5° を送る（既定の角度 5 -5。人がリモコンを持ち、危なければ L2+B）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/waist_turn_check.py --execute
    # 問題が無ければ ±10°、さらに ±20°（max_deg は絶対の上限 20°。--max-deg を明示しないと 5° を超える角度は断る）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/waist_turn_check.py --execute --angles 10 -10 --max-deg 10
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/waist_turn_check.py --execute --angles 20 -20 --max-deg 20

実機での順番（この順に、1 つずつ。前のが問題なければ次へ）:
1. dry-run（上の 1 つ目）。立っているか・骨盤の傾き・腰の角度・モーションコントローラの稼働を見る
2. --execute で ±5°（既定）
3. 問題なければ --angles 10 -10 --max-deg 10
4. 問題なければ --angles 20 -20 --max-deg 20
どの段階でも、止まったら（終了コード 6）理由を読んでから進める。記録（_local/button_press/waist_turn/<日時>/）を見て、
指令と実測のずれ・骨盤の傾き・ほかの関節のずれを確かめる。

⚠️ 実機で動かす前に:
- ハーネスを付け、ストップボタン（L2+B のリモコンの人）を用意する。柱・壁・人から 60 cm 以上離れる
- 腰・腕のモータを有効にする（リモコンでダンピング → 立たせる。ゼロトルクのままだと動かない）
- 腰を動かすのは、この道具が初めて（シミュレーションは骨盤を固定していて、反動・歩行制御との干渉が見えない）

送り方は real/move_arm_real.py と同じ（ArmCommander + DdsBackend の arm_sdk）: weight を 2 秒で 0→1（その間は開始時の
姿勢を保持）、終了時は 1→0。腕と腰 roll / pitch は開始時の角度を保持する指令を送り続け、腰 yaw だけ動かす。
Ctrl+C・例外・途中で止める判定のどれでも、ArmCommander の安全終了（weight を 2 秒で 1→0）に入る。そのまえに、
腕の位置が分かるときは腰 yaw を始めの角度へゆっくり戻す（もう一度 Ctrl+C で戻らずに weight を下げる）。

腰の PD（⚠️ どれが正しいかは未決定）: 既定は、J1-gen の実機用の道具（ArmCommander）が腰に送っている
configs/arm.yaml の arm_sdk.waist_hold（kp 60 / kd 1.5）。評価環境の実機用の口（Yada の contest.yaml の gains.waist、
kp 300 / kd 5。実機未確認）の値で試すなら --waist-kp 300 --waist-kd 5。使う値は起動時に表示する。

記録: 毎周期の CSV（body_log.csv。列は contest_agent の BodyLog と同じ t, phase, q0..q28, qw..qz, tau0..tau28, push_force_n に、
末尾の yaw_cmd_rad を足したもの。common/body_motion.py の read_lowstate_csv で読める）、まとめの JSON（summary.json）、
--save-images なら保つ間に頭カメラの RGB・深度を 1 枚ずつ。

終了コード: 0 完了 / 2 設定の誤り / 4 lowstate・モータ・相手の問題 / 6 途中で止める判定に当たった / 10 事前確認で止めた / 130 中止（Ctrl+C）
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from auto_press import Check, motion_controller_check  # noqa: E402
from auto_press import stand_check as _stand_check  # noqa: E402
from common.arm import ArmCommander, StateTimeoutError, StopRequested, WaistDeviationError, joint_limits  # noqa: E402
from common.arm.backend import ArmBackend  # noqa: E402
from common.arm.types import JointState  # noqa: E402
from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.robot_model import JOINT_NAMES, NUM_MOTORS, WAIST_IDX  # noqa: E402
from common.waist_turn import (  # noqa: E402
    ABS_MAX_DEG,
    DEFAULT_MAX_DEG,
    WAIST_YAW_IDX,
    GuardLimits,
    StopReason,
    WaistGuard,
    WaistTurnPlan,
    WaistTurnRecorder,
    smooth_return_path,
    summary_text,
)
from common.head_tilt import pelvis_tilt_deg  # noqa: E402

EXIT_PRECHECK = 10
EXIT_GUARD = 6
# 腰の PD の許す範囲。上は、今までに誰かが腰に送った最大の値（Yada の contest.yaml の kp 300 / kd 5）。
# それより強い値は、根拠が無いので受け付けない
MAX_WAIST_KP = 300.0
MAX_WAIST_KD = 5.0
SETTLE_TICKS = 25  # weight を 1 にしたあと、始めの角度を保ちながら基準を測る周期の数（50 Hz で 0.5 秒）


class WaistCommander(ArmCommander):
    """ArmCommander に、腰 yaw だけを動かす 1 周期分の送信を足したもの（weight・保持・安全終了はそのまま使う）。"""

    def set_waist_yaw(self, yaw: float) -> JointState | None:
        """腰 yaw の指令を yaw [rad] にして 1 周期送る。送る前に読んだ lowstate を返す。"""
        self._cmd.q[WAIST_YAW_IDX] = float(yaw)
        return self._send_tick()


class CsvLog:
    """毎周期の記録。列は contest_agent/agent.py の BodyLog と同じ（read_lowstate_csv が読む）。末尾に yaw_cmd_rad を足す。"""

    def __init__(self, folder: Path) -> None:
        folder.mkdir(parents=True, exist_ok=True)
        self.dir = folder
        self._f = open(folder / "body_log.csv", "w", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "phase"] + [f"q{i}" for i in range(NUM_MOTORS)] + ["qw", "qx", "qy", "qz"]
                         + [f"tau{i}" for i in range(NUM_MOTORS)] + ["push_force_n", "yaw_cmd_rad"])

    def write(self, t: float, phase: str, st: JointState, yaw_cmd: float) -> None:
        tau = st.tau_est if st.tau_est is not None else [""] * NUM_MOTORS
        self._w.writerow([f"{t:.4f}", phase] + [f"{v:.6f}" for v in st.q] + [f"{v:.6f}" for v in st.imu_quat]
                         + [(f"{v:.4f}" if v != "" else "") for v in tau] + ["", f"{yaw_cmd:.6f}"])

    def close(self) -> None:
        self._f.close()


class FrameGrabber:
    """頭カメラの最新の 1 枚を別スレッドで読み続け、頼まれたら次の 1 枚を保存する（読むのを待って制御周期を止めない）。"""

    def __init__(self, source: Any) -> None:
        self._src = source
        self._stop = False
        self._want: Path | None = None
        self._lock = threading.Lock()
        self.saved: list[str] = []
        self._th = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._th.start()

    def request(self, stem: Path) -> None:
        with self._lock:
            self._want = stem

    def _loop(self) -> None:
        from common.rgbd_io import save_rgbd

        while not self._stop:
            try:
                f = self._src.read_rgbd()
            except Exception as e:  # noqa: BLE001  カメラの失敗で制御を止めない
                print(f"[waist] ⚠️ カメラを読めない: {type(e).__name__}: {e}")
                time.sleep(0.5)
                continue
            with self._lock:
                stem, self._want = self._want, None
            if stem is not None and f is not None:
                try:
                    save_rgbd(f, stem)
                    self.saved.append(str(stem))
                except Exception as e:  # noqa: BLE001
                    print(f"[waist] ⚠️ 画像を保存できない: {type(e).__name__}: {e}")

    def stop(self) -> None:
        self._stop = True
        self._th.join(timeout=2.0)


# ---- 引数と事前確認 ---------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--angles", type=float, nargs="+", default=[5.0, -5.0], metavar="DEG",
                   help="始めの腰 yaw からの差 [deg] の並び（既定 5 -5。+ が左回り）")
    p.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG,
                   help=f"角度の上限 [deg]（既定 {DEFAULT_MAX_DEG:.0f}。超える角度は、これを明示して上げないと断る。絶対の上限 {ABS_MAX_DEG:.0f}）")
    p.add_argument("--speed-deg-s", type=float, default=10.0,
                   help="回す平均の速さ [deg/s]（既定 10。smoothstep の最大は 1.5 倍。最大が 20°/s を超える値は断る）")
    p.add_argument("--hold-s", type=float, default=3.0, help="各角度で止めて保つ秒数（既定 3）")
    p.add_argument("--waist-kp", type=float, help="腰の PD の kp（既定は configs/arm.yaml の arm_sdk.waist_hold。ほかの腰の値は 300 まで）")
    p.add_argument("--waist-kd", type=float, help="腰の PD の kd（既定は同じ。5 まで）")
    p.add_argument("--max-track-err-deg", type=float, default=GuardLimits.max_track_err_deg,
                   help="止める判定: 腰 yaw の指令と実測のずれ [deg]（仮の値）")
    p.add_argument("--max-tilt-change-deg", type=float, default=GuardLimits.max_tilt_change_deg,
                   help="止める判定: 骨盤の傾き（IMU roll/pitch）の始めからの変化 [deg]（仮の値）")
    p.add_argument("--max-other-dev-deg", type=float, default=GuardLimits.max_other_dev_deg,
                   help="止める判定: 腰 roll/pitch・腕の始めからのずれ [deg]（仮の値）")
    p.add_argument("--stale-s", type=float, default=GuardLimits.stale_s,
                   help="止める判定: lowstate がこの秒数更新されない（仮の値）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は設定ファイル）")
    p.add_argument("--execute", action="store_true", help="実際に送信する（付けなければ dry-run）")
    p.add_argument("--no-confirm", action="store_true", help="開始前の Enter 待ちを省く（--execute 時の既定は確認あり）")
    p.add_argument("--no-robot", action="store_true", help="ロボットにつながず、計画の表示だけして終わる（dry-run のみ）")
    p.add_argument("--out", help="記録の置き場（既定 _local/button_press/waist_turn/<日時>/）")
    p.add_argument("--save-images", action="store_true", help="各角度で保つ間に、頭カメラの RGB・深度を 1 枚ずつ保存する")
    p.add_argument("--skip-prechecks", action="store_true", help="事前確認を飛ばす（模擬ロボットで試すときだけ）")
    add_camera_args(p)
    return p


def stand_check(st: JointState) -> Check:
    """立っているか（real/auto_press.py の事前確認と同じ基準: 骨盤の傾き 5° 以下、腰 yaw/roll 3°・pitch 6° 以内）。"""
    return _stand_check(pelvis_tilt_deg(st.imu_quat), np.degrees(st.q[list(WAIST_IDX)]))


def resolve_gains(args: argparse.Namespace, arm_cfg: dict[str, Any]) -> tuple[float, float]:
    hold = arm_cfg["arm_sdk"]["waist_hold"]
    kp = float(hold["kp"] if args.waist_kp is None else args.waist_kp)
    kd = float(hold["kd"] if args.waist_kd is None else args.waist_kd)
    if not (0.0 < kp <= MAX_WAIST_KP):
        raise ValueError(f"--waist-kp は 0 より大きく {MAX_WAIST_KP:.0f} 以下: {kp}")
    if not (0.0 <= kd <= MAX_WAIST_KD):
        raise ValueError(f"--waist-kd は 0 以上 {MAX_WAIST_KD:.0f} 以下: {kd}")
    return kp, kd


def make_arm_cfg(args: argparse.Namespace, kp: float, kd: float) -> dict[str, Any]:
    """arm.yaml を、この道具用に上書きする（腰の PD、重力補償なし、腰ずれの上限は自前の判定より外）。"""
    cfg = load_config("arm.yaml")
    cfg["arm_sdk"]["include_waist_hold"] = True
    cfg["arm_sdk"]["waist_hold"] = {"kp": kp, "kd": kd}
    cfg["gravity_compensation"]["scale"] = 0.0
    cfg["gravity_compensation"]["waist_scale"] = 0.0
    # ArmCommander の「腰が開始時から waist_max_deviation_rad ずれたら中止」は、動かす角度より外に置く。
    # 先に当たるのは、この道具の判定（WaistGuard。戻る動作つき）にする
    cfg["safety"]["waist_max_deviation_rad"] = float(np.radians(args.max_deg + args.max_track_err_deg + 1.0))
    return cfg


def print_plan(args: argparse.Namespace, plan: WaistTurnPlan, kp: float, kd: float, limits: GuardLimits) -> None:
    print(f"[waist] 角度（始めからの差）: {', '.join(f'{a:+.1f}' for a in plan.angles_deg)}°（上限 {plan.max_deg:.1f}°、"
          f"絶対の上限 {ABS_MAX_DEG:.0f}°）")
    print(f"[waist] 平均 {plan.speed_deg_s:.1f}°/s（最大 {1.5 * plan.speed_deg_s:.1f}°/s）、保つ {plan.hold_s:.1f} 秒、"
          f"全体 {plan.duration:.1f} 秒（weight の上げ下げは別に 2 秒ずつ）")
    print(f"[waist] 腰の PD: kp {kp:g} / kd {kd:g}"
          + ("（configs/arm.yaml の既定）" if args.waist_kp is None and args.waist_kd is None else "（指定）"))
    print(f"[waist] 止める判定（仮の値）: 指令-実測 {limits.max_track_err_deg}°、骨盤の傾き {limits.max_tilt_change_deg}°、"
          f"ほかの関節 {limits.max_other_dev_deg}°、lowstate {limits.stale_s} 秒")
    for line in plan.describe():
        print(f"[waist]   {line}")


def limits_from(args: argparse.Namespace) -> GuardLimits:
    return GuardLimits(args.max_track_err_deg, args.max_tilt_change_deg, args.max_other_dev_deg, args.stale_s)


# ---- 実行 -------------------------------------------------------------------------


def _wait_state(backend: ArmBackend, timeout_s: float = 10.0) -> JointState:
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        st = backend.read_state()
        if st is not None:
            return st
        time.sleep(0.05)
    raise StateTimeoutError(f"{timeout_s} 秒待っても lowstate が届かない（有線接続・NIC 名・G1 の電源を確認）")


def _return_to_start(arm: WaistCommander, guard: WaistGuard, plan: WaistTurnPlan, rec: WaistTurnRecorder,
                     log: CsvLog | None, t0: float, dt: float) -> bool:
    """止めたあと、今の指令から始めの角度へゆっくり戻す。戻れたら True。

    戻る途中は判定を 2 倍にゆるめて続け（ArmCommander の戻る動作と同じ考え）、それも超えた・lowstate が途切れた・
    もう一度 Ctrl+C を受けたときは、戻るのをやめる（呼び出し元が、その場で weight を下げる）。
    """
    yaw_now = float(arm._cmd.q[WAIST_YAW_IDX])
    path = smooth_return_path(yaw_now, plan.start_yaw, dt)
    print(f"[waist] 始めの角度（{np.degrees(plan.start_yaw):+.1f}°）へゆっくり戻る（{len(path) * dt:.1f} 秒。"
          "もう一度 Ctrl+C で、戻らずにその場で weight を下げる）")
    t = t0
    try:
        for yaw in list(path) + [plan.start_yaw] * int(0.5 / dt):
            st = arm.backend.read_state()
            if st is None:
                raise StateTimeoutError("lowstate が無い")
            age = time.monotonic() - st.stamp
            why = guard.check(st.q, st.imu_quat, float(arm._cmd.q[WAIST_YAW_IDX]), age, scale=2.0)
            if why is not None:
                print(f"[waist] ⚠️ 戻る途中でも判定に当たった（{why.message}）。戻るのをやめて、その場で weight を下げる")
                return False
            t += dt
            if log is not None:
                log.write(t, "abort_return", st, yaw)
            arm.set_waist_yaw(float(yaw))
    except (StopRequested, StateTimeoutError, WaistDeviationError) as e:
        print(f"[waist] ⚠️ 戻るのをやめた（{type(e).__name__}: {e}）。その場で weight を下げる")
        return False
    print("[waist] 始めの角度へ戻った")
    return True


def execute_turn(args: argparse.Namespace, backend: ArmBackend, arm_cfg: dict[str, Any], out_dir: Path,
                 input_fn: Callable[[str], str], lower: np.ndarray, upper: np.ndarray,
                 grabber: FrameGrabber | None) -> tuple[int, dict[str, Any]]:
    """weight を上げ、計画どおりに腰 yaw を回し、安全に終える。(終了コード, 記録の辞書)。"""
    limits = limits_from(args)
    dt = 1.0 / float(arm_cfg["control_hz"])
    log = CsvLog(out_dir)
    rec = WaistTurnRecorder()
    result: dict[str, Any] = {"completed": False, "stop_reason": None, "returned_to_start": None}
    code = 0
    plan: WaistTurnPlan | None = None
    t_last = 0.0
    try:
        with WaistCommander(backend, arm_cfg, lower, upper, confirm=not args.no_confirm, input_fn=input_fn) as arm:
            q_start = arm._cmd.q.copy()  # 開始時に測った 29 関節（腕・腰 roll / pitch はこれを保持する指令）
            plan = WaistTurnPlan(float(q_start[WAIST_YAW_IDX]), args.angles, args.max_deg, args.speed_deg_s, args.hold_s)
            guard: WaistGuard | None = None
            try:
                # weight が 1 になったあと、始めの角度を保ちながら基準（判定の基準）を測る
                states = []
                for _ in range(SETTLE_TICKS):
                    st = arm.set_waist_yaw(plan.start_yaw)
                    if st is not None:
                        states.append(st)
                base = states[-10:]
                guard = WaistGuard(np.mean([s.q for s in base], axis=0), base[-1].imu_quat, limits)
                print(f"[waist] 基準: 腰 yaw {np.degrees(guard.q_base[WAIST_YAW_IDX]):+.2f}°、"
                      f"指令の始め {np.degrees(plan.start_yaw):+.2f}°。送り始める")
                n = int(np.ceil(plan.duration / dt))
                shot_for: set[int] = set()
                for k in range(n + 1):
                    t = k * dt
                    t_last = t
                    seg = plan.segment_at(t)
                    yaw_cmd = plan.at(t)
                    st = arm.backend.read_state()
                    if st is None:
                        raise StateTimeoutError("lowstate が無い")
                    why = guard.check(st.q, st.imu_quat, float(arm._cmd.q[WAIST_YAW_IDX]), time.monotonic() - st.stamp)
                    if why is not None:
                        raise _GuardTrip(why)
                    rec.add(t, seg.kind, seg.index, yaw_cmd, st.q, st.imu_quat)
                    log.write(t, seg.label, st, yaw_cmd)
                    if (grabber is not None and seg.kind == "hold" and seg.index not in shot_for
                            and t - seg.t0 >= seg.t1 - seg.t0 - 0.5 * min(1.0, seg.t1 - seg.t0)):
                        shot_for.add(seg.index)  # 保つ間の後ろのほう（整定したあと）に 1 枚
                        grabber.request(out_dir / f"image_hold{seg.index}_{plan.angles_deg[seg.index]:+.0f}deg")
                    arm.set_waist_yaw(yaw_cmd)
                result["completed"] = True
                print("[waist] 計画どおりに回し終えた")
            except _GuardTrip as e:
                why = e.reason
                result["stop_reason"] = {"kind": why.kind, "message": why.message, "t_s": round(t_last, 3)}
                print(f"[waist] ❌ 止める判定に当たった: {why.message}")
                code = EXIT_GUARD
                if guard is not None and not why.hold_in_place:
                    result["returned_to_start"] = _return_to_start(arm, guard, plan, rec, log, t_last, dt)
                else:
                    print("[waist] 腰・骨盤の位置が分からないので戻らず、その場で保持して weight を下げる")
                    result["returned_to_start"] = False
            except StopRequested as e:
                result["stop_reason"] = {"kind": "user", "message": str(e) or "中止", "t_s": round(t_last, 3)}
                code = 130
                if guard is not None:
                    result["returned_to_start"] = _return_to_start(arm, guard, plan, rec, log, t_last, dt)
                else:
                    result["returned_to_start"] = None  # 基準を測る前。腰は動かしていない
            except StateTimeoutError as e:
                result["stop_reason"] = {"kind": "stale", "message": str(e), "t_s": round(t_last, 3)}
                print(f"[waist] ❌ {e}。位置が分からないので戻らず、その場で保持して weight を下げる")
                result["returned_to_start"] = False
                code = 4
            except WaistDeviationError as e:  # ArmCommander 側の上限（自前の判定より外）。戻らず weight を下げる
                result["stop_reason"] = {"kind": "waist_deviation", "message": str(e), "t_s": round(t_last, 3)}
                print(f"[waist] ❌ {e}")
                result["returned_to_start"] = False
                code = 5
    except StopRequested as e:  # 開始前の確認で q、または開始中の Ctrl+C（腰は動かしていない）
        print(f"[waist] 中止: {e}（何も動かしていない）")
        result["stop_reason"] = {"kind": "user", "message": str(e) or "中止", "t_s": 0.0}
        code = 130
    except RuntimeError as e:  # モータが無効（ゼロトルク）、mode_machine が違う、相手の食い違い、など
        print(f"[waist] ❌ {e}")
        result["stop_reason"] = {"kind": "start", "message": str(e), "t_s": 0.0}
        code = 4
    finally:
        log.close()
    if plan is not None and rec.samples:
        summary = rec.summarize(plan.start_yaw, plan.angles_deg)
        result["summary"] = summary
        print("[waist] まとめ")
        print(summary_text(summary))
    return code, result


class _GuardTrip(Exception):
    def __init__(self, reason: StopReason) -> None:
        super().__init__(reason.message)
        self.reason = reason


def run(args: argparse.Namespace, backend: ArmBackend | None = None, input_fn: Callable[[str], str] = input,
        lower_upper: tuple[np.ndarray, np.ndarray] | None = None) -> int:
    """backend を渡さなければ、実機用の DdsBackend（arm_sdk）を作る（テストは偽のバックエンドを渡す）。"""
    try:
        arm_cfg0 = load_config("arm.yaml")
        kp, kd = resolve_gains(args, arm_cfg0)
        limits = limits_from(args)
        # 角度・速さ・上限の確認（通らなければ ValueError。何もつながない前に断る）
        plan0 = WaistTurnPlan(0.0, args.angles, args.max_deg, args.speed_deg_s, args.hold_s)
    except ValueError as e:
        print(f"[waist] 設定の誤り（何も送っていない）: {e}")
        return 2
    mode = "送る（--execute）" if args.execute else "dry-run（送らない）"
    print(f"[waist] {mode}")
    arm_cfg = make_arm_cfg(args, kp, kd)
    if args.no_robot:
        if args.execute:
            print("[waist] --no-robot と --execute は一緒に使えない")
            return 2
        print_plan(args, plan0, kp, kd, limits)
        print("[waist] --no-robot: ロボットにつながず、計画を表示しただけ（始めの角度は 0° と仮定）")
        return 0

    iface = args.network_interface or arm_cfg["network_interface"]
    own_backend = backend is None
    if backend is None:
        from common.arm.backend_dds import DdsBackend

        backend = DdsBackend("arm_sdk", network_interface=iface, domain_id=int(arm_cfg.get("domain_id", 0)),
                             dry_run=not args.execute)
    try:
        backend.open()
        st = _wait_state(backend)
        backend.verify_peer(st)
    except (StateTimeoutError, ValueError, RuntimeError) as e:
        print(f"[waist] ❌ {e}")
        return 4
    plan = WaistTurnPlan(float(st.q[WAIST_YAW_IDX]), args.angles, args.max_deg, args.speed_deg_s, args.hold_s)
    print(f"[waist] 今の腰 yaw {np.degrees(st.q[WAIST_YAW_IDX]):+.2f}°、骨盤の傾き {pelvis_tilt_deg(st.imu_quat):.1f}°、"
          f"mode_machine {st.mode_machine}、lowstate の受信 OK")
    print_plan(args, plan, kp, kd, limits)

    if args.skip_prechecks:
        print("[waist] ⚠️ 事前確認を飛ばす（--skip-prechecks）")
        checks: list[Check] = []
    else:
        checks = [stand_check(st)]
        if own_backend:  # モーションコントローラへの問い合わせ（見るだけ）。偽のバックエンドでは行わない
            checks.insert(0, motion_controller_check())
        off = [JOINT_NAMES[i] for i in range(12, 29) if st.motor_mode[i] != 1]
        checks.append(Check("モータの mode", not off, "腰・腕すべて 1" if not off else
                            f"1 でない: {', '.join(off)}（ゼロトルク。リモコンでダンピングに入れてから立たせる）"))
        print("[waist] 事前確認（見るだけ）")
        for c in checks:
            print(f"[waist]   {'OK' if c.ok else 'NG'} {c.name}: {c.message}")
    if not all(c.ok for c in checks):
        print(f"[waist] ❌ 事前確認で止めた（何も送っていない）")
        return EXIT_PRECHECK
    if not args.execute:
        print("[waist] dry-run なので、ここで終わる（送るには --execute）")
        backend.close()
        return 0

    out_dir = Path(args.out) if args.out else REPO_ROOT / "_local" / "button_press" / "waist_turn" / time.strftime("%Y%m%d_%H%M%S")
    print(f"[waist] ⚠️ 実機の腰を動かす。ハーネス・ストップボタン（L2+B）を用意し、柱・壁・人から 60 cm 以上離れていること。記録: {out_dir}")
    grabber = None
    source = None
    if args.save_images:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_camera_config(args, quiet=True)["rgbd"]
        source = RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"]))
        source.open()
        grabber = FrameGrabber(source)
        grabber.start()
    try:
        if lower_upper is None:
            lower_upper = joint_limits(load_config("robot.yaml"))
        code, result = execute_turn(args, backend, arm_cfg, out_dir, input_fn, lower_upper[0], lower_upper[1], grabber)
    finally:
        if grabber is not None:
            grabber.stop()
        if source is not None:
            source.close()
        backend.close()
    result.update(started_at=time.strftime("%Y-%m-%d %H:%M:%S"), argv=sys.argv, exit_code=code,
                  plan={"start_yaw_deg": float(np.degrees(plan.start_yaw)), "angles_deg": plan.angles_deg,
                        "max_deg": plan.max_deg, "speed_deg_s": plan.speed_deg_s, "hold_s": plan.hold_s,
                        "duration_s": plan.duration},
                  waist_pd={"kp": kp, "kd": kd}, limits=asdict(limits),
                  images=(grabber.saved if grabber is not None else []))
    (out_dir / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, default=float))
    print(f"[waist] 終了コード {code}。記録: {out_dir}")
    return code


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
