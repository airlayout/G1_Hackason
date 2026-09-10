#!/usr/bin/env python3
"""テレオペで記録した腕の動きを G1 で再生する。

「教えた動きをそのまま再生できる。ただし物の位置が変わると失敗する
 → だから学習（模倣学習）が必要になる」という所まで体験できる。

    # 実機に送らずに軌道だけ確認する（既定。まずこれで中身を見る）
    python3 tools/replay_arm.py --episode <episode_0000>

    # 実機で再生する（必ず人が立ち会うこと）
    python3 tools/replay_arm.py --episode <episode_0000> \
        --network-interface enp0s31f6 --execute

安全のための設計:
  - 既定は dry-run。--execute を明示しない限り DDS には何も出さない
  - 実機モードは起動時に Y/N 確認を挟む
  - 現在姿勢 → 記録初期姿勢へ数秒かけて線形補間（いきなり飛ばない）
  - 中断(Enter / Ctrl+C)は現在姿勢をホールドしたまま制御権を返す（脱力させない）
  - 関節角は URDF のリミットでクリップ
  - 触るのは腕（と保持のための腰）のみ。脚・ハンドの姿勢は変えない

トピックの選択（ここが分かりにくい）:
  rt/arm_sdk … ロボットのモーションコントローラが動いているときに、腕の制御権だけ
                分けてもらう方式。コントローラが止まっていると **何も起きない**。
  rt/lowcmd  … 全身の低レベル指令。デバッグ状態（モーションコントローラ停止）では
                こちらでないと動かない。腕以外を空で送ると脱力するので、
                このスクリプトは送信前に全関節を現在角度でロックする。
  既定の --topic auto は実機に問い合わせて適切な方を選ぶ。
"""


import argparse
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

# ---- 関節インデックス（unitree_hg の motor_cmd 配列上の位置） ------------------

WEIGHT_JOINT = 29  # kNotUsedJoint。ここの q が arm_sdk の制御権 (0..1)

ARM_JOINTS_23 = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]           # 5 + 5
ARM_JOINTS_29 = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]  # 7 + 7
WAIST_JOINTS_23 = [12]
WAIST_JOINTS_29 = [12, 13, 14]

JOINT_NAMES = {
    12: "waist_yaw", 13: "waist_roll", 14: "waist_pitch",
    15: "L_shoulder_pitch", 16: "L_shoulder_roll", 17: "L_shoulder_yaw",
    18: "L_elbow", 19: "L_wrist_roll", 20: "L_wrist_pitch", 21: "L_wrist_yaw",
    22: "R_shoulder_pitch", 23: "R_shoulder_roll", 24: "R_shoulder_yaw",
    25: "R_elbow", 26: "R_wrist_roll", 27: "R_wrist_pitch", 28: "R_wrist_yaw",
}

# URDF から読んだ可動範囲 [rad]
LIMITS = {
    12: (-2.6180, 2.6180), 13: (-0.5200, 0.5200), 14: (-0.5200, 0.5200),
    15: (-3.0892, 2.6704), 16: (-1.5882, 2.2515), 17: (-2.6180, 2.6180),
    18: (-1.0472, 2.0944), 19: (-1.9722, 1.9722), 20: (-1.6144, 1.6144),
    21: (-1.6144, 1.6144),
    22: (-3.0892, 2.6704), 23: (-2.2515, 1.5882), 24: (-2.6180, 2.6180),
    25: (-1.0472, 2.0944), 26: (-1.9722, 1.9722), 27: (-1.6144, 1.6144),
    28: (-1.6144, 1.6144),
}

# g1_arm7_sdk_dds_example.py と同じ値。独自に上げない。
DEFAULT_KP = 60.0
DEFAULT_KD = 1.5
CONTROL_DT = 0.02  # 50 Hz

# rt/lowcmd を使うときの「腕以外を現在角度でロックする」ためのゲイン。
# xr_teleoperate の G1_29_ArmController.__init__ と同じ分類・同じ値にしてある。
# ここを外すと脚が脱力するので、値を変えないこと。
KP_HIGH, KD_HIGH = 300.0, 3.0     # 強いモータ（脚・腰など）
KP_LOW, KD_LOW = 80.0, 3.0        # 弱いモータ
KP_WRIST, KD_WRIST = 40.0, 1.5    # 手首
WEAK_MOTORS = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}      # 足首ピッチ + 肩/肘
WRIST_MOTORS = {19, 20, 21, 26, 27, 28}
N_BODY_JOINTS = 29                # 0..28 が実関節。29 は arm_sdk の weight 用

# 安全のための上限。ここを緩めるほど危険になる。
# 1 フレーム間の関節角変化[rad]。実機で問題なく再生できた記録は 0.28 rad/frame
# (30Hz で約 8 rad/s) 程度まで出るので、そこは通す。桁が違う値だけを弾く。
WARN_STEP_RAD = 0.35      # これを超えたら警告（人の操作でも稀に出る）
MAX_STEP_RAD = 1.0        # これを超えたら拒否（30Hz で 30 rad/s。記録の破損とみなす）
# lowstate がこの秒数途切れたら通信断とみなして停止する。
# 制御経路は有線（require_wired が 192.168.123.x を必須にする）で、実測 1000Hz 前後
# 届くので、0.5 秒は約 500 メッセージ落ちに相当し検出としては十分に緩い。
# それでも RViz や PlotJuggler と同時に動かすと GC や CPU 競合でコールバックが
# 遅れることがあるため、余裕を持たせている。--state-timeout で変更可。
STALE_STATE_SEC = 0.5
ARG_RANGES = {            # 引数名: (下限, 上限)
    "frequency": (1.0, 200.0),
    "approach": (0.5, 60.0),
    "weight_ramp": (0.2, 30.0),
    "kp": (1.0, 200.0),
    "kd": (0.0, 20.0),
    "state_timeout": (0.1, 5.0),
}


# ---- 記録の読み込み ------------------------------------------------------------

def load_episode(episode_dir, source="actions"):
    """episode ディレクトリの data.json を読み、(軌道 ndarray, fps, 関節数) を返す。

    ⚠️ 記録側の落とし穴:
    teleop_hand_and_arm.py は左右腕を `current_lr_arm_q[:7]` / `[-7:]` で切り出して
    保存している。29DoF 機（片腕 7 関節・計 14）なら正しく前半/後半に分かれるが、
    **23DoF 機は片腕 5 関節・計 10** なので、[:7] と [-7:] が中央 4 要素で重なった
    “ずれた 7 要素” が left_arm / right_arm として保存される。
    そのまま左右 7 関節として再生すると全く別の姿勢になるため、ここで
    重なり部分の一致を見て元の 10 要素配列を復元する。
    """
    episode_dir = Path(episode_dir)
    json_path = episode_dir / "data.json"
    if not json_path.exists():
        sys.exit(f"エラー: {json_path} がありません（episode_XXXX ディレクトリを指定）")

    with open(json_path, encoding="utf-8") as f:
        doc = json.load(f)

    frames = doc.get("data", [])
    if not frames:
        sys.exit(f"エラー: {json_path} にフレームがありません")

    fps = float(doc.get("info", {}).get("image", {}).get("fps", 30.0)) or 30.0

    left, right = [], []
    for i, fr in enumerate(frames):
        block = fr.get(source)
        if not block:
            sys.exit(f"エラー: frame {i} に '{source}' がありません")
        try:
            left.append(block["left_arm"]["qpos"])
            right.append(block["right_arm"]["qpos"])
        except (KeyError, TypeError):
            sys.exit(f"エラー: frame {i} の {source}.left_arm/right_arm.qpos を読めません")

    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    if left.shape[1] != 7 or right.shape[1] != 7:
        sys.exit(f"エラー: 想定外の要素数 left={left.shape[1]} right={right.shape[1]}（7 のはず）")

    return left, right, fps


def infer_dof(left, right, forced=None):
    """記録が 10 関節(23DoF機) か 14 関節(29DoF機) かを判定し、軌道を組み直す。

    23DoF 機では left=q[0:7], right=q[3:10] なので中央 4 要素が完全に一致する。
    29DoF 機では left=q[0:7], right=q[7:14] で重なりが無い。
    """
    overlap_err = float(np.abs(left[:, 3:7] - right[:, 0:4]).max())
    motion = float(np.abs(np.concatenate([left, right], axis=1).ptp(axis=0)).max())

    if forced == 23:
        dof = 23
    elif forced == 29:
        dof = 29
    elif overlap_err < 1e-6 and motion > 1e-3:
        dof = 23
    elif overlap_err > 1e-3:
        dof = 29
    else:
        sys.exit(
            "エラー: 記録が 23DoF 機のものか 29DoF 機のものか判定できません\n"
            f"  重なり誤差={overlap_err:.3e} / 動きの大きさ={motion:.3e}\n"
            "  （ほぼ静止した記録だと区別できません）\n"
            "  --dof 23 か --dof 29 を明示してください。")

    if dof == 23:
        # q = left[0:3] ++ right[0:7] で元の 10 要素を復元
        traj = np.concatenate([left[:, 0:3], right], axis=1)
        joints = ARM_JOINTS_23
    else:
        traj = np.concatenate([left, right], axis=1)
        joints = ARM_JOINTS_29

    assert traj.shape[1] == len(joints), (traj.shape, len(joints))
    return traj, joints, dof, overlap_err


def validate_trajectory(traj, fps):
    """軌道に NaN/inf や急激な飛びが無いかを確認する。

    位置リミットのクリップだけでは安全にならない。1 フレームで可動域いっぱいを
    移動する指令は、クリップを通ってしまうが実機では危険な急動作になる。
    記録の破損や IK の不連続を、送信前にここで弾く。
    """
    if not np.isfinite(traj).all():
        bad = int((~np.isfinite(traj)).sum())
        sys.exit(f"エラー: 軌道に NaN/inf が {bad} 個あります。記録が壊れています。")

    if len(traj) < 2:
        return
    steps = np.abs(np.diff(traj, axis=0))
    worst = float(steps.max())
    idx = int(np.unravel_index(steps.argmax(), steps.shape)[0])

    if worst > MAX_STEP_RAD:
        sys.exit(
            f"エラー: 記録に異常な飛びがあります（frame {idx}→{idx+1} で {worst:.3f} rad）\n"
            f"  {fps:.0f} Hz なら約 {worst * fps:.0f} rad/s に相当し、実機では出ない値です。\n"
            "  記録が壊れている可能性が高いので、実機では再生しません。\n"
            "  --source states / actions を切り替えるか、記録を録り直してください。")

    if worst > WARN_STEP_RAD:
        print(f"  ⚠️ 最大の飛びが {worst:.3f} rad/frame "
              f"（frame {idx}→{idx+1}、約 {worst * fps:.1f} rad/s）あります。\n"
              "     人の操作でも起こりますが、再生時に速い動きになります。"
              "実機では最初の動きを見ながら、危なければ Enter で中断してください。\n")


def validate_args(args):
    """数値引数が有限かつ安全な範囲かを確認する。"""
    for name, (lo, hi) in ARG_RANGES.items():
        v = getattr(args, name, None)
        if v is None:
            continue
        if not math.isfinite(v) or not (lo <= v <= hi):
            sys.exit(f"エラー: --{name.replace('_', '-')} の値 {v} は許容範囲外です"
                     f"（{lo} 〜 {hi}）。")


# ---- 表示 ----------------------------------------------------------------------

def summarize(traj, joints, dof, fps, overlap_err, source):
    print("=" * 62)
    print(" 記録の内容")
    print("=" * 62)
    print(f"  フレーム数 : {len(traj)}   ({len(traj) / fps:.1f} 秒 @ {fps:.0f} Hz)")
    print(f"  データ源   : {source}  (actions=IK解 / states=実測)")
    print(f"  機体判定   : {dof}DoF 機（腕 {len(joints)} 関節）"
          f"  重なり誤差={overlap_err:.2e}")
    print(f"  {'関節':<20}{'最小':>9}{'最大':>9}{'可動域':>9}   リミット")
    over = 0
    for k, j in enumerate(joints):
        lo, hi = traj[:, k].min(), traj[:, k].max()
        llo, lhi = LIMITS[j]
        mark = ""
        if lo < llo or hi > lhi:
            mark = "  ← 逸脱（クリップします）"
            over += 1
        print(f"  {JOINT_NAMES[j]:<20}{lo:>+9.3f}{hi:>+9.3f}{hi - lo:>9.3f}   "
              f"[{llo:+.2f}, {lhi:+.2f}]{mark}")
    if over:
        print(f"  ⚠️ {over} 関節がリミットを超えています。再生時はクリップされます。")
    print()


def plot_trajectory(traj, joints, fps, save=None):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    from matplotlib import font_manager
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Noto Sans CJK JP", "IPAGothic", "Droid Sans Fallback"):
        if cand in installed:
            matplotlib.rcParams["font.family"] = cand
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    t = np.arange(len(traj)) / fps
    n = len(joints)
    half = (n + 1) // 2
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharex=True, sharey=True)
    for ax, (title, idx) in zip(axes, (("左腕", range(half)), ("右腕", range(half, n)))):
        for k in idx:
            ax.plot(t, traj[:, k], lw=1.8, label=JOINT_NAMES[joints[k]])
        ax.set_title(title)
        ax.set_xlabel("時間 [s]")
        ax.grid(alpha=0.25, linestyle=":")
        ax.legend(fontsize=9)
    axes[0].set_ylabel("関節角 [rad]")
    fig.suptitle(f"記録された腕の軌道  {len(traj)} フレーム / {len(traj)/fps:.1f} 秒", fontsize=13)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=110, bbox_inches="tight")
        print(f"軌道プロットを保存しました: {save}")
    else:
        plt.show()


# ---- 実機再生 ------------------------------------------------------------------

class ArmReplayer:
    """arm_sdk 経由で腕だけを動かす。脚・ハンドには一切書き込まない。"""

    def __init__(self, iface, joints, waist_joints, kp, kd, topic,
                 gains_overridden=False, state_timeout=STALE_STATE_SEC):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self.joints = joints
        self.waist_joints = waist_joints
        self.kp, self.kd = kp, kd
        self.gains_overridden = gains_overridden
        self.state_timeout = state_timeout
        self.crc = CRC()
        self.low_state = None
        self.last_state_at = None    # time.monotonic()。通信断の検出に使う
        self.abort = threading.Event()
        self.abort_reason = ""
        self.started_sending = False

        # ChannelFactoryInitialize は main() で先に済ませてある
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.topic = topic
        self.pub = ChannelPublisher(topic, LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        self._header_ready = False

    def _gain_for(self, j):
        """関節 j に使う (kp, kd)。

        rt/lowcmd では teleop と同じ分類（手首 40 / 肩肘 80）にする。実機で
        動作実績のある値なので、ここを arm_sdk 例の一律 60 にはしない。
        --kp/--kd を明示されたときはそちらを優先する。
        """
        if self.gains_overridden or self.topic == "rt/arm_sdk":
            return self.kp, self.kd
        if j in WRIST_MOTORS:
            return KP_WRIST, KD_WRIST
        if j in WEAK_MOTORS:
            return KP_LOW, KD_LOW
        return KP_HIGH, KD_HIGH

    def _prepare_header(self):
        """指令メッセージの土台を作る。ここを省くとロボットは指令を無視する。

        xr_teleoperate の G1_29_ArmController.__init__ と同じ設定:
          - mode_pr / mode_machine（mode_machine は lowstate の値と一致させる）
          - 全モータの motor_cmd[].mode = 1
        この 3 つが無いと「送信は成功するのに腕が動かない」状態になる。

        rt/lowcmd は **全身の** 低レベル指令なので、腕以外を空のまま送ると
        q=0/kp=0/kd=0 が渡って脱力する（座位でも崩れる）。teleop と同じく
        起動時の実測角で全関節をロックしてから、腕だけを上書きする。
        """
        self.cmd.mode_pr = 0
        self.cmd.mode_machine = self.low_state.mode_machine
        for j in range(len(self.cmd.motor_cmd)):
            self.cmd.motor_cmd[j].mode = 1

        if self.topic == "rt/lowcmd":
            locked = 0
            for j in range(N_BODY_JOINTS):
                if j in WRIST_MOTORS:
                    kp, kd = KP_WRIST, KD_WRIST
                elif j in WEAK_MOTORS:
                    kp, kd = KP_LOW, KD_LOW
                else:
                    kp, kd = KP_HIGH, KD_HIGH
                m = self.cmd.motor_cmd[j]
                m.q = float(self.low_state.motor_state[j].q)   # 現在角度で保持
                m.dq = 0.0
                m.tau = 0.0
                m.kp = kp
                m.kd = kd
                locked += 1
            print(f"  全 {locked} 関節を現在角度でロックしました（腕以外は動きません）")
        self._header_ready = True

    def _on_state(self, msg):
        self.low_state = msg
        self.last_state_at = time.monotonic()

    def state_is_stale(self):
        """lowstate が途切れていないか。途切れたまま送信を続けると危険。"""
        if self.last_state_at is None:
            return True
        return (time.monotonic() - self.last_state_at) > self.state_timeout

    def check_alive(self):
        """通信が生きているかを確認し、切れていれば停止を要求する。"""
        if self.state_is_stale():
            self.abort.set()
            self.abort_reason = (
                f"ロボットの状態（rt/lowstate）が {self.state_timeout} 秒以上途切れました。"
                "ケーブル断・ロボット停止の可能性があります")
            return False
        return True

    def wait_for_state(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self.low_state is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.low_state is None:
            sys.exit("エラー: rt/lowstate を受信できません。"
                     "ネットワークインターフェース名とロボットの起動を確認してください。")

    def verify_robot_dof(self, expected_dof):
        """実機のモータ構成を読み、記録の DoF と一致するか確認する。

        記録側の構造だけで判定していると、23DoF 機に 29DoF の記録を流したときに
        右腕の値が別関節へずれて送られる。送信前にここで必ず弾く。
        """
        def live(idx):
            m = self.low_state.motor_state[idx]
            return m.vol > 0.1 or m.temperature[0] > 0

        n_left = sum(live(j) for j in (15, 16, 17, 18, 19, 20, 21))
        n_right = sum(live(j) for j in (22, 23, 24, 25, 26, 27, 28))
        if n_left == n_right == 7:
            actual = 29
        elif n_left == n_right == 5:
            actual = 23
        else:
            sys.exit(
                f"エラー: 実機のモータ構成を判定できません（腕 {n_left}+{n_right} 軸）。\n"
                "  ロボットの電源とモータの有効化を確認してください。\n"
                "  判定できない状態では実機に送信しません。")

        if actual != expected_dof:
            sys.exit(
                f"エラー: 記録は {expected_dof}DoF 機のものですが、"
                f"接続中の実機は {actual}DoF 機です（腕 {n_left}+{n_right} 軸）。\n"
                "  そのまま送ると右腕の値が別の関節へずれて送られ、危険です。\n"
                f"  この実機で記録したデータを使うか、{actual}DoF 機に繋いでください。")
        print(f"  実機の構成を確認: {actual}DoF（腕 {n_left}+{n_right} 軸）— 記録と一致")

    def current_q(self, joints):
        return np.array([self.low_state.motor_state[j].q for j in joints], dtype=np.float64)

    def _send(self, weight, targets, waist_hold):
        """1 フレーム分の指令を送る。targets は self.joints と同じ並び。"""
        if not self._header_ready:
            self._prepare_header()
        self.started_sending = True
        # 制御権 weight は arm_sdk 方式でのみ意味を持つ
        if self.topic == "rt/arm_sdk":
            self.cmd.motor_cmd[WEIGHT_JOINT].q = float(np.clip(weight, 0.0, 1.0))
        for k, j in enumerate(self.joints):
            lo, hi = LIMITS[j]
            self.cmd.motor_cmd[j].q = float(np.clip(targets[k], lo, hi))
            self.cmd.motor_cmd[j].dq = 0.0
            self.cmd.motor_cmd[j].tau = 0.0
            self.cmd.motor_cmd[j].kp, self.cmd.motor_cmd[j].kd = self._gain_for(j)
        # 腰は「動かさない」が、指令ゼロだと剛性ゼロで脱力する恐れがあるため
        # 開始時の実測値をホールドし続ける（--waist none で無効化できる）
        if waist_hold is not None:
            for k, j in enumerate(self.waist_joints):
                lo, hi = LIMITS[j]
                self.cmd.motor_cmd[j].q = float(np.clip(waist_hold[k], lo, hi))
                self.cmd.motor_cmd[j].dq = 0.0
                self.cmd.motor_cmd[j].tau = 0.0
                self.cmd.motor_cmd[j].kp, self.cmd.motor_cmd[j].kd = self._gain_for(j)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def _ramp(self, seconds, fn, label):
        """seconds 秒かけて fn(ratio) を 50Hz で送る。中断されたら False を返す。"""
        steps = max(1, int(seconds / CONTROL_DT))
        for i in range(steps + 1):
            if not self.check_alive() or self.abort.is_set():
                print(f"\n  中断（{label} の途中）"
                      + (f": {self.abort_reason}" if self.abort_reason else ""))
                return False
            fn(i / steps)
            time.sleep(CONTROL_DT)
        return True

    def run(self, traj, fps, weight_ramp, approach, waist_mode):
        self.wait_for_state()
        start_q = self.current_q(self.joints)
        waist_hold = self.current_q(self.waist_joints) if waist_mode == "hold" else None

        print("  現在の腕姿勢: " + " ".join(f"{v:+.3f}" for v in start_q))
        if waist_hold is not None:
            print("  腰をホールド: " + " ".join(f"{v:+.3f}" for v in waist_hold))

        self.last_target = start_q.copy()

        # [1] 制御権を 0→1 にゆっくり渡す（姿勢は現在のまま）
        print(f"  [1/6] 制御権を取得 ({weight_ramp:.1f} 秒)")
        if not self._ramp(weight_ramp,
                          lambda r: self._send(r, start_q, waist_hold), "制御権取得"):
            return self._release(waist_hold, weight_ramp)

        # [2] 現在姿勢 → 記録の初期姿勢へ線形補間
        print(f"  [2/6] 記録の初期姿勢へ移行 ({approach:.1f} 秒)")

        def to_first(r):
            self.last_target = (1 - r) * start_q + r * traj[0]
            self._send(1.0, self.last_target, waist_hold)

        if not self._ramp(approach, to_first, "初期姿勢への移行"):
            return self._release(waist_hold, weight_ramp)

        # [3] 記録軌道の再生（経過時間で補間サンプリング）
        duration = (len(traj) - 1) / fps
        print(f"  [3/6] 再生 ({duration:.1f} 秒 / {len(traj)} フレーム)")
        # 実時間ではなく monotonic を使う（NTP 調整やサスペンドで時計が飛ぶと
        # 軌道が一気に進んで急動作になるため）
        t0 = time.monotonic()
        while True:
            if not self.check_alive() or self.abort.is_set():
                print("\n  中断（再生中）"
                      + (f": {self.abort_reason}" if self.abort_reason else ""))
                return self._release(waist_hold, weight_ramp)
            t = time.monotonic() - t0
            if t >= duration:
                break
            pos = t * fps
            i = int(pos)
            frac = pos - i
            self.last_target = (1 - frac) * traj[i] + frac * traj[min(i + 1, len(traj) - 1)]
            self._send(1.0, self.last_target, waist_hold)
            time.sleep(CONTROL_DT)
            print(f"\r        {t:5.1f} / {duration:.1f} s", end="", flush=True)
        print()

        # [4] 記録の初期姿勢へ戻す
        print("  [4/6] 初期姿勢へ復帰 (3.0 秒)")
        end_q = self.last_target.copy()

        def back(r):
            self.last_target = (1 - r) * end_q + r * traj[0]
            self._send(1.0, self.last_target, waist_hold)

        self._ramp(3.0, back, "復帰")
        return self._release(waist_hold, weight_ramp)

    def _release(self, waist_hold, seconds):
        """指令を実測姿勢へ寄せてから送信を終える。

        単に最後の目標値を保持し続けると、追従遅れがあるぶん実測姿勢との差が
        残ったまま高ゲインで引っ張り続けることになる。そこで、まず実測角へ
        ゆっくり寄せて力を抜き、それから終了する。

        ⚠️ rt/arm_sdk では weight を 1→0 に落として制御権をロボット側へ返せる。
           rt/lowcmd には返却の仕組みが無く、**送信を止めるだけ**である。
           停止後の挙動はロボット側の watchdog に依存し、このスクリプトからは
           保証できない。ロボットは支持された状態で使うこと。
        """
        if not self.started_sending:
            print("  （まだ何も送信していないため、終了処理は不要です）")
            return True

        hold = (self.last_target.copy() if getattr(self, "last_target", None) is not None
                else None)

        # 実測角が取れるなら、そこへ寄せて力を抜く
        if not self.state_is_stale():
            measured = self.current_q(self.joints)
            if hold is None:
                hold = measured
            else:
                print("  [5/6] 指令を実測姿勢へ寄せる (1.0 秒)")
                start = hold.copy()
                steps = max(1, int(1.0 / CONTROL_DT))
                for i in range(steps + 1):
                    target = start + (measured - start) * (i / steps)
                    self._send(1.0, target, waist_hold)
                    time.sleep(CONTROL_DT)
                hold = measured
        elif hold is None:
            print("  ⚠️ 実測姿勢が取得できず、送信済みの目標値も無いため終了処理を省きます。")
            return False

        if self.topic == "rt/arm_sdk":
            print(f"  [6/6] 制御権を返却 ({seconds:.1f} 秒・姿勢はホールド)")
            steps = max(1, int(seconds / CONTROL_DT))
            for i in range(steps + 1):
                self._send(1.0 - i / steps, hold, waist_hold)
                time.sleep(CONTROL_DT)
            print("  完了（制御権をロボット側へ返しました）。")
        else:
            # rt/lowcmd に「返却」は無い。姿勢を保ったまま送信を止める。
            print(f"  [6/6] 姿勢を保持して送信終了 ({seconds:.1f} 秒)")
            steps = max(1, int(seconds / CONTROL_DT))
            for i in range(steps + 1):
                self._send(1.0, hold, waist_hold)
                time.sleep(CONTROL_DT)
            print("  送信を終了しました。")
            print("  ⚠️ rt/lowcmd には制御権の返却手順がありません。停止後の挙動は")
            print("     ロボット側の watchdog に依存します（このキットでは未検証）。")
        return True


def detect_control_topic(iface):
    """ロボットに問い合わせて、使うべき指令トピックを決める（読み取りのみ）。

    モーションコントローラが動いていれば rt/arm_sdk が使える。止まっていれば
    arm_sdk は効かない（制御権を渡す相手がいない）ので rt/lowcmd を使う。
    これを間違えると「再生成功と表示されるのに腕が動かない」状態になる。
    """
    name = current_motion_mode()
    if name:
        return "rt/arm_sdk", f"モーションコントローラ '{name}' が動作中"
    return "rt/lowcmd", "モーションコントローラが停止（デバッグ状態）= arm_sdk は効かない"


def require_motion_stopped(topic, force):
    """rt/lowcmd を使う前に、モーションコントローラが止まっていることを確認する。

    Unitree の手順では低レベル制御の前に高レベルのモーションサービスを止める。
    動いたまま lowcmd を送ると 2 つの制御が同じモータを取り合うことになる。
    """
    if topic != "rt/lowcmd":
        return
    name = current_motion_mode()
    if not name:
        return
    if force:
        print(f"  ⚠️ モーションコントローラ '{name}' が動作中ですが "
              "--force-lowcmd が指定されたため続行します（非推奨）。")
        return
    sys.exit(
        f"エラー: モーションコントローラ '{name}' が動作中です。\n"
        "  この状態で rt/lowcmd を送ると、ロボット側の制御と指令が競合します。\n"
        "  リモコンでモーション制御を停止してから実行してください。\n"
        "  （腕だけ動かしたいなら --topic rt/arm_sdk が本来の経路です）")


def current_motion_mode():
    """稼働中のモーションコントローラ名を返す。停止中や照会失敗時は空文字。"""
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient)
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        _, result = msc.CheckMode()
    except Exception:
        return ""
    return (result or {}).get("name", "") if isinstance(result, dict) else ""


def watch_motion_mode(replayer, interval=1.0):
    """再生中もモーションコントローラの状態を監視する（rt/lowcmd のときだけ）。

    起動前に一度確認するだけでは、送信中にモーションコントローラが立ち上がった
    ケースを検出できない。低レベル指令とロボット側の制御が同じモータを取り合うと
    危険なので、変化を見つけたら即停止する。

    ⚠️ この監視は実機で検証できていない（検証時にはモード変更を再現できなかった）。
    """
    while not replayer.abort.is_set():
        time.sleep(interval)
        if replayer.abort.is_set():
            return
        name = current_motion_mode()
        if name:
            replayer.abort.set()
            replayer.abort_reason = (
                f"再生中にモーションコントローラ '{name}' が起動しました。"
                "低レベル指令と競合するため停止しました")
            return


def watch_for_enter(replayer):
    try:
        sys.stdin.readline()
        replayer.abort.set()
    except Exception:
        pass


# ---- エントリポイント -----------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="記録した腕の動きを G1 で再生（P4）")
    p.add_argument("--episode", required=True, help="episode_XXXX ディレクトリ")
    p.add_argument("--source", choices=["actions", "states"], default="actions",
                   help="actions=IKの解（既定） / states=実測値")
    p.add_argument("--dof", type=int, choices=[23, 29], default=None,
                   help="機体の DoF を明示（既定は記録から自動判定）")
    p.add_argument("--frequency", type=float, default=None,
                   help="再生周波数[Hz]（既定は記録された値）")
    p.add_argument("--dry-run", action="store_true", default=True,
                   help="DDS 送信せず軌道の確認のみ（既定）")
    p.add_argument("--execute", dest="dry_run", action="store_false",
                   help="⚠️ 実機に送信する（ユーザー立ち会い必須）")
    p.add_argument("--network-interface", default=None, help="例: enp0s31f6")
    p.add_argument("--weight-ramp", type=float, default=2.0, help="制御権ランプ秒数")
    p.add_argument("--approach", type=float, default=4.0, help="初期姿勢への移行秒数")
    p.add_argument("--waist", choices=["hold", "none"], default="hold",
                   help="hold=腰を現在角で保持（既定） / "
                        "none=腰に書き込まない。rt/arm_sdk では腰の剛性が無くなる場合がある")
    p.add_argument("--topic", choices=["auto", "rt/arm_sdk", "rt/lowcmd"], default="auto",
                   help="指令トピック。auto（既定）は実機に問い合わせて選ぶ。"
                        "rt/arm_sdk はモーションコントローラ稼働時のみ有効。"
                        "rt/lowcmd は全身の低レベル指令（腕以外は現在角度でロックする）")
    p.add_argument("--state-timeout", type=float, default=STALE_STATE_SEC,
                   help=f"rt/lowstate がこの秒数途切れたら停止（既定 {STALE_STATE_SEC}）")
    p.add_argument("--force-lowcmd", action="store_true",
                   help="⚠️ モーションコントローラ稼働中でも rt/lowcmd を送る（非推奨）")
    p.add_argument("--kp", type=float, default=DEFAULT_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_KD)
    p.add_argument("--no-plot", action="store_true", help="dry-run でプロットを出さない")
    p.add_argument("--save-plot", default=None, help="dry-run のプロットを PNG 保存（画面不要）")
    args = p.parse_args()

    validate_args(args)
    left, right, rec_fps = load_episode(args.episode, args.source)
    traj, joints, dof, overlap_err = infer_dof(left, right, args.dof)
    fps = args.frequency or rec_fps

    summarize(traj, joints, dof, fps, overlap_err, args.source)
    validate_trajectory(traj, fps)

    if args.dry_run:
        print("=" * 62)
        print(" DRY-RUN（DDS には何も送信していません）")
        print("=" * 62)
        print("  先頭 3 フレームの送信予定値 [rad]:")
        print("    " + "  ".join(f"{JOINT_NAMES[j][:9]:>9}" for j in joints))
        for i in range(min(3, len(traj))):
            clipped = [np.clip(traj[i, k], *LIMITS[j]) for k, j in enumerate(joints)]
            print(f"  #{i} " + "  ".join(f"{v:>+9.3f}" for v in clipped))
        print(f"\n  制御権 weight は motor_cmd[{WEIGHT_JOINT}].q に 0→1→0 で設定されます")
        print(f"  kp={args.kp} kd={args.kd}  制御周期={CONTROL_DT * 1000:.0f} ms"
              f"  腰={args.waist}")
        print("\n  実機で再生するには --execute と --network-interface を付けてください。")
        if not args.no_plot:
            plot_trajectory(traj, joints, fps, args.save_plot)
        return

    # ---- ここから実機モード ----
    if not args.network_interface:
        sys.exit("エラー: --execute には --network-interface が必要です（例: enp0s31f6）")

    # DDS の初期化はここで 1 回だけ行う（二重に呼ぶと落ちる）
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    except ImportError as e:
        sys.exit(f"エラー: unitree_sdk2_python が見つかりません ({e})\n"
                 "  conda 環境 tv を有効にしてから実行してください。")
    ChannelFactoryInitialize(0, args.network_interface)

    topic = args.topic
    reason = "明示指定"
    if topic == "auto":
        topic, reason = detect_control_topic(args.network_interface)

    require_motion_stopped(topic, args.force_lowcmd)

    print("=" * 62)
    print(" ⚠️  実機送信モード")
    print("=" * 62)
    print(f"  インターフェース : {args.network_interface}")
    print(f"  トピック         : {topic}   （{reason}）")
    if topic == "rt/lowcmd":
        print("  ゲイン           : xr_teleoperate と同じ分類（手首40 / 肩肘80 / その他300）")
        print("  腕以外           : 起動時の実測角でロック（脱力させない）")
    else:
        print(f"  ゲイン           : kp={args.kp} kd={args.kd}")
    print(f"  対象関節         : 腕 {len(joints)} 関節のみ（脚・ハンドには書き込みません）")
    print(f"  通信断の検出     : rt/lowstate が {args.state_timeout} 秒途切れたら停止")
    if topic == "rt/lowcmd":
        print("  モード監視       : 再生中にモーションコントローラが起動したら停止")
    print(f"  腰               : {args.waist}")
    if args.waist == "none" and topic == "rt/arm_sdk":
        print("  ⚠️ --waist none と rt/arm_sdk の組み合わせでは、腰の指令が")
        print("     kp=0/kd=0 のまま送られ、腰の剛性が失われる場合があります。")
        print("     ロボットが支持されていない場合は --waist hold（既定）を使ってください。")
    print("  ロボットの腕の可動範囲に人・物が無いことを確認してください。")
    print("  中断は Enter または Ctrl+C（姿勢をホールドしたまま制御権を返します）。")
    if input("\n  実行しますか? [y/N] ").strip().lower() != "y":
        print("  中止しました。")
        return

    waist_joints = WAIST_JOINTS_23 if dof == 23 else WAIST_JOINTS_29
    gains_overridden = (args.kp != DEFAULT_KP) or (args.kd != DEFAULT_KD)
    replayer = ArmReplayer(args.network_interface, joints, waist_joints,
                           args.kp, args.kd, topic, gains_overridden,
                           args.state_timeout)

    # Ctrl+C 以外（SIGTERM・端末切断）でも必ず終了処理を通す。
    # ここを外すと、送信途中でプロセスが消えて腕が指令のまま取り残される。
    def on_signal(signum, _frame):
        replayer.abort.set()
        replayer.abort_reason = f"シグナル {signal.Signals(signum).name} を受信"
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, on_signal)
        except (ValueError, OSError):
            pass

    threading.Thread(target=watch_for_enter, args=(replayer,), daemon=True).start()
    if topic == "rt/lowcmd":
        threading.Thread(target=watch_motion_mode, args=(replayer,), daemon=True).start()

    try:
        replayer.wait_for_state()
        replayer.verify_robot_dof(dof)
        replayer.run(traj, fps, args.weight_ramp, args.approach, args.waist)
    except KeyboardInterrupt:
        replayer.abort.set()
        replayer.abort_reason = "Ctrl+C を受信"
    except BaseException as e:
        replayer.abort.set()
        replayer.abort_reason = f"{type(e).__name__}: {e}"
        print(f"\n  ⚠️ 予期しないエラー: {type(e).__name__}: {e}")
        raise
    finally:
        # 例外・シグナル・正常終了のいずれでも安全側の終了処理を試みる。
        # 終了処理中の 2 回目の割り込みで抜けないよう、ここでは例外を潰す。
        if replayer.started_sending:
            try:
                waist_hold = (replayer.current_q(waist_joints)
                              if args.waist == "hold" and not replayer.state_is_stale()
                              else None)
                replayer._release(waist_hold, args.weight_ramp)
            except BaseException as e:
                print(f"  ⚠️ 終了処理に失敗しました（{type(e).__name__}）。"
                      "ロボットの状態を目視で確認してください。")
        if replayer.abort_reason:
            print(f"  中断理由: {replayer.abort_reason}")


if __name__ == "__main__":
    main()
