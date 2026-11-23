"""Local SO-101 filming desk. Default backend is simulated."""
import argparse
import json
import math
import mimetypes
import secrets
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from baseline import Baseline
from followlog import FollowLog
from home import capture, load_home, neutral_map, save_home
import reach_map
from ik_follow import CONTROLLED_JOINTS, INPUT_JOINTS, IkFollow, hand_motion, near_home
from palm import MIN_GAP, PALM_STATES, load_palm_up, save_palm_up
from limits import (JOINTS, MIN_SPAN, RANGES, clamp_pose, default_limits, load_limits, parse_limits, raw, save_limits,
                    speed_raw, step_raw)
from skeleton import FEATURES, POSE_NAMES, Calibration, PoseMover, calibration_poses, parse_features
from rest import rest_pose
from sim_log import SimLog
from sim_web import SimService
from teach import HUMAN_IDS, HUMAN_POSES, SAME_POSE, entry, load_teach, nearest_gap, save_teach

ROOT = Path(__file__).resolve().parent
LIMITS_FILE = ROOT / "limits.json"   # モーター別の制限の保存先（起動時に自動で読む）
HOME_FILE = ROOT / "home.json"       # 初期姿勢（追従の基準）の保存先（起動時に自動で読む）
PALM_FILE = ROOT / "palm.json"       # 「手のひら上」の手首の回転の保存先（起動時に自動で読む）
TEACH_FILE = ROOT / "teach.json"     # 教示（人の姿勢 H ごとのロボットの関節値）の保存先（起動時に自動で読む）
TIMEOUT = 0.65
LABELS = {"low": "つかむ位置", "high": "持ち上げた位置"}
# グリッパーは登録なしの固定範囲。指の開き 0（くっつけた）→ 最大閉、1（90度に開いた）→ 最大開。
# 向き（どちらが開くか）は実機で未確認。逆なら入れ替える（校正済みの 0〜100% が可動範囲いっぱい）。
GRIPPER_CLOSED, GRIPPER_OPEN = 0.0, 100.0
FREE_INPUTS = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "reach",
               "wrist_flex": "flex", "wrist_roll": "roll"}     # 関節 → サンプルの項目名（-1〜1）
# 位置の登録なしで追従する関節（gripper_only モード。名前は、最初がグリッパーだけだった名残）。入力が送られた関節だけ動かす
GRIPPER_ONLY_JOINTS = ("gripper", *FREE_INPUTS)
FOLLOW_NOTE = "追従中（全関節。位置の登録なし）"
WAIT_NOTE = "基準を取っています: 手を止めてください（腕は動かしません）"
EDGE_NOTE = "手を画面の真ん中の高さで止めてください（端で始めると、腕が机側へ動かせません）。腕は動かしません"
IK_REASONS = {"home": "初期姿勢が未登録", "far": "腕が初期姿勢から離れている", "follow": "肩・肘・手首の曲げの追従がすべてONではない",
              "baseline": "基準を取っていない", "input": "肩・肘の入力が来ていない", "units": "校正の単位が取れない"}
IK_NOTE = "IK 追従を使えません（{}）。関節ごとの追従になります"
# 校正が無い（模擬・テスト）ときの、各関節の可動範囲の代用（lerobot の校正の ticks）
FALLBACK_CALIBRATION = {j: {"range_min": 800, "range_max": 3300} for j in CONTROLLED_JOINTS}
WAIT_HAND_NOTE ="手が映るのを待っています（腕は動かしません。カメラに片手を映してください）"
SIM_LOG_BODY_MAX = 512 * 1024      # /sim/log の 1 回の送信の最大サイズ
DEBUG_VALUES_MAX = 12       # サンプルに付く、画面側の生の値（デバッグログ用）の最大の数
BODY_VALUES_MAX = 12        # サンプルに付く、人の骨格（肩・肘・手首の画像上の位置 x,y）の最大の数
LOG_IMAGE_MAX = 300         # 追従ログ1本に残す画像の最大の数
LOG_VIDEO_MAX = 150 * 1024 * 1024   # 追従ログ1本に残す映像（webm）の最大の大きさ（バイト）
VIDEO_CHUNK_MAX = 8 * 1024 * 1024   # 映像1回分（数秒）の送信の最大の大きさ（バイト）
WHY_MAX = 60                # 停止の原因（画面が付ける文字列）の最大の長さ
ARM_GAP = (1, 35)          # 上下２点の関節差の許容範囲（内部単位。表示は半分の%）
HISTORY_MAX = 50
PAGES = {"/": "desk.html", "/skeleton": "skeleton.html", "/teach": "teach.html", "/sim": "sim.html", "/sim/camera": "sim_camera.html"}      # 画面（URL → ファイル）
SCRIPTS = ("/app.js", "/limits.js", "/sync.js", "/style.css", "/skeleton.js", "/skeleton_math.js", "/skeleton_figure.js",
           "/teach.js", "/sim.js", "/sim_scene.js", "/sim.css", "/sim_camera.js", "/sim_hand.js")
FINGER_FILE = ROOT / "finger_calibration.json"   # 指の校正の保存先（1人1回。手が変わるときだけ測り直す）
FINGER_RATIO_MAX = 3.0
SNAPSHOT_MAX = 200          # 診断の保存の上限（件）
SNAPSHOT_BYTES = 3_000_000   # 診断1件の最大サイズ
# スライダー操作は、サーバーが持つ「指令位置」を一定の速さで目標へ進めて送る。
# 以前は「毎回 実際の位置 + 1〜3」を送っていたが、動くたびに差が消えて止まる→動くを繰り返し、
# 肩が揺れ（P=48 では机まで揺れた）、肘は摩擦に負けて全く動かなかった（実測 2026-11-22、
# 肘 P=16・差1 で 25 単位の指令に対し 0.09 しか動かない）。
# 指令位置の進みは 1回 MOVE_STEP、実際の位置との差は MOVE_LAG まで（lerobot の max_relative_target=3 以下）。
# 実測: この方式なら P=16（既定）のまま、肘は目標へ届き、肩も休め姿勢（-89）から持ち上がる。
MOVE_STEP = 1.0
MOVE_LAG = 3.0
LAG_WARN_RAW = 15          # 目標と実測の差がこれを超えたら、画面に「追いつけていない」と出す
DIAG_HOLD_S = 1.0          # 一度出した警告は、この時間は消さない（ちらつき防止）
MOVE_HOLD_S = 0.3          # この時間より長く指令が途切れたら、指令位置を実際の位置から取り直す
MOVE_DONE = 0.5            # 目標との差がこれ未満なら到達とみなし、今の位置で保持する
# モーターの状態: on=固く保つ、soft=弱い力で保つ（外から押せば動く）、off=力を抜く（手で動かせる）
DRIVE_NAMES = {"on": "ON", "soft": "やわらか", "off": "OFF"}
# やわらかモード＝「外力では動かせて、外力がなければ姿勢を保つ」。
# トルク上限を下げるだけでは作れない（実測 2026-11-22: 上限 350 は全く動かず、上限 10〜30 も押し返す。
# 上限 30 以下では、モーターが自力で動くこともできない）。そこで、目標を常に「今の位置」へ追従させる。
# 押されて動くと目標も一緒に動くのでモーターは押し返さず、外力が無ければ目標が動かず姿勢を保つ。
# 速度に比例して押し返す D 項は 0 にする。目標が動き出すまでの余裕（不感帯）を画面で調整する。
SOFT_TORQUE = 200               # やわらかモードのトルク上限（最大 1000）。暫定値・未調整
SOFT_DEADBAND = (0.5, 4.0)      # 目標を追従させ始める位置のずれ（内部単位）。やわらかさ 0% → 0.5、100% → 4.0
SOFT_LEVEL = 20                 # 既定の「保持の強さ」(%)。大きいほど押しても動きにくい。未調整の暫定値
SOFT_TICK_S = 0.03              # 追従の間隔


def pct(joint, value):
    """内部の値(関節 -100〜100、グリッパー 0〜100)を、可動範囲に対する 0〜100% にする。"""
    return value if joint == "gripper" else (value + 100) / 2


def number(value, low, high):
    if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"範囲外の入力 ({low} ～ {high})")
    return float(value)


def clip(value, low, high):
    return min(high, max(low, value))


def parse_follow(value):
    """追従する関節の指定（画面のチェック）。送られなければ全部。不正なら fail fast。"""
    if value is None:
        return frozenset(GRIPPER_ONLY_JOINTS)
    if not isinstance(value, list) or any(j not in GRIPPER_ONLY_JOINTS for j in value):
        raise ValueError("追従する関節の指定が不正です")
    return frozenset(value)


def debug_values(raw, limit=DEBUG_VALUES_MAX):
    """画面が付けた生の値（デバッグログ用）から、有限の数値だけを最大 limit 個取り出す。不正なら空。
    ログのための値なので、不正でも追従は止めない。"""
    if not isinstance(raw, dict):
        return {}
    ok = {k: float(v) for k, v in raw.items()
          if isinstance(k, str) and type(v) in (int, float) and math.isfinite(v)}
    return dict(list(ok.items())[:limit])


def soft_deadband(level):
    """保持の強さ(0〜100%)を、目標が今の位置に追従し始めるずれ（SOFT_DEADBAND の範囲）にする。"""
    low, high = SOFT_DEADBAND
    return low + (high - low) * clip(level, 0, 100) / 100


class MockArm:
    def __init__(self):
        self.pose = dict.fromkeys(JOINTS, 0.0)
        self.pose["gripper"] = 55.0
        self.drive = "on"
        self.limits = default_limits()

    def read(self):
        return self.pose.copy()

    def write(self, pose):
        self.pose.update(clamp_pose(pose, self.limits))

    def set_limits(self, limits):
        self.limits = limits

    def set_drive(self, mode, level=SOFT_LEVEL):
        self.drive = mode

    def soft_track(self):
        pass

    def close(self):
        pass


class RealArm:
    def __init__(self, args):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        config = SO101FollowerConfig(port=args.port, id=args.robot_id, calibration_dir=args.calibration_dir,
                                     use_degrees=False, max_relative_target=3.0)
        self.robot = SO101Follower(config)
        self.limits = default_limits()
        if not self.robot.calibration:
            raise ValueError(f"校正データがありません: {self.robot.calibration_fpath}")
        # IK 追従用: 校正で測った各関節の可動範囲（ticks）。関節値 1 あたりの角度の計算に使う
        self.calibration = {j: dict(range_min=self.robot.calibration[j].range_min, range_max=self.robot.calibration[j].range_max)
                            for j in CONTROLLED_JOINTS}
        try:
            self.robot.connect(calibrate=False)
            if not self.robot.is_calibrated:
                raise ValueError("実機と保存済み校正が一致しません。公式の校正手順を確認してください。")
            bus = self.robot.bus
            # 「ON」に戻すときのトルク上限と D ゲイン。やわらかモードから戻すために、いまの設定を覚えておく
            # Torque_Limit は電源を切るまで実機に残る。前回のやわらかモードの低い値を「ふだんの値」と
            # 誤って覚えないよう、起動時の値ではなく各モーター固有の上限(Max_Torque_Limit)を使う
            self.normal = {m: (bus.read("Max_Torque_Limit", m, normalize=False),
                               bus.read("D_Coefficient", m, normalize=False)) for m in bus.motors}
            for motor, (limit, _) in self.normal.items():
                bus.write("Torque_Limit", motor, limit, normalize=False)
            self.soft_goal, self.deadband = None, soft_deadband(SOFT_LEVEL)
            self.drive = "on"
            self.write(self.read())
        except BaseException:
            if self.robot.is_connected:
                self.close()
            raise

    def read(self):
        """手で動かして校正の範囲を少し超えても止まらないよう、範囲内に丸める（NaN・桁違いの値は異常として拒否）。"""
        obs = self.robot.get_observation()
        pose = {}
        for j in JOINTS:
            low = 0 if j == "gripper" else -100
            pose[j] = clip(number(obs[j + ".pos"], low - 50, 150), low, 100)
        return pose

    def write(self, pose):
        """範囲の外は丸め、1回の移動量は関節ごとの上限にする（LeRobot の安全装置は、書く関節の分だけ辞書で渡す）。"""
        pose = clamp_pose(pose, self.limits)
        self.robot.config.max_relative_target = {j: float(step_raw(j, self.limits)) for j in pose}
        self.robot.send_action({j + ".pos": value for j, value in pose.items()})

    def set_limits(self, limits):
        self.limits = limits

    def set_drive(self, mode, level=SOFT_LEVEL):
        """モーターの状態を切り替える。力を入れる前に、目標を今の位置にそろえて、急に動かないようにする。
        やわらか(soft)は、トルク上限を下げ D を 0 にして、soft_track() で目標を今の位置へ追従させる。
        level（保持の強さ 0〜100%）は不感帯の大きさ。soft のまま level を変えても使える。"""
        bus = self.robot.bus
        self.soft_goal = None
        if mode == "off":
            bus.disable_torque()
        else:
            pose = self.read()
            self.write(pose)
            soft = mode == "soft"
            for motor in bus.motors:
                limit, damping = self.normal[motor]
                bus.write("Torque_Limit", motor, min(limit, SOFT_TORQUE) if soft else limit)
                bus.write("D_Coefficient", motor, 0 if soft else damping)
            bus.enable_torque()
            if soft:
                self.soft_goal = pose
        self.drive, self.deadband = mode, soft_deadband(level)

    def soft_track(self):
        """やわらかモード中に一定間隔で呼ぶ。不感帯を超えて動いた関節だけ、目標を今の位置へ移す。"""
        if self.drive != "soft" or self.soft_goal is None:
            return
        pose = self.read()
        moved = {j: v for j, v in pose.items() if abs(v - self.soft_goal[j]) > self.deadband}
        if moved:
            self.soft_goal.update(moved)
            self.write(moved)

    def close(self):
        self.robot.disconnect()


class Desk:
    """arm が None のときは「実機が未校正」。retry() で接続をやり直し、成功すれば操作できる。
    retry を渡さない(模擬・テスト)ときは校正が不要で、最初から済みとして扱う。"""

    def __init__(self, arm, live=False, clock=time.monotonic, retry=None, why="", command="",
                 limits=None, limits_file=None, log=None, home=None, home_file=None, sync=False,
                 palm_up=None, palm_file=None, teach=None, teach_file=None, ik_follow=False):
        self.arm, self.live, self.clock = arm, live, clock
        self.ik_follow = ik_follow           # True なら、肩・肘・手首の曲げを IK で追従する（既定オフ。実機では未確認。ik_follow.py）
        self.ik = None                       # 追従 1 回ぶんの IK の状態（IkFollow）。使えない間は None
        self.teach, self.teach_file = dict(teach or {}), teach_file   # 教示: H の id → {"robot": 関節値, "time": 時刻}
        self.palm_up, self.palm_file = palm_up, palm_file   # 「手のひら上」の手首の回転（未登録なら None）
        self.palm_down = None                # 追従中の「手のひら下」の手首の回転（初期姿勢、無ければ追従開始時の値）
        self.log = log                       # 追従のデバッグログ（FollowLog）。None なら残さない
        self.limits, self.limits_file = limits or default_limits(), limits_file
        self.home, self.home_file = home, home_file      # 初期姿勢（5関節。未登録なら None）
        self.baseline = Baseline(sync)       # 追従の開始時に、手と腕の姿勢を基準にする（sync=True のとき。取るまで腕は動かさない）
        if arm is not None:
            arm.set_limits(self.limits)
        self.retry, self.why, self.command_hint = retry, why, command
        self.pose = arm.read() if arm else dict.fromkeys(JOINTS, 0.0)
        self.drive = arm.drive if arm else "on"
        self.soft_level = SOFT_LEVEL
        self.cmd = {}                        # 関節ごとの (指令位置, 時刻)。スライダー移動で、目標へ一定速度で進める
        self.marks = {}
        self.lagged = {}                     # 関節 → (種類 "fast"/"stuck", 目標との差, 時刻)。画面の「何が問題か」用
        self.followed = frozenset()          # 直前のサンプルで追従がONだった関節（途中でONにした関節の検出用）
        self.calibration_sk = Calibration()  # 骨格版の較正（3姿勢の撮影）
        self.mover = PoseMover()             # 骨格版の較正で、ロボットを3姿勢へ動かす
        self.poses_sk = None                 # 3姿勢（較正を始めたときの基準から作る）
        self.skeleton_on = False             # 骨格版で追従中
        self.mover_last = 0.0
        self.gripper_only = False           # True の間は、グリッパーだけ追従する（腕は今の姿勢のまま）
        self.history = []
        self.token = None
        self.note = "待機中"
        self.last = 0.0
        self.fault = False

    def calibration(self):
        """校正の状態。値の 0〜100% は、校正で測った各関節の可動範囲が基準になる。"""
        if self.arm is None:
            return dict(required=True, done=False, detail=self.why, command=self.command_hint)
        if self.retry is None:
            return dict(required=False, done=True, detail="模擬モードのため不要", command="")
        return dict(required=True, done=True, detail="保存済みの校正が実機と一致", command="")

    def note_lag(self, joint, want, goal, actual, now):
        """目標が実測から大きく離れた関節を記録する。fast=速度上限で指令が目標に届かない（手が速い）、
        stuck=指令は先へ出ているのに実測が付いてこない（モーターが動かない・遅い）。"""
        gap = abs(want - actual)
        if gap <= LAG_WARN_RAW:
            return
        lead = step_raw(joint, self.limits)
        kind = "stuck" if abs(goal - actual) >= lead - 0.3 else "fast"
        self.lagged[joint] = (kind, gap, now)

    def issues(self):
        """追従中の、いま画面に出す問題。速度上限・実測の遅れ・差の大きさを、関節ごとに返す。"""
        if not self.token:
            return []
        now = self.clock()
        found = [dict(joint=j, kind=k, gap=round(g), speed=self.limits[j]["speed"])
                 for j, (k, g, t) in self.lagged.items() if now - t < DIAG_HOLD_S]
        if self.baseline.edge:
            found.append(dict(joint="shoulder_lift", kind="edge", gap=0, speed=0))
        return found

    def state(self):
        return dict(live=self.live, pose=self.pose, marks={k: v.copy() for k, v in self.marks.items()},
                    history=list(self.history), problem=self.problem(), active=self.token is not None,
                    note=self.note, fault=self.fault, issues=self.issues(), calibration=self.calibration(), drive=self.drive,
                    soft_level=self.soft_level,
                    gripper=dict(closed=GRIPPER_CLOSED, open=GRIPPER_OPEN), gripper_only=self.gripper_only,
                    home=self.home, rest=rest_pose(self.teach, self.limits),
                    sync=self.baseline.state() if self.token and self.gripper_only else None,
                    skeleton=self.skeleton_state(), teach=dict(poses=HUMAN_POSES, saved=self.teach),
                    limits=self.limits, limit_defaults=default_limits(), limit_ranges=RANGES,
                    limit_min_span=MIN_SPAN)

    def tick(self):
        """一定間隔で呼ぶ。やわらかモード中は、目標を今の位置へ追従させる。"""
        if self.arm is not None and self.drive == "soft" and not self.token:
            self.arm.soft_track()
        if self.arm is not None and self.mover.active and not self.token:
            self.move_to_pose()

    def move_to_pose(self):
        """骨格版の較正の姿勢へ、指令を一歩進める。着いたら（または時間切れで）止まり、今の姿勢を保つ。"""
        now = self.clock()
        dt = clip(now - self.mover_last, 0, 0.12)
        self.mover_last = now
        current = self.arm.read()
        goal = self.mover.step(current, now, lambda j, actual, target: self.follow_goal(j, actual, target, dt, now))
        if goal:
            self.arm.write(goal)
        else:
            self.cmd.clear()
            self.pose = current
            self.note = (f"{POSE_NAMES[self.mover.at]}に着きました" if self.mover.at is not None
                         else "着かないまま時間切れ（姿勢を確認してください）")

    def refresh(self):
        """力を抜いた・やわらかい間は、手で動かした姿勢を画面に出すため、実機の位置を読み直す。"""
        if self.arm is not None and self.drive != "on" and not self.token:
            self.pose = self.arm.read()

    def recheck(self):
        """校正が済んだか、実機につなぎ直して確かめる。"""
        if self.arm is not None or self.retry is None:
            return
        try:
            arm = self.retry()
        except Exception as exc:
            self.why = str(exc)
            raise ValueError("まだ校正が済んでいません: " + self.why)
        self.arm, self.why = arm, ""
        arm.set_limits(self.limits)
        self.pose, self.drive = arm.read(), arm.drive
        self.note = "校正を確認。実機に接続"

    def problem(self):
        """登録済みの点どうしの食い違いを、数値つきの説明で返す。問題なければ None。"""
        m = self.marks
        if "low" in m and "high" in m:
            gap = max(abs(m["high"][j] - m["low"][j]) for j in JOINTS[:-1])
            low, high = ARM_GAP
            need = f"{low/2:g}〜{high/2:g}% が必要"
            if gap < low:
                return (f"つかむ位置と持ち上げた位置の関節差が {gap/2:.1f}% です（{need}）。"
                        "ほぼ同じ姿勢です。肩か肘を動かしてから「持ち上げた位置を登録」をやり直してください。")
            if gap > high:
                return (f"つかむ位置と持ち上げた位置の関節差が {gap/2:.1f}% です（{need}）。"
                        "離れすぎです。近い姿勢で登録し直してください。")
        return None

    def next_goal(self, joint, actual, target):
        """スライダー移動の次の指令位置。前回の指令位置を MOVE_STEP だけ目標へ進め、実際の位置から MOVE_LAG 以内に収める。
        目標に着いた（または止める）ときは、今の位置で保持する。指令が MOVE_HOLD_S 途切れたら取り直す。"""
        now = self.clock()
        previous = self.cmd.get(joint)
        if abs(target - actual) < MOVE_DONE:
            self.cmd.pop(joint, None)
            return actual
        base = previous[0] if previous and now - previous[1] < MOVE_HOLD_S else actual
        lag = min(MOVE_LAG, step_raw(joint, self.limits))     # 関節ごとの「1回の最大移動量」を超えない
        step = min(MOVE_STEP, lag)
        goal = base + clip(target - base, -step, step)
        goal = clip(goal, actual - lag, actual + lag)
        limits = (0, 100) if joint == "gripper" else (-100, 100)
        goal = clip(goal, *limits)
        self.cmd[joint] = (goal, now)
        return goal

    def follow_goal(self, joint, actual, want, dt, now):
        """追従の次の指令位置。前回の「指令位置」を速度制限ぶん目標へ進める（実測位置から毎回出すと、
        差が 1 サンプルぶん（約 1.4）しか開かず、重い関節や摩擦で動かなくなる）。
        ただし実際の位置からは 1 回の最大移動量(step)より先へ離さない（引っかかったときに指令だけ先へ行かない）。"""
        previous = self.cmd.get(joint)
        base = previous[0] if previous and now - previous[1] < MOVE_HOLD_S else actual
        speed = speed_raw(joint, self.limits)
        goal = base + clip(want - base, -speed*dt, speed*dt)
        lead = step_raw(joint, self.limits)
        goal = clip(goal, actual - lead, actual + lead)
        self.cmd[joint] = (goal, now)
        return goal

    def ik_why(self, free, follow, current):
        """IK 追従を使えない理由の鍵（IK_REASONS）。使えるなら None。"""
        if not self.ik_follow:
            return "off"
        if not self.baseline.enabled or not self.baseline.ready or not set(INPUT_JOINTS) <= set(self.baseline.hand):
            return "baseline"
        if not set(CONTROLLED_JOINTS) <= set(follow):
            return "follow"
        if not set(INPUT_JOINTS) <= set(free):
            return "input"
        if self.ik is None:
            if not self.home:
                return "home"
            if not near_home(current, self.home):
                return "far"
        return None

    def ik_wants(self, free, follow, current, dt):
        """IK 追従の、肩・肘・手首の曲げの目標（関節値）。使えないときは {}（関節ごとの追従になる）。"""
        why = self.ik_why(free, follow, current)
        if why is None and self.ik is None:
            calibration = getattr(self.arm, "calibration", None) or FALLBACK_CALIBRATION
            self.ik = IkFollow(reach_map.joint_units(calibration), self.home)
        if why is not None:
            if why == "follow":                 # 追従の関節が変わったら IK をやめる（一時的な入力の欠けでは続ける）
                self.ik = None
            if why != "off":
                self.note = IK_NOTE.format(IK_REASONS[why])
            return {}
        return self.ik.step(*hand_motion(self.baseline.hand, free), dt)

    def skeleton_state(self):
        c = self.calibration_sk
        return dict(names=list(POSE_NAMES), targets=self.poses_sk, captured=[i in c.captures for i in range(len(POSE_NAMES))],
                    moving=self.mover.active, at=self.mover.at, ready=c.ready, fits=c.fits, joints=FEATURES,
                    palm_up=self.palm_up,
                    following=bool(self.token and self.skeleton_on))

    def skeleton_pose_index(self, data):
        index = data.get("index")
        if type(index) is not int or not 0 <= index < len(POSE_NAMES):
            raise ValueError("不明な姿勢")
        return index

    def skeleton_command(self, op, data):
        """骨格版の操作（追従中でないときだけ）。較正の姿勢へ動かす・撮影・やり直し・追従の開始。"""
        now = self.clock()
        if op == "skeleton_reset":
            self.mover.stop()
            self.mover.at = None
            self.cmd.clear()
            self.calibration_sk.clear()
            self.poses_sk = None
            self.note = "骨格版の較正をやり直し"
        elif op == "skeleton_pose":
            index = self.skeleton_pose_index(data)
            if self.poses_sk is None:          # 最初に動かすときの初期姿勢（未登録なら今の姿勢）が、3姿勢の基準
                self.poses_sk = calibration_poses(self.home or capture(self.arm.read()), self.limits)
            self.cmd.clear()
            self.mover_last = now
            self.mover.go(index, self.poses_sk[index], now)
            self.note = f"{POSE_NAMES[index]}へ動かしています"
        elif op == "skeleton_capture":
            index = self.skeleton_pose_index(data)
            if self.mover.active:
                raise ValueError("ロボットが動いている間は撮影できません")
            if self.mover.at != index:
                raise ValueError(f"先にロボットを{POSE_NAMES[index]}へ動かしてください")
            human = parse_features(data.get("features"))
            if not human:
                raise ValueError("骨格が認識できていません")
            self.calibration_sk.add(index, self.arm.read(), human)
            self.note = f"{POSE_NAMES[index]}を撮影"
        elif op == "skeleton_start":
            if not self.calibration_sk.ready:
                raise ValueError("較正が済んでいません。3姿勢を撮影してください")
            down = self.home["wrist_roll"] if self.home else self.arm.read()["wrist_roll"]
            if self.palm_up is not None and abs(self.palm_up - down) < MIN_GAP:
                raise ValueError("「手のひら上」と「下」の手首の回転が近すぎます。モーターOFFで手首を回して、登録し直してください")
            self.palm_down = down
            self.gripper_only, self.skeleton_on = False, True
            self.mover.stop()
            self.cmd.clear()
            self.last = now
            self.token = secrets.token_hex(16)
            self.note = "追従中（骨格版）"
            self.log_begin("skeleton")
            return dict(**self.state(), token=self.token)
        else:
            raise ValueError("不明な操作")
        return self.state()

    def skeleton_sample(self, data):
        """骨格版の追従の1サンプル。人の角度を、較正の直線でロボットの値にして、速度制限つきで指令する。
        骨格が隠れた関節・追従をOFFにした関節は指令を出さない（今の姿勢のまま）。空の角度でも、入力途絶の停止を防ぐ。"""
        if not self.token or data.get("token") != self.token:
            raise ValueError("追従は停止中")
        if not self.skeleton_on:
            raise ValueError("骨格版の追従ではありません")
        features = parse_features(data.get("features"))
        follow = parse_follow(data.get("follow")) - {"gripper"}
        palm = data.get("palm", "down")
        if palm not in PALM_STATES:
            raise ValueError("手のひらは下か上で指定してください")
        if palm == "up" and self.palm_up is None:
            raise ValueError("「手のひら上」の位置が未登録です")
        now = self.clock()
        dt = clip(now - self.last, 0, 0.12)
        self.last = now
        current = self.arm.read()
        goal, want = {}, {}
        for joint, key in FEATURES.items():
            target = self.calibration_sk.target(joint, features[key]) if key in features and joint in follow else None
            if target is None:
                self.cmd.pop(joint, None)
                continue
            want[joint] = clip(target, raw(joint, self.limits[joint]["min"]), raw(joint, self.limits[joint]["max"]))
            goal[joint] = self.follow_goal(joint, current[joint], want[joint], dt, now)
        roll = "wrist_roll"      # 手首の回転は人の角度を使わず、指示した手のひらの上下の値へ（速度制限つき）
        want[roll] = clip(self.palm_up if palm == "up" else self.palm_down,
                          raw(roll, self.limits[roll]["min"]), raw(roll, self.limits[roll]["max"]))
        goal[roll] = self.follow_goal(roll, current[roll], want[roll], dt, now)
        self.arm.write(goal)
        self.pose = current
        if self.log is not None:
            self.log.write("sample", skeleton=True, palm=palm, follow=sorted(follow), inp=features, dt=dt, cur=current, want=want, goal=goal)

    def set_limits(self, data):
        """画面からの制限の変更（reset なら既定値へ）。検証→保存→適用の順で、保存に失敗したら何も変えない。"""
        new = default_limits() if data.get("reset") is True else parse_limits(data.get("limits", {}), self.limits)
        if self.limits_file:
            try:
                save_limits(new, self.limits_file)
            except OSError as exc:
                raise ValueError(f"制限を保存できません: {exc}")
        self.limits = new
        self.arm.set_limits(new)
        self.note = "制限を保存して適用"

    def set_home(self):
        """今の姿勢を初期姿勢として記録する（保存に失敗したら何も変えない）。"""
        home = capture(self.arm.read())
        if self.home_file:
            try:
                save_home(home, self.home_file)
            except OSError as exc:
                raise ValueError(f"初期姿勢を保存できません: {exc}")
        self.home = home
        self.note = "今の姿勢を初期姿勢に記録"

    def set_palm_up(self):
        """今の手首の回転を「手のひら上」として記録する（保存に失敗したら何も変えない）。初期姿勢と近すぎるときは拒否する。"""
        value = float(self.arm.read()["wrist_roll"])
        down = self.home["wrist_roll"] if self.home else None
        if down is not None and abs(value - down) < MIN_GAP:
            raise ValueError("初期姿勢（手のひら下）と手首の回転がほぼ同じです。手首を手のひら上の向きに回してから登録してください")
        if self.palm_file:
            try:
                save_palm_up(value, self.palm_file)
            except OSError as exc:
                raise ValueError(f"手のひら上を保存できません: {exc}")
        self.palm_up = value
        self.note = "今の手首を「手のひら上」に記録"

    def write_teach(self, new):
        """教示を保存してから反映する（保存に失敗したら何も変えない）。"""
        if self.teach_file:
            try:
                save_teach(new, self.teach_file)
            except OSError as exc:
                raise ValueError(f"教示を保存できません: {exc}")
        self.teach = new

    def teach_save(self, key):
        """今のロボットの姿勢を、人の姿勢 key の教示として保存する（同じ key は上書き）。"""
        if key not in HUMAN_IDS:
            raise ValueError("不明な人の姿勢")
        new_entry = entry(self.arm.read())
        self.write_teach({**self.teach, key: new_entry})
        gap = nearest_gap(self.teach, key, new_entry["robot"])
        self.note = f"{key} を保存"
        if gap is not None and gap < SAME_POSE:
            self.note += "（ほかの保存済みの姿勢とほぼ同じ値です。ロボットを動かしましたか？）"

    def teach_delete(self, key):
        if key not in self.teach:
            raise ValueError("その姿勢は保存されていません")
        self.write_teach({k: v for k, v in self.teach.items() if k != key})
        self.note = f"{key} の保存を消した"

    def log_begin(self, mode):
        """追従の開始をデバッグログに残す（開始時の姿勢・制限・登録点つき）。"""
        if self.log is not None:
            self.log.begin(mode=mode, live=self.live, pose=self.arm.read(), limits=self.limits, marks=self.marks,
                           home=self.home)

    def log_end(self, reason):
        """追従のデバッグログを、停止の理由つきで閉じる（ログなし・追従中でなければ何もしない）。"""
        if self.log is not None:
            self.log.end(reason)

    def log_image(self, data):
        """追従中の映像（認識点・人の骨格つき JPEG）を、今の追従ログの隣の画像フォルダへ保存し、ログに image の行を残す。
        追従中でなければ保存しない（ファイル名を返す。保存しなければ None）。"""
        import base64
        head = "data:image/jpeg;base64,"
        image = data.get("image")
        if not self.token or self.log is None or self.log.path is None:
            return None
        if not isinstance(image, str) or not image.startswith(head):
            raise ValueError("画像(JPEG)が必要です")
        folder = self.log.image_folder()
        folder.mkdir(exist_ok=True)
        number = len(list(folder.glob("*.jpg")))
        if number >= LOG_IMAGE_MAX:
            return None
        name = f"{number:04d}.jpg"
        (folder / name).write_bytes(base64.b64decode(image[len(head):], validate=True))
        self.log.write("image", file=f"{folder.name}/{name}", body=debug_values(data.get("body"), BODY_VALUES_MAX) or None)
        return name

    def log_video(self, chunk):
        """追従中のカメラ映像（webm の続き）を、今のログの隣の .webm へ追記する。最後の一切れは停止の後に届くので、
        追従中かどうかは見ず、直近のログに付ける。ログが無い・上限を超えたときは保存しない（保存後の大きさ、保存しなければ None）。"""
        if self.log is None or self.log.path is None:
            return None
        if not chunk:
            raise ValueError("映像が空です")
        path = self.log.video_path()
        first = not path.exists()
        size = 0 if first else path.stat().st_size
        if size + len(chunk) > LOG_VIDEO_MAX:
            return None
        with open(path, "ab") as f:
            f.write(chunk)
        if first:
            self.log.write("video", file=path.name)
        return size + len(chunk)

    def hold(self, reason):
        self.log_end(reason)
        self.token = None
        self.skeleton_on = False
        self.mover.stop()
        self.cmd.clear()
        self.lagged.clear()
        self.note = reason
        self.pose = self.arm.read()
        self.arm.write(self.pose)

    def expired(self):
        if self.token and self.clock() - self.last > TIMEOUT:
            self.hold("入力途絶で停止。開始ボタンで再開")

    def command(self, data):
        self.expired()
        op = data["op"]
        if self.fault:
            raise ValueError("通信異常。実機と配線を確認してサーバーを再起動")
        if op == "calibration_check":
            self.recheck()
        elif self.arm is None:
            if op != "stop":
                raise ValueError("先にモーターの校正が必要です")
            self.token = None
        elif op == "stop":
            why = data.get("why")           # 画面が付ける、停止の原因（デバッグログ用）
            self.hold("停止" + (f"（{why[:WHY_MAX]}）" if isinstance(why, str) and why else ""))
        elif op == "drive":
            mode = data["mode"]
            if mode not in DRIVE_NAMES:
                raise ValueError("不明なモーターの状態")
            level = number(data.get("level", self.soft_level), 0, 100)
            self.log_end("モーターの状態を切り替え")
            self.token = None                      # 追従中でも、先に追従を止める
            self.cmd.clear()
            self.arm.set_drive(mode, level)
            self.drive, self.soft_level = mode, level
            self.pose = self.arm.read()
            self.note = f"モーター: {DRIVE_NAMES[mode]}"
        elif op == "sample":
            if not self.token or data.get("token") != self.token:
                raise ValueError("追従は停止中")
            if self.skeleton_on:
                raise ValueError("骨格版の追従中です（skeleton_sample を使ってください）")
            height = number(data["height"], 0, 1)
            opening = number(data["opening"], 0, 1)
            # 位置の登録が要らない関節の入力（台座 pan、肩 lift、肘 reach、手首の曲げ flex、手首の回転 roll。すべて -1〜1）。
            # 送られた関節だけ動かす
            free = {j: number(data[key], -1, 1) for j, key in FREE_INPUTS.items() if key in data}
            now = self.clock()
            dt = clip(now - self.last, 0, 0.12)
            self.last = now
            current = self.arm.read()
            goal, want, rels = {}, {}, {}
            follow = parse_follow(data.get("follow"))
            was_ready = self.baseline.ready
            engaged = self.baseline.update(free, current, self.limits) if self.gripper_only else True
            if self.gripper_only:
                self.note = FOLLOW_NOTE if engaged else (EDGE_NOTE if self.baseline.edge else WAIT_NOTE)
                if was_ready:                   # 追従の途中でONにした関節は、その時点の手と腕を基準にし直す（飛び出し防止）
                    for j in follow - self.followed:
                        self.cmd.pop(j, None)
                        if j in free:
                            self.baseline.rebase(j, free[j], current)
                self.followed = follow
            ik_want = self.ik_wants(free, follow, current, dt) if self.gripper_only and engaged else {}
            for j in ((GRIPPER_ONLY_JOINTS if self.gripper_only else JOINTS) if engaged else ()):
                if self.gripper_only and j not in follow:      # 追従をOFFにした関節は、指令を出さない（今の姿勢のまま）
                    self.cmd.pop(j, None)
                    continue
                if j == "gripper":
                    a, b, fraction = GRIPPER_CLOSED, GRIPPER_OPEN, opening
                elif j in ik_want:                 # IK 追従（既定オフ）: 肩・肘・手首の曲げを先端の目標から同時に決める
                    a, b, fraction = 0.0, ik_want[j], 1.0
                elif self.gripper_only:
                    rel = self.baseline.hand_input(j, free[j]) if j in free else None
                    if rel is None:                # 入力が送られなければ（基準が無ければ）、その関節は今の姿勢のまま
                        continue
                    # 入力 -1〜1 を、その関節の「動かせる下限〜上限」へ写す。0 は腕の基準（始めたときの姿勢。基準を使わないなら初期姿勢）
                    rels[j] = rel                  # 反転・基準を反映した入力（ログ用）
                    lo, hi = raw(j, self.limits[j]["min"]), raw(j, self.limits[j]["max"])
                    a, b, fraction = 0.0, neutral_map(rel, lo, hi, self.baseline.reference(j, self.home)), 1.0
                else:
                    a, b, fraction = self.marks["low"][j], self.marks["high"][j], height
                want[j] = a + (b-a)*fraction
                goal[j] = self.follow_goal(j, current[j], want[j], dt, now)
                if j != "gripper":
                    self.note_lag(j, want[j], goal[j], current[j], now)
            if goal:
                self.arm.write(goal)
            self.pose = current
            if self.log is not None:
                self.log.write("sample", follow=sorted(follow) if self.gripper_only else None,
                               inp=dict(height=height, opening=opening, **{FREE_INPUTS[j]: v for j, v in free.items()}),
                               ik=list(self.ik.last) if ik_want else None,
                               dbg=debug_values(data.get("dbg")), body=debug_values(data.get("body"), BODY_VALUES_MAX) or None, dt=dt, rel=rels, cur=current, want=want, goal=goal,
                               sync=dict(engaged=engaged, gaps=self.baseline.gaps, hand=self.baseline.hand,
                                         anchor=self.baseline.anchor, flip=sorted(j for j, v in self.baseline.flip.items() if v)) if self.gripper_only else None)
        elif op == "wait":
            # 追従を始めたが、まだ手が映らない間の合図。姿勢は動かさず、入力途絶（TIMEOUT）で止まらないようにするだけ
            if not self.token or data.get("token") != self.token:
                raise ValueError("追従は停止中")
            if self.skeleton_on:
                raise ValueError("骨格版の追従中です")
            self.last = self.clock()
            self.note = WAIT_HAND_NOTE
        elif op == "skeleton_sample":
            self.skeleton_sample(data)
        elif self.token:
            raise ValueError("設定を変える前に追従を停止してください")
        elif op == "limits":
            self.set_limits(data)
        elif op == "home":
            self.set_home()
        elif op == "palm_set":
            self.set_palm_up()
        elif op == "teach_save":
            self.teach_save(data.get("id"))
        elif op == "teach_delete":
            self.teach_delete(data.get("id"))
        elif self.drive != "on" and op in ("move", "start", "skeleton_pose", "skeleton_start"):
            raise ValueError(f"モーターが「{DRIVE_NAMES[self.drive]}」のため動かせません。"
                             "動かすには「ON」に切り替えてください")
        elif op.startswith("skeleton_"):
            return self.skeleton_command(op, data)
        elif op == "move":
            j = data["joint"]
            if j not in JOINTS:
                raise ValueError("不明な関節")
            target = number(data["value"], 0 if j == "gripper" else -100, 100)
            actual = self.arm.read()[j]
            goal = self.next_goal(j, actual, target)
            self.arm.write({j: goal})
            self.mover.at = None
            self.pose = self.arm.read()
        elif op == "mark":
            label = data["label"]
            if label not in LABELS:
                raise ValueError("不明な登録点")
            self.pose = self.arm.read()
            self.marks[label] = self.pose.copy()
            self.history = (self.history + [dict(time=time.strftime("%H:%M:%S"), label=label,
                                                 pose=self.pose.copy())])[-HISTORY_MAX:]
            self.note = f"{LABELS[label]}を登録"
        elif op == "start" and data.get("gripper_only") is True:
            # 全関節の追従（名前は名残）。位置の登録は要らない。始めた瞬間の手と腕の姿勢を基準にする
            self.gripper_only, self.skeleton_on = True, False
            self.baseline.reset()
            self.ik = None
            self.followed = frozenset()
            self.cmd.clear()
            self.last = self.clock()
            self.token = secrets.token_hex(16)
            self.note = WAIT_NOTE if self.baseline.enabled else FOLLOW_NOTE
            self.log_begin("gripper_only")
            return dict(**self.state(), token=self.token)
        elif op == "start":
            self.gripper_only, self.skeleton_on = False, False
            missing = [name for key, name in LABELS.items() if key not in self.marks]
            if missing:
                raise ValueError("未登録の位置があります: " + "、".join(missing))
            if self.problem():
                raise ValueError(self.problem())
            current = self.arm.read()
            low, high = self.marks["low"], self.marks["high"]
            axes = JOINTS[:-1]
            travel = {j: high[j] - low[j] for j in axes}
            fraction = clip(sum((current[j]-low[j])*travel[j] for j in axes) /
                            sum(travel[j]**2 for j in axes), 0.0, 1.0)
            if any(abs(current[j] - (low[j]+fraction*travel[j])) > 4 for j in axes):
                raise ValueError("現在の姿勢が登録区間から外れています。手動操作で登録点付近に戻してください")
            self.last = self.clock()
            self.token = secrets.token_hex(16)
            self.note = "追従中"
            self.log_begin("full")
            return dict(**self.state(), token=self.token)
        else:
            raise ValueError("不明な操作")
        return self.state()


def save_snapshot(data, folder=None, now=time.time):
    """画面の「診断を保存」で送られた JPEG と数値を、debug フォルダに保存する（原因調査用。ファイル名を返す）。"""
    import base64
    folder = folder or ROOT / "debug"
    head = "data:image/jpeg;base64,"
    image, info = data.get("image"), data.get("info")
    if not isinstance(image, str) or not image.startswith(head) or not isinstance(info, dict):
        raise ValueError("画像(JPEG)と数値が必要です")
    jpeg = base64.b64decode(image[len(head):], validate=True)
    folder.mkdir(exist_ok=True)
    if len(list(folder.glob("snap-*.jpg"))) >= SNAPSHOT_MAX:
        raise ValueError(f"診断の保存が {SNAPSHOT_MAX} 件に達しました。debug フォルダを整理してください")
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now())) + f"-{int(now() * 1000) % 1000:03d}"
    (folder / f"snap-{stamp}.jpg").write_bytes(jpeg)
    (folder / f"snap-{stamp}.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
    return f"snap-{stamp}"


def load_finger(path=None):
    """指の校正（開いた・くっつけた距離比）。保存ファイルが無い・壊れていれば空。"""
    path = path or FINGER_FILE
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}
    return {k: float(data[k]) for k in ("open", "closed") if isinstance(data.get(k), (int, float))}


def save_finger(data, path=None):
    """画面から送られた校正値を保存する。送られた項目だけ上書きし、既存の他の項目は残す。"""
    path = path or FINGER_FILE
    merged = load_finger(path)
    for key in ("open", "closed"):
        if key in data:
            merged[key] = number(data[key], 0, FINGER_RATIO_MAX)
    path.write_text(json.dumps(merged, indent=1))
    return merged


def calibrate_command(args):
    """未校正のときに画面へ出す、公式の校正コマンド。"""
    command = (f"lerobot-calibrate --robot.type=so101_follower --robot.port={args.port} "
               f"--robot.id={args.robot_id}")
    if args.calibration_dir:
        command += f" --robot.calibration_dir={args.calibration_dir}"
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="実機のUSBシリアルポート。省略で模擬")
    parser.add_argument("--robot-id", help="公式の校正に使用したID")
    parser.add_argument("--calibration-dir", type=Path)
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--ik-follow", action="store_true",
                        help="肩・肘・手首の曲げを IK で追従する（既定オフ。シミュレーションで確認済み・実機では未確認。ik_follow.py）")
    args = parser.parse_args()
    if args.port and not args.robot_id:
        parser.error("--port には --robot-id も必要です")
    lock = threading.Lock()
    sim = SimService()
    sim_log = SimLog(ROOT / "debug")
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, body, kind="application/json"):
            if kind == "application/json":
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/finger":
                return self.reply(200, load_finger())
            if path == "/state":
                with lock:
                    try:
                        desk.refresh()
                    except Exception as exc:
                        fail(exc)
                    return self.reply(200, desk.state())
            served = sim.get(path)      # /sim 系（シミュレーション。実機の Desk とは別の世界）
            if served:
                return self.reply(served[0], served[1], served[2])
            if path in PAGES:
                file = ROOT / PAGES[path]
            elif path in SCRIPTS:
                file = ROOT / path[1:]
            elif path.startswith("/assets/") and path[8:] in ASSETS:
                file = ROOT / path[1:]
            else:
                return self.reply(404, {"error": "Not found"})
            if not file.is_file():
                return self.reply(404, {"error": "先に prepare.py を実行してください"})
            self.reply(200, file.read_bytes(), mimetypes.guess_type(file.name)[0] or "application/octet-stream")

        def do_POST(self):
            video = self.path == "/followvideo"
            if (self.path not in ("/command", "/snapshot", "/finger", "/followvideo", "/sim/command", "/sim/log") or self.headers.get("Host") != address
                    or self.headers.get("Origin") != "http://" + address
                    or self.headers.get_content_type() != ("video/webm" if video else "application/json")):
                return self.reply(403, {"error": "このPCの操作画面を使用してください"})
            if video:     # 追従中のカメラ映像の続き（生のバイト列）
                try:
                    size = int(self.headers.get("Content-Length", 0))
                    if not 0 < size <= VIDEO_CHUNK_MAX:
                        raise ValueError("映像のサイズが不正")
                    with lock:
                        return self.reply(200, {"saved": desk.log_video(self.rfile.read(size))})
                except ValueError as e:
                    return self.reply(400, {"error": str(e)})
                except OSError as e:
                    print(f"[video] 映像を保存できません（{e}）", file=sys.stderr, flush=True)
                    return self.reply(500, {"error": "映像を保存できません"})
            if self.path == "/sim/log":      # /sim/camera の操作ログ（1 フレームごとの記録のまとめ）。debug/sim-camera-*.jsonl へ
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= SIM_LOG_BODY_MAX:
                    return self.reply(400, {"error": "入力サイズが不正"})
                try:
                    return self.reply(200, {"saved": sim_log.append(self.rfile.read(size))})
                except ValueError as e:
                    return self.reply(400, {"error": str(e)})
                except OSError as e:
                    print(f"[sim-log] ログを保存できません（{e}）", file=sys.stderr, flush=True)
                    return self.reply(500, {"error": "ログを保存できません"})
            if self.path == "/sim/command":      # シミュレーションの操作。実機の lock も Desk も使わない
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 2048:
                    return self.reply(400, {"error": "入力サイズが不正"})
                served = sim.post(self.path, self.rfile.read(size))
                return self.reply(served[0], served[1], served[2])
            snapshot = self.path == "/snapshot"
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= (SNAPSHOT_BYTES if snapshot else 2048):
                    raise ValueError("入力サイズが不正")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("JSONオブジェクトが必要です")
                if snapshot and data.get("follow") is True:      # 追従ログへ残す画像（診断の保存とは別）
                    with lock:
                        return self.reply(200, {"saved": desk.log_image(data)})
                if snapshot:
                    return self.reply(200, {"saved": save_snapshot(data)})
                if self.path == "/finger":
                    return self.reply(200, save_finger(data))
            except ValueError as e:
                return self.reply(400, {"error": str(e)})
            with lock:
                try:
                    result = desk.command(data)
                except (KeyError, TypeError, ValueError) as e:
                    if desk.token and desk.log is not None:     # 追従中に拒否された指令（原因調査用）
                        desk.log.write("error", reason=f"{type(e).__name__}: {e}", op=str(data.get("op")))
                    return self.reply(400, {"error": str(e)})
                except Exception as e:
                    fail(e)
                    return self.reply(503, {"error": desk.note})
            self.reply(200, result)

    def fail(exc):
        if desk.log is not None:
            desk.log.write("error", reason=f"{type(exc).__name__}: {exc}")
        desk.log_end("通信異常で停止")
        desk.token = None
        desk.skeleton_on = False
        desk.mover.stop()
        desk.fault = True
        desk.note = "通信異常: " + str(exc)
        print(desk.note, flush=True)

    from prepare import ASSETS
    with ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler) as http:
        address = f"127.0.0.1:{http.server_port}"
        if args.port:
            try:
                arm, why = RealArm(args), ""
            except ValueError as exc:      # 校正の欠落・不一致は、画面で案内して待つ
                arm, why = None, str(exc)
            desk = Desk(arm, True, retry=lambda: RealArm(args), why=why, command=calibrate_command(args),
                        limits=load_limits(LIMITS_FILE), limits_file=LIMITS_FILE, log=FollowLog(ROOT / "debug"),
                        home=load_home(HOME_FILE), home_file=HOME_FILE, sync=True,
                        palm_up=load_palm_up(PALM_FILE), palm_file=PALM_FILE,
                        teach=load_teach(TEACH_FILE), teach_file=TEACH_FILE, ik_follow=args.ik_follow)
        else:
            desk = Desk(MockArm(), limits=load_limits(LIMITS_FILE), limits_file=LIMITS_FILE,
                        log=FollowLog(ROOT / "debug"), home=load_home(HOME_FILE), home_file=HOME_FILE, sync=True,
                        palm_up=load_palm_up(PALM_FILE), palm_file=PALM_FILE,
                        teach=load_teach(TEACH_FILE), teach_file=TEACH_FILE, ik_follow=args.ik_follow)
        done = threading.Event()
        def watch():
            while not done.wait(SOFT_TICK_S):
                with lock:
                    try:
                        desk.expired()
                        desk.tick()
                    except Exception as exc:
                        fail(exc)
        guard = threading.Thread(target=watch, daemon=True)
        guard.start()
        try:
            print(f"http://{address}  ({'実機' if args.port else '模擬'})", flush=True)
            http.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            done.set()
            guard.join()
            sim.close()
            with lock:
                desk.token = None
                if desk.arm is not None:
                    desk.arm.close()


if __name__ == "__main__":
    main()
