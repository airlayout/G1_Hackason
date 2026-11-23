"""人が操作する、シミュレーション上の SO-101 のピックアンドプレース（MuJoCo の物理。サーバーの /sim 画面の中身）。

公式 SO-101 モデル＋机＋お菓子（自由に動く箱）＋景品が落ちる机の穴。人は「先端の位置」（台座の向き・前後・高さ）と手首の回転・指の開きを
指定し、腕の関節は reach_map の逆運動学で決める（crane_sim.py / grasp_sim.py と同じ考え方）。実機とは無関係で、実機サーバーの状態は読まない・書かない。
mujoco と標準ライブラリ以外には、reach_map / smoothing だけに依存する（rerun は不要）。

【前提（未測定。推測を値に使っている）】お菓子の質量・摩擦、机の摩擦は仮置き（grasp_sim.py と同じ値）。サーボの強さはモデル付属の値。
お菓子の形は箱。机の高さは reach_map.Z_DESK（実機で定規では測っていない）。穴の位置・大きさは、このデモ用の仮置き。
"""
import json
import math
import struct
from pathlib import Path

import mujoco
import numpy as np

import reach_map
from smoothing import CriticallyDamped

HERE = Path(__file__).resolve().parent
MODEL_XML = HERE / "sim_assets" / "SO101" / "so101_new_calib.xml"
CALIBRATION = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/so101.json"
FALLBACK_RANGE = {"range_min": 800, "range_max": 3300}      # 校正が無いときの代用（server.FALLBACK_CALIBRATION と同じ）
TICKS_PER_TURN = 4096
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICK = 1 / 60                       # 1 回の更新の長さ（秒）。物理の刻みは model.opt.timestep（0.002 s）

# --- 机・お菓子・穴（grasp_sim.py と同じ値。未測定の仮置き）
DESK_HALF = (0.30, 0.45, 0.01)
DESK_CENTER_X = 0.28
SNACK_HALF = (0.015, 0.015, 0.025)  # 3 × 3 × 5 cm
SNACK_MASS = 0.03
SNACK_FRICTION = (2.0, 0.005, 0.0001)
# デモでは「確実に掴める」ことを優先し、実物の値からは外してある（実機の物性ではない）。
# 公式モデルのままだと、ジョーが 15〜19 mm もめり込んで潰し、お菓子が押し出されて掴めなかった。
JAW_FRICTION = 2.0                  # ジョーの滑り摩擦（接触では大きいほうが使われるので、お菓子側も同じ値）
GRIPPER_FORCE_SCALE = 0.3           # グリッパーの力（サーボの最大トルクと剛性）の倍率。弱めて潰さない
CONTACT_SOLREF = (0.006, 1.0)       # お菓子の接触の硬さ（既定 0.02 は柔らかく、めり込みが大きい）
JAW_BODIES = ("gripper", "moving_jaw_so101_v1")
DESK_FRICTION = (0.8, 0.005, 0.0001)
IMPRATIO = 5.0
SETTLE = 0.5                        # 初めに、お菓子が机に落ち着くまで待つ秒数
SNACK_X = 0.27                      # お菓子を置く、先端の前後の位置（m。grasp_sim で挟めた帯 0.24〜0.32 の真ん中あたり）
AIM_DX = -0.025                     # 先端の狙いは、お菓子の中心より手前へ 2.5 cm（grasp_sim.AIM_DX。固定爪が手前に来る）
PLACE_PAN = -30.0                   # 穴の向き（台座。マイナスで右。操作の単位。アームから見て右側に穴がある）
HOLE_HALF = 0.07                    # 机の穴の半分の大きさ（14 × 14 cm の正方形。お菓子は 3 × 3 cm。以前は 7 × 7 cm だったのを 2 倍にした）
BIN_DEPTH = 0.30                    # 机の面から、落ちたお菓子を受ける床までの深さ（m）

# --- 操作（人の指令）の範囲と速さ。値は設計値で、実機で確かめたものではない
# 腕が使ってよい関節値。実機の範囲（reach_map.USABLE）は肩を -35 までしか後ろへ倒せず、台座の近く（x≈0.2〜0.25）では先端が 3〜6 cm までしか上がらない
# （お菓子の上面は 5 cm。上げられないので、横から低く入って蹴飛ばす）。シミュレーションでは肩を -90 まで許し、全域で 16 cm まで上げられるようにする。
SIM_USABLE = {**reach_map.USABLE, "shoulder_lift": (-90.0, reach_map.USABLE["shoulder_lift"][1])}
START_X = 0.22                     # 始めの先端の前後（m）。机に近い低さ
PAN_MAX = 45.0                      # 台座の向きの範囲（±。操作の単位。関節値）
ROLL_MAX = 60.0                     # 手首の回転の範囲（±。関節値）
REACH_SPEED, HEIGHT_SPEED = 0.15, 0.12      # 先端の最大速度（m/s）
PAN_SPEED, ROLL_SPEED = 40.0, 70.0          # 台座・手首の回転の最大速度（関節値/s）
GRIP_SPEED = 1.5                    # 指の開きの最大速度（/s。0〜1）
MOVE_TIMEOUT = 0.4                  # 速度指令がこの時間（秒）途切れたら止める
COMMAND_TIME_CONSTANT = 0.05        # 関節指令の臨界減衰フィルタの時定数（秒）
CLAMP_HOLD = 0.5                    # 「届きません」を出し続ける秒数
CLAMP_TOLERANCE = 0.0008            # 先端が目標からこれ以上離れたら「届かない範囲」（m。1 ステップの移動 2.5 mm より小さくする）

# --- 課題の判定
LIFT_OK = 0.02                      # お菓子の底が机からこれ以上浮いたら「持ち上げた」（m）
FALL_MARGIN = 0.05                  # お菓子の中心が置いた面よりこれ以上下で、穴の外なら「机から落ちた」（m）
GOT_BELOW = 0.03                    # お菓子の中心が机の裏面よりこれ以上下まで、穴を通り抜けたら「景品ゲット」（m）
# --- アシスト（人の操作の難しさを下げる。実物にはない、デモ用の補助。既定はオフ。画面のチェックで入れる）
GRAB_RADIUS = 0.035                 # 先端からお菓子の中心までがこの距離（m）以内で、グリッパーを閉じると自動でつかむ（寄せはしない。その位置のまま持つ。狙いは中心の手前 2.5 cm）
GRAB_CLOSE = 0.35                   # 「閉じた」とみなす、グリッパーの開き（0 閉〜1 全開）の目標。これ以下でつかむ
GRAB_OPEN = 0.6                     # これ以上に開いたら、持っていたお菓子を離す
DROP_HALF = HOLE_HALF - 0.03        # 持ったお菓子の中心がこの範囲（穴の中心から）に入ったら（縁から 3 cm 内側。お菓子の半分 1.5 cm が縁に掛からない）、
DROP_HOLD_S = 0.25                  # この時間（秒）そのままで、自動で離す
# エイムアシスト（アームの位置を、狙いへ少し寄せる。アシストのチェックがオンのとき）。狙いに近いほど強く、遠い端では 0（途中で急に変わらない）
AIM_PICK_RADIUS = 0.09              # お菓子をつかむ狙い（お菓子の手前 2.5 cm・机すれすれ）が効き始める、先端との距離（m）
AIM_PLACE_RADIUS = 0.12             # 持っているお菓子を穴の真上へ寄せる狙いが効き始める、水平の距離（m。高さは触らない）
AIM_STRENGTH = 0.85                 # 水平のずれのうち、寄せる割合の最大（狙いの真上のとき。0〜1）
AIM_CLEAR_HEIGHT = 0.075            # 狙いから離れているとき、先端が保つ机からの高さ（m。お菓子の上面は 5 cm）
AIM_FUNNEL_IN, AIM_FUNNEL_OUT = 0.02, 0.05      # 狙いからの水平距離（m）。IN 以内は机すれすれまで下りられ、OUT 以上は AIM_CLEAR_HEIGHT を保つ（間はなめらか）
AIM_FUNNEL_FADE = 0.15              # これ以上離れたら、保つ高さを 0 へ戻していく（AIM_CLEAR_RADIUS で 0。急に変わらない）
AIM_CLEAR_RADIUS = 0.18             # 高さを保つ範囲は寄せる範囲（AIM_PICK_RADIUS）より広くとる（先端が上がる速さは横の動きより遅く、近づいてからでは間に合わない）
AIM_HEIGHT_RATIO = 0.4             # 高さは、水平の寄せ方のこの倍率だけ弱く寄せる（お菓子をつかむ狙いのときだけ）
RATE_KEYS = ("reach", "height", "pan", "roll", "grip")
SET_RANGES = {"x": (reach_map.X_MIN, reach_map.X_MAX), "height": (0.0, reach_map.HEIGHT_MAX),
              "pan": (-PAN_MAX, PAN_MAX), "roll": (-ROLL_MAX, ROLL_MAX), "opening": (0.0, 1.0)}


def load_calibration(path=CALIBRATION):
    """(較正, 実機の較正を使えたか)。無い・欠けているときは代用の範囲（仮）。"""
    try:
        data = json.loads(path.read_text())
        if all(j in data and {"range_min", "range_max"} <= set(data[j]) for j in ARM_JOINTS):
            return {j: data[j] for j in ARM_JOINTS}, True
    except (OSError, ValueError, TypeError):
        pass
    return {j: dict(FALLBACK_RANGE) for j in ARM_JOINTS}, False


def joint_radians(cal, joint, value):
    """関節値（-100〜100）をモデルの角度 rad へ（sim_replay.joint_radians と同じ）。"""
    c = cal[joint]
    return value / 100 * (c["range_max"] - c["range_min"]) / 2 * 2 * math.pi / TICKS_PER_TURN


def number(data, key, low, high):
    """data[key] を数として検証して low〜high に丸める。数でなければ ValueError。"""
    value = data[key]
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{key} は数で指定してください")
    return min(high, max(low, float(value)))


def smoothstep(x, low, high):
    """x が low 以下で 0、high 以上で 1、間はなめらかにつなぐ。"""
    f = min(1.0, max(0.0, (x - low) / (high - low)))
    return f * f * (3.0 - 2.0 * f)


def desk_pieces(hole=None):
    """机の天板を作る固定の箱 (中心, 半分の大きさ) の一覧。hole=(x, y) なら、そこに HOLE_HALF の正方形の穴をあける（箱 4 つで囲む）。"""
    x0, x1 = DESK_CENTER_X - DESK_HALF[0], DESK_CENTER_X + DESK_HALF[0]
    y0, y1 = -DESK_HALF[1], DESK_HALF[1]
    zc = reach_map.Z_DESK - DESK_HALF[2]
    if hole is None:
        rects = [(x0, x1, y0, y1)]
    else:
        hx0, hx1, hy0, hy1 = hole[0] - HOLE_HALF, hole[0] + HOLE_HALF, hole[1] - HOLE_HALF, hole[1] + HOLE_HALF
        # 継ぎ目をお菓子の初めの位置（y≈0）から遠ざける（継ぎ目の上でつかむと、箱の縁に引っかかって結果が変わる）
        rects = [(x0, x1, y0, hy0), (x0, x1, hy1, y1), (x0, hx0, hy0, hy1), (hx1, x1, hy0, hy1)]
    return [([(a + b) / 2, (c + d) / 2, zc], [(b - a) / 2, (d - c) / 2, DESK_HALF[2]]) for a, b, c, d in rects]


def build_model(hole=None):
    """公式モデルに、机（穴つきの固定の箱）・穴の下の床・お菓子（自由な箱）を足す。hole が None なら穴なし（腕の座標を測るためだけに使う）。"""
    spec = mujoco.MjSpec.from_file(str(MODEL_XML))
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = IMPRATIO
    world = spec.worldbody
    for i, (center, half) in enumerate(desk_pieces(hole)):
        piece = world.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=half, friction=list(DESK_FRICTION), pos=center,
                               rgba=[0.43, 0.43, 0.48, 0.5])
        piece.name = f"desk_{i}"
    bin_floor = world.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.0, 1.0, 0.1], friction=list(DESK_FRICTION),
                               pos=[0, 0, reach_map.Z_DESK - BIN_DEPTH], rgba=[0.2, 0.2, 0.2, 1.0])
    bin_floor.name = "bin_floor"
    body = world.add_body(name="snack", pos=[0, 0, 1])
    body.add_freejoint()
    geom = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(SNACK_HALF), mass=SNACK_MASS, condim=4,
                         rgba=[0.94, 0.59, 0.16, 1.0], friction=list(SNACK_FRICTION))
    geom.name = "snack_box"
    geom.solref = list(CONTACT_SOLREF)
    model = spec.compile()
    _tune_grasp(model)
    return model


def _tune_grasp(model):
    """掴みやすさの調整（ジョーの摩擦とグリッパーの力）。値の理由は定数の注釈。"""
    for g in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]))
        if body in JAW_BODIES and model.geom_group[g] == 3:
            model.geom_friction[g, 0] = JAW_FRICTION
    act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
    model.actuator_forcerange[act] *= GRIPPER_FORCE_SCALE
    model.actuator_gainprm[act, 0] *= GRIPPER_FORCE_SCALE
    model.actuator_biasprm[act, 1] *= GRIPPER_FORCE_SCALE


class PickPlaceSim:
    """1 つの操作セッションの世界。スレッドを使わないので、呼び出す側が排他制御をする（sim_web.SimService）。"""

    def __init__(self, cal=None, assist=False, running=True):
        self.assist = assist
        self.running = running      # False の間は、時間も腕も止まり、操作も受け付けない（スタート前・ストップ中）
        self.cal, self.real_cal = (cal, False) if cal else load_calibration()
        self.units = reach_map.joint_units(self.cal)
        self.pan_rad_per_unit = abs(joint_radians(self.cal, "shoulder_pan", 10.0) - joint_radians(self.cal, "shoulder_pan", 0.0)) / 10.0
        self.model = build_model()           # 穴の位置は腕の座標から決める。まず穴なしで測り、決まったら穴つきで作り直す
        self._bind()
        self.safe = reach_map.solve(START_X, reach_map.Z_MIN, self.units)[0]
        self.pan_sign = self._pan_sign()
        self.snack_start = self._place_for(0.0, SNACK_X)
        self.hole = self._place_for(PLACE_PAN, SNACK_X)[:2]
        self.model = build_model(self.hole)
        self._bind()
        self.reset()

    def _bind(self):
        """self.model に対する、シミュレーションの状態と名前の対応を作る。"""
        m = self.model
        self.data, self.probe = mujoco.MjData(m), mujoco.MjData(m)
        self.act = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in (*ARM_JOINTS, "gripper")}
        self.joint = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in (*ARM_JOINTS, "gripper")}
        self.snack_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "snack")
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.hand_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self.shoulder_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shoulder")
        self.snack_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "snack_box")
        self.snack_collide = (int(m.geom_contype[self.snack_geom]), int(m.geom_conaffinity[self.snack_geom]))
        self.geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] in (0, 1, 2)]

    # --- 座標（MuJoCo の順運動学で測る。平面の IK は y・台座の回転を扱わないため）
    def _pan_sign(self):
        """操作の pan（＋で左＝世界の＋y）に当たる、関節値の符号。"""
        before = self._tip_world(0.0, START_X, reach_map.Z_MIN)
        after = self._tip_world(10.0, START_X, reach_map.Z_MIN)
        return 1.0 if after[1] > before[1] else -1.0

    def _tip_world(self, pan_raw, x, z):
        """台座の関節値 pan_raw・先端の平面の位置 (x, z) に構えたときの、先端の世界の位置。"""
        pose = {**reach_map.solve(x, z, self.units, SIM_USABLE)[0], "shoulder_pan": pan_raw, "wrist_roll": 0.0}
        for joint, value in pose.items():
            self.probe.qpos[self.model.jnt_qposadr[self.joint[joint]]] = joint_radians(self.cal, joint, value)
        mujoco.mj_forward(self.model, self.probe)
        return self.probe.site_xpos[self.site].copy()

    def _place_for(self, pan, x):
        """操作の向き pan で先端を前後 x に構えたとき、その狙いの先にあるお菓子の中心 (x, y, z)。"""
        tip = self._tip_world(self.pan_sign * pan, x + AIM_DX, reach_map.Z_MIN)
        axis = self.probe.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "shoulder")][:2]
        forward = (tip[:2] - axis) / np.linalg.norm(tip[:2] - axis)
        xy = tip[:2] - AIM_DX * forward
        return (float(xy[0]), float(xy[1]), reach_map.Z_DESK + SNACK_HALF[2] + 0.0005)

    # --- 状態
    def reset(self):
        """腕を初期姿勢へ、お菓子を元の位置へ。課題もやり直す。"""
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        self.t = 0.0
        self.goal = dict(x=START_X, height=reach_map.Z_MIN - reach_map.Z_DESK, pan=0.0, roll=0.0, opening=1.0)
        self.want = dict(self.goal)
        self.rates, self.rates_until = dict.fromkeys(RATE_KEYS, 0.0), 0.0
        self.pose = self._arm_pose(self.goal)[0]
        self.filters = {j: CriticallyDamped(COMMAND_TIME_CONSTANT) for j in (*reach_map.USABLE, "shoulder_pan", "wrist_roll")}
        for joint, value in self.pose.items():
            d.qpos[m.jnt_qposadr[self.joint[joint]]] = joint_radians(self.cal, joint, value)
        self._apply_controls(self.pose, self.goal["opening"])
        gid = self.joint["gripper"]
        d.qpos[m.jnt_qposadr[gid]] = m.jnt_range[gid][0] + self.goal["opening"] * (m.jnt_range[gid][1] - m.jnt_range[gid][0])
        self._release()           # つかんだままやり直しても、お菓子の衝突を戻す（切ったままだと机をすり抜けて落ち続ける）
        adr = m.jnt_qposadr[m.body_jntadr[self.snack_id]]
        d.qpos[adr:adr + 7] = [*self.snack_start, 1, 0, 0, 0]
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        for _ in range(round(SETTLE / m.opt.timestep)):
            mujoco.mj_step(m, d)
        self.task = dict(lifted=False, got=False, fallen=False, started=None, finished=None)
        self.clamp_until = -1.0
        self.held, self.grabbable, self.held_rel, self.over_hole_since, self.dropped = False, False, None, None, False

    @property
    def clamped(self):
        """届かない所へ動かそうとして、先ごろ引き戻したか（画面の「届きません」。1 ステップで消えないよう少し保つ）。"""
        return self.t < self.clamp_until

    def _arm_pose(self, goal):
        """先端の目標 goal から、関節値と、実際に届く先端 (x, z)。使ってよい範囲の外は内側へ引く（reach_map）。"""
        x, z = reach_map.pull_in(goal["x"], reach_map.Z_DESK + goal["height"], self.units, usable=SIM_USABLE)
        joints, _ = reach_map.solve(x, z, self.units, SIM_USABLE)
        joints = reach_map.clamp_to_envelope(joints, self.safe, self.units, usable=SIM_USABLE)
        (x, z), _ = reach_map.tip_of(joints, self.units)
        pose = {**joints, "shoulder_pan": self.pan_sign * goal["pan"], "wrist_roll": goal["roll"]}
        return pose, (x, z)

    def _apply_controls(self, pose, opening):
        for joint in ARM_JOINTS:
            self.data.ctrl[self.act[joint]] = joint_radians(self.cal, joint, pose[joint])
        low, high = self.model.jnt_range[self.joint["gripper"]]
        self.data.ctrl[self.act["gripper"]] = low + opening * (high - low)

    # --- 指令
    def command(self, data):
        """画面からの指令。op: move（速度 -1〜1 を RATE_KEYS ごと。途切れると止まる）/ set（絶対値）/ assist /
        start（始める・ストップから再開。課題が終わっていれば最初から）/ stop（止める）/ restart（最初からやり直して始める）/ reset（最初から。始める・止める状態は変えない）。
        止まっている間の move・set は、検証だけして反映しない。"""
        op = data.get("op")
        if op == "reset":
            self.reset()
        elif op == "restart":
            self.reset()
            self.running = True
            self._start_timer()
        elif op == "start":
            if self.task["got"] or self.task["fallen"]:
                self.reset()
            self.running = True
            self._start_timer()
        elif op == "stop":
            self.running = False
            self.stop_motion()
        elif op == "move":
            rates = {k: number(data, k, -1.0, 1.0) if k in data else 0.0 for k in RATE_KEYS}
            if self.running:
                self.rates = rates
                self.rates_until = self.t + MOVE_TIMEOUT
                self._start_timer()
        elif op == "assist":
            if type(data.get("on")) is not bool:
                raise ValueError("on は true / false で指定してください")
            self.assist = data["on"]
            if not self.assist:
                self._release()
        elif op == "set":
            new = {key: number(data, key, low, high) for key, (low, high) in SET_RANGES.items() if key in data}
            if self.running:
                self.want.update(new)
                self._start_timer()
        else:
            raise ValueError(f"不明な操作: {op}")

    def stop_motion(self):
        self.rates, self.rates_until = dict.fromkeys(RATE_KEYS, 0.0), 0.0
        self.want = dict(self.goal)

    def _start_timer(self):
        if self.task["started"] is None:
            self.task["started"] = self.t

    def _integrate(self, dt):
        """速度指令を目標に足し、実際の目標を目標へ最大速度で近づける。"""
        if self.t < self.rates_until:
            for key, rate, speed in (("x", "reach", REACH_SPEED), ("height", "height", HEIGHT_SPEED),
                                     ("pan", "pan", PAN_SPEED), ("roll", "roll", ROLL_SPEED), ("opening", "grip", GRIP_SPEED)):
                low, high = SET_RANGES[key]
                self.want[key] = min(high, max(low, self.want[key] + self.rates[rate] * speed * dt))
        aimed = self._aimed_want()
        for key, speed in (("x", REACH_SPEED), ("height", HEIGHT_SPEED), ("pan", PAN_SPEED), ("roll", ROLL_SPEED), ("opening", GRIP_SPEED)):
            step = speed * dt
            self.goal[key] += max(-step, min(step, aimed[key] - self.goal[key]))

    def _aimed_want(self):
        """エイムアシスト: 人の目標 want を、狙いの位置へ少し寄せたもの（want 自体は変えない）。
        お菓子が机の上にあるときは、先端をつかむ位置（お菓子の手前 2.5 cm・机すれすれ）へ。持っている間は、お菓子が穴の真上に来る位置へ（高さは触らない）。
        寄せる割合は狙いに近いほど大きく、遠い端では 0。want からの計算なので、カメラの絶対指令でもキーの速度指令でも同じに効く。"""
        want = dict(self.want)
        if not self.assist or self.task["got"]:
            return want
        pos, _ = self.snack_state()
        tip = self._tip_world(self.pan_sign * want["pan"], want["x"], reach_map.Z_DESK + want["height"])
        axis = self.probe.xpos[self.shoulder_id][:2]
        if self.held:
            offset = pos - self.data.site_xpos[self.site]        # 持っているお菓子の、先端からのずれ
            target = np.array([self.hole[0] - offset[0], self.hole[1] - offset[1], tip[2]])
            radius = AIM_PLACE_RADIUS
        elif not self.dropped and pos[2] < reach_map.Z_DESK + 0.05:
            forward = (pos[:2] - axis) / np.linalg.norm(pos[:2] - axis)
            target = np.array([*(pos[:2] + AIM_DX * forward), reach_map.Z_MIN])
            radius = AIM_PICK_RADIUS
        else:
            return want
        error = target - tip
        distance = np.linalg.norm(error[:2])                       # 水平の距離で効かせる（高さは人の操作に任せ、弱く寄せるだけ）
        alpha = AIM_STRENGTH * (1.0 - distance / radius) if distance < radius else 0.0
        if alpha > 0.0:
            now, goal = tip[:2] - axis, target[:2] - axis
            turn = (math.atan2(goal[1], goal[0]) - math.atan2(now[1], now[0]) + math.pi) % (2 * math.pi) - math.pi
            want["x"] += alpha * (np.hypot(*goal) - np.hypot(*now))
            want["pan"] += alpha * turn / self.pan_rad_per_unit
            if not self.held:
                want["height"] += AIM_HEIGHT_RATIO * alpha * error[2]
        if not self.held:
            # 狙いの真上（水平 2 cm 以内）以外では、お菓子の上を越える高さを保つ（低いまま横から入って蹴飛ばさない）。狙いに合わせたら下りられる
            # 判定は、寄せたあとの目標と、いま実際に追従している先端（goal）のうち遠いほう（先端は目標より遅れてついてくる）
            now_tip = self._tip_world(self.pan_sign * self.goal["pan"], self.goal["x"], reach_map.Z_DESK + self.goal["height"])
            left = max(distance * (1.0 - alpha), float(np.linalg.norm(target[:2] - now_tip[:2])))
            want["height"] = max(want["height"], AIM_CLEAR_HEIGHT * smoothstep(left, AIM_FUNNEL_IN, AIM_FUNNEL_OUT)
                                 * (1.0 - smoothstep(left, AIM_FUNNEL_FADE, AIM_CLEAR_RADIUS)))
        return {k: (min(SET_RANGES[k][1], max(SET_RANGES[k][0], v)) if k in SET_RANGES else v) for k, v in want.items()}

    # --- 時間を進める
    def advance(self, dt=TICK):
        if not self.running:
            return
        self._integrate(dt)
        pose, (x, z) = self._arm_pose(self.goal)
        reached = math.hypot(x - self.goal["x"], z - (reach_map.Z_DESK + self.goal["height"])) <= CLAMP_TOLERANCE
        if not reached:             # 届かない目標へ行き過ぎたままにしない（戻すときに遅れる）
            self.clamp_until = self.t + CLAMP_HOLD
            self.goal["x"] = self.want["x"] = x
            self.goal["height"] = self.want["height"] = z - reach_map.Z_DESK
        self.pose = {j: self.filters[j](v, dt) if j in self.filters else v for j, v in pose.items()}
        self._apply_controls(self.pose, self.goal["opening"])
        for _ in range(max(1, round(dt / self.model.opt.timestep))):
            self._hold()
            mujoco.mj_step(self.model, self.data)
        self.t += dt
        self._assist(dt)
        self._judge()

    # --- アシスト（自動でつかむ・離す）
    def _grab(self):
        """お菓子を、今の位置・向きのまま固定する（以後、グリッパーと一緒に動く。寄せたり動かしたりしない）。"""
        d = self.data
        rot = d.xmat[self.hand_id].reshape(3, 3)
        snack_rot = d.xmat[self.snack_id].reshape(3, 3)
        self.held_rel = (rot.T @ (d.xpos[self.snack_id] - d.xpos[self.hand_id]), rot.T @ snack_rot)
        self.held = True
        # 持っている間は、指との衝突を切る（指の中にめり込んだ景品が、腕を押し返して動きを妨げるため）
        self.model.geom_contype[self.snack_geom] = self.model.geom_conaffinity[self.snack_geom] = 0

    def _release(self):
        self.held, self.held_rel, self.over_hole_since = False, None, None
        self.model.geom_contype[self.snack_geom], self.model.geom_conaffinity[self.snack_geom] = self.snack_collide

    def _hold(self):
        """持っている間、物理の 1 刻みごとに、お菓子をグリッパーの動きへ合わせて置き直す（速さは 0。接触の力は物理が計算する）。"""
        if not self.held:
            return
        d, m = self.data, self.model
        rot = d.xmat[self.hand_id].reshape(3, 3)
        rel_pos, rel_rot = self.held_rel
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, (rot @ rel_rot).flatten())
        adr, dof = m.jnt_qposadr[m.body_jntadr[self.snack_id]], m.body_dofadr[self.snack_id]
        pos = d.xpos[self.hand_id] + rot @ rel_pos
        if max(abs(pos[0] - self.hole[0]), abs(pos[1] - self.hole[1])) > HOLE_HALF:
            pos[2] = max(pos[2], self.snack_start[2])        # 机の上では、机にめり込まない（穴の上だけは下へ落とせる）
        d.qpos[adr:adr + 7] = [*pos, *quat]
        d.qvel[dof:dof + 6] = 0.0

    def _assist(self, dt):
        """つかめる距離か（画面で光らせる）。つかめる距離で閉じたらつかむ。開いたら離す。持ったまま穴の上で止まったら、開いて落とす。"""
        if not self.assist:
            return
        pos, _ = self.snack_state()
        close = np.linalg.norm(self.data.site_xpos[self.site] - pos) <= GRAB_RADIUS
        if not close:
            self.dropped = False              # 自動で離したお菓子が手元から離れたら、また持てる
        self.grabbable = bool(close and not self.held and not self.dropped and not self.task["got"])
        opening = self.goal["opening"]
        if self.grabbable and opening <= GRAB_CLOSE:
            self._grab()
        elif self.held and opening >= GRAB_OPEN:
            self._release()
        if self.held and max(abs(pos[0] - self.hole[0]), abs(pos[1] - self.hole[1])) <= DROP_HALF:
            self.over_hole_since = self.t if self.over_hole_since is None else self.over_hole_since
            if self.t - self.over_hole_since >= DROP_HOLD_S:
                self.want["opening"] = self.goal["opening"] = 1.0       # 開いて落とす（手が握ったままでも、落ちきるまで持ち直さない）
                self.dropped = True
                self._release()
        else:
            self.over_hole_since = None

    def snack_state(self):
        """お菓子の中心の位置と、並進の速さ（自由関節の速度の先頭 3 つが世界座標の並進速度）。"""
        d = self.data
        dof = self.model.body_dofadr[self.snack_id]
        return d.xpos[self.snack_id].copy(), float(np.linalg.norm(d.qvel[dof:dof + 3]))

    def _judge(self):
        """持ち上げた・景品ゲット・机から落ちたの判定。景品ゲット＝お菓子が穴を通り抜け、机の裏面より下へ出た（一度なったら戻さない）。"""
        pos, _ = self.snack_state()
        floor = reach_map.Z_DESK + SNACK_HALF[2]
        task = self.task
        over_hole = max(abs(pos[0] - self.hole[0]), abs(pos[1] - self.hole[1])) <= HOLE_HALF
        if not task["got"] and over_hole and pos[2] < reach_map.Z_DESK - 2 * DESK_HALF[2] - GOT_BELOW:
            task["got"], task["finished"] = True, self.t
        task["fallen"] = bool(pos[2] < floor - FALL_MARGIN and not over_hole and not task["got"])
        if pos[2] - floor >= LIFT_OK:
            task["lifted"] = True

    # --- 画面へ渡す
    def state(self):
        d = self.data
        tip = d.site_xpos[self.site]
        pos, _ = self.snack_state()
        floor = reach_map.Z_DESK + SNACK_HALF[2]
        task, elapsed = self.task, 0.0
        if task["started"] is not None:
            elapsed = (task["finished"] if task["finished"] is not None else self.t) - task["started"]
        geoms = [[float(v) for v in (*d.geom_xpos[g], *d.geom_xmat[g])] for g in self.geoms]
        return dict(t=round(self.t, 3), geoms=geoms, tip=[float(v) for v in tip],
                    snack=dict(pos=[float(v) for v in pos], mat=[float(v) for v in d.xmat[self.snack_id]]),
                    goal={k: round(v, 4) for k, v in self.goal.items()}, clamped=self.clamped,
                    assist=self.assist, running=self.running, held=self.held, grabbable=self.grabbable,
                    task=dict(lifted=task["lifted"], got=task["got"], fallen=task["fallen"], elapsed=round(elapsed, 2),
                              lift=round(max(0.0, float(pos[2] - floor)), 4), started=task["started"] is not None))

    def describe(self):
        """画面が最初に 1 回だけ受け取る、動かない物（机の天板の箱・穴・床・お菓子の大きさ）と、腕の部品の色。"""
        m = self.model
        mat = lambda g: m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
        return dict(parts=[dict(mesh=int(m.geom_dataid[g]), rgba=[float(v) for v in mat(g)]) for g in self.geoms],
                    meshes=m.nmesh, desk=[dict(center=c, half=h) for c, h in desk_pieces(self.hole)],
                    hole=dict(center=[self.hole[0], self.hole[1], reach_map.Z_DESK], half=HOLE_HALF),
                    floor_z=reach_map.Z_DESK - BIN_DEPTH,
                    snack=dict(half=list(SNACK_HALF)), ranges=SET_RANGES, real_calibration=self.real_cal)

    def mesh_bytes(self, index):
        """メッシュ 1 つ。先頭に頂点数・面数（uint32 × 2）、続いて頂点 float32 × 3n、面 uint32 × 3m（リトルエンディアン）。"""
        m = self.model
        if not 0 <= index < m.nmesh:
            raise ValueError("メッシュの番号が不正です")
        v0, vn, f0, fn = m.mesh_vertadr[index], m.mesh_vertnum[index], m.mesh_faceadr[index], m.mesh_facenum[index]
        verts = np.ascontiguousarray(m.mesh_vert[v0:v0 + vn], dtype="<f4")
        faces = np.ascontiguousarray(m.mesh_face[f0:f0 + fn], dtype="<u4")
        return struct.pack("<II", int(vn), int(fn)) + verts.tobytes() + faces.tobytes()
