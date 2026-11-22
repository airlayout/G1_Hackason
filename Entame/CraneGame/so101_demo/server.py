"""Local SO-101 filming desk. Default backend is simulated."""
import argparse
import json
import math
import mimetypes
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TIMEOUT = 0.65
LABELS = {"low": "つかむ位置", "high": "持ち上げた位置", "open": "開いた幅", "closed": "つかむ幅"}
MIN_GRIP_GAP = 2           # 開いた幅 − つかむ幅 の最小値（%）
ARM_GAP = (1, 35)          # 上下２点の関節差の許容範囲（内部単位。表示は半分の%）
HISTORY_MAX = 50
# スライダー操作は、サーバーが持つ「指令位置」を一定の速さで目標へ進めて送る。
# 以前は「毎回 実際の位置 + 1〜3」を送っていたが、動くたびに差が消えて止まる→動くを繰り返し、
# 肩が揺れ（P=48 では机まで揺れた）、肘は摩擦に負けて全く動かなかった（実測 2026-11-22、
# 肘 P=16・差1 で 25 単位の指令に対し 0.09 しか動かない）。
# 指令位置の進みは 1回 MOVE_STEP、実際の位置との差は MOVE_LAG まで（lerobot の max_relative_target=3 以下）。
# 実測: この方式なら P=16（既定）のまま、肘は目標へ届き、肩も休め姿勢（-89）から持ち上がる。
MOVE_STEP = 1.0
MOVE_LAG = 3.0
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


def soft_deadband(level):
    """保持の強さ(0〜100%)を、目標が今の位置に追従し始めるずれ（SOFT_DEADBAND の範囲）にする。"""
    low, high = SOFT_DEADBAND
    return low + (high - low) * clip(level, 0, 100) / 100


class MockArm:
    def __init__(self):
        self.pose = dict.fromkeys(JOINTS, 0.0)
        self.pose["gripper"] = 55.0
        self.drive = "on"

    def read(self):
        return self.pose.copy()

    def write(self, pose):
        self.pose.update(pose)

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
        if not self.robot.calibration:
            raise ValueError(f"校正データがありません: {self.robot.calibration_fpath}")
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
        self.robot.send_action({j + ".pos": value for j, value in pose.items()})

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

    def __init__(self, arm, live=False, clock=time.monotonic, retry=None, why="", command=""):
        self.arm, self.live, self.clock = arm, live, clock
        self.retry, self.why, self.command_hint = retry, why, command
        self.pose = arm.read() if arm else dict.fromkeys(JOINTS, 0.0)
        self.drive = arm.drive if arm else "on"
        self.soft_level = SOFT_LEVEL
        self.cmd = {}                        # 関節ごとの (指令位置, 時刻)。スライダー移動で、目標へ一定速度で進める
        self.marks = {}
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

    def state(self):
        return dict(live=self.live, pose=self.pose, marks={k: v.copy() for k, v in self.marks.items()},
                    history=list(self.history), problem=self.problem(), active=self.token is not None,
                    note=self.note, fault=self.fault, calibration=self.calibration(), drive=self.drive,
                    soft_level=self.soft_level)

    def tick(self):
        """一定間隔で呼ぶ。やわらかモード中は、目標を今の位置へ追従させる。"""
        if self.arm is not None and self.drive == "soft" and not self.token:
            self.arm.soft_track()

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
        self.pose, self.drive = arm.read(), arm.drive
        self.note = "校正を確認。実機に接続"

    def problem(self):
        """登録済みの点どうしの食い違いを、数値つきの説明で返す。問題なければ None。"""
        m = self.marks
        if "open" in m and "closed" in m:
            opened, closed = m["open"]["gripper"], m["closed"]["gripper"]
            if opened - closed < MIN_GRIP_GAP:
                hint = "開いた幅とつかむ幅が逆です。" if opened < closed else "ほぼ同じ値です。"
                return (f"グリッパー: 開いた幅 {opened:.1f}%、つかむ幅 {closed:.1f}%。{hint}"
                        f"開いた幅がつかむ幅より {MIN_GRIP_GAP} 以上大きくなるよう、グリッパーを動かして"
                        "「開いた幅を登録」か「つかむ幅を登録」をやり直してください。")
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
        goal = base + clip(target - base, -MOVE_STEP, MOVE_STEP)
        goal = clip(goal, actual - MOVE_LAG, actual + MOVE_LAG)
        limits = (0, 100) if joint == "gripper" else (-100, 100)
        goal = clip(goal, *limits)
        self.cmd[joint] = (goal, now)
        return goal

    def hold(self, reason):
        self.token = None
        self.cmd.clear()
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
            self.hold("停止")
        elif op == "drive":
            mode = data["mode"]
            if mode not in DRIVE_NAMES:
                raise ValueError("不明なモーターの状態")
            level = number(data.get("level", self.soft_level), 0, 100)
            self.token = None                      # 追従中でも、先に追従を止める
            self.cmd.clear()
            self.arm.set_drive(mode, level)
            self.drive, self.soft_level = mode, level
            self.pose = self.arm.read()
            self.note = f"モーター: {DRIVE_NAMES[mode]}"
        elif op == "sample":
            if not self.token or data.get("token") != self.token:
                raise ValueError("追従は停止中")
            height = number(data["height"], 0, 1)
            opening = number(data["opening"], 0, 1)
            now = self.clock()
            dt = clip(now - self.last, 0, 0.12)
            self.last = now
            current = self.arm.read()
            goal = {}
            for j in JOINTS:
                if j == "gripper":
                    a, b, fraction, speed = self.marks["closed"][j], self.marks["open"][j], opening, 45
                else:
                    a, b, fraction, speed = self.marks["low"][j], self.marks["high"][j], height, 22
                delta = clip(a + (b-a)*fraction - current[j], -speed*dt, speed*dt)
                goal[j] = current[j] + delta
            self.arm.write(goal)
            self.pose = current
        elif self.token:
            raise ValueError("設定を変える前に追従を停止してください")
        elif self.drive != "on" and op in ("move", "start"):
            raise ValueError(f"モーターが「{DRIVE_NAMES[self.drive]}」のため動かせません。"
                             "動かすには「ON」に切り替えてください")
        elif op == "move":
            j = data["joint"]
            if j not in JOINTS:
                raise ValueError("不明な関節")
            target = number(data["value"], 0 if j == "gripper" else -100, 100)
            actual = self.arm.read()[j]
            goal = self.next_goal(j, actual, target)
            self.arm.write({j: goal})
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
        elif op == "start":
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
            return dict(**self.state(), token=self.token)
        else:
            raise ValueError("不明な操作")
        return self.state()


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
    args = parser.parse_args()
    if args.port and not args.robot_id:
        parser.error("--port には --robot-id も必要です")
    lock = threading.Lock()
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
            if path == "/state":
                with lock:
                    try:
                        desk.refresh()
                    except Exception as exc:
                        fail(exc)
                    return self.reply(200, desk.state())
            if path in ("/", "/app.js", "/style.css"):
                file = ROOT / ("desk.html" if path == "/" else path[1:])
            elif path.startswith("/assets/") and path[8:] in ASSETS:
                file = ROOT / path[1:]
            else:
                return self.reply(404, {"error": "Not found"})
            if not file.is_file():
                return self.reply(404, {"error": "先に prepare.py を実行してください"})
            self.reply(200, file.read_bytes(), mimetypes.guess_type(file.name)[0] or "application/octet-stream")

        def do_POST(self):
            if (self.path != "/command" or self.headers.get("Host") != address
                    or self.headers.get("Origin") != "http://" + address
                    or self.headers.get_content_type() != "application/json"):
                return self.reply(403, {"error": "このPCの操作画面を使用してください"})
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 2048:
                    raise ValueError("入力サイズが不正")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("JSONオブジェクトが必要です")
            except ValueError as e:
                return self.reply(400, {"error": str(e)})
            with lock:
                try:
                    result = desk.command(data)
                except (KeyError, TypeError, ValueError) as e:
                    return self.reply(400, {"error": str(e)})
                except Exception as e:
                    fail(e)
                    return self.reply(503, {"error": desk.note})
            self.reply(200, result)

    def fail(exc):
        desk.token = None
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
            desk = Desk(arm, True, retry=lambda: RealArm(args), why=why, command=calibrate_command(args))
        else:
            desk = Desk(MockArm())
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
            with lock:
                desk.token = None
                if desk.arm is not None:
                    desk.arm.close()


if __name__ == "__main__":
    main()
