"""運動制御（物理リモコンの代替）。標準ライブラリのみ。Python 3.8 で動くこと。

ブラウザ（10 Hz）→ ハブ（MotionController、20 Hz で送信）→ 運動専用ヘルパー → LocoClient.SetVelocity(duration=0.2)。

安全の考え方（設計書 §4）:
- 生存判定は「最後に受け取ってからの経過時間」。TCP の切断検知には頼らない（2026-09-09 実測: 切断後も約 4 秒歩いた）。
- 停止の判断は送信より先。送信が詰まっても、古い目標を送り続けない。
- 送信の周期（0.05 秒）は duration（0.2 秒）より短くする。ハブが落ちてもロボットが自分で止まる。
- 操作者は 1 人。接続断・緊急停止・解放のあとは自動で再開しない（もう一度 claim が要る）。
- ソフトの停止は安全規格を満たさない。最後の砦は物理リモコン。
上限・時間の値は初期値で、実機では未検証。
"""
import math
import os
import queue
import secrets
import subprocess
import sys
import threading
import time
from pathlib import Path

WALK_FSM = frozenset((500, 501))  # 歩行系の FSM。ここでだけ運動指令を出す
LIMITS = {"vx": (-0.2, 0.3), "vy": (-0.2, 0.2), "wz": (-0.4, 0.4)}  # m/s, m/s, rad/s。保守的な初期値（実機未検証）
DEADBAND = 0.01
SEND_PERIOD_S = 0.05  # 20 Hz
DURATION_S = 0.2  # SetVelocity の有効時間。SEND_PERIOD_S より長く、途絶したらロボットが自分で止まる
LIVENESS_S = 0.5  # この間 ctl が来なければ停止して操作権を解除する
FSM_MAX_AGE_S = 5.0  # FSM の確認がこれより古ければ動かさない
HELPER_TIMEOUT_S = 1.0  # ヘルパーの応答待ち。長くすると停止が遅れる
HELPER_READY_S = 30.0  # DDS 初期化待ち
ARM_START_WAIT_S = 4.0  # アーム動作の実行ファイルを待つ時間。受理されれば動作中は戻らない（最大約 10 秒）ので、超えたら実行中として返す
ARM_IFACE = "eth0"
ARM_BIN_DEFAULT = "~/work/unitree_sdk2/build/bin/g1_arm_action_example"  # 実機の GetActionList で動作を確認した実行ファイル
ARM_BANNER = ("--- Unitree Robotics ---", "G1 Arm Action Example")  # これ以外の出力はエラーの説明として扱う
SEND_FAIL_LIMIT = 3  # 連続で送信に失敗したら操作権を解除する
ZERO = (0.0, 0.0, 0.0)

HERE = Path(__file__).resolve().parent.parent
HELPER_SRC = HERE / "jetson" / "motion_helper.py"


class MotionError(Exception):
    """HTTP 状態つきのエラー。message は画面にそのまま出せる平易な文言。"""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status, self.message = status, message


def parse_velocity(body: dict) -> tuple:
    """本文から (vx, vy, wz) を取り出して上限で丸める。数値でない・有限でないなら ValueError。"""
    out = []
    for key in ("vx", "vy", "wz"):
        v = body.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError("%s は有限の数値" % key)
        lo, hi = LIMITS[key]
        v = min(max(float(v), lo), hi)
        out.append(0.0 if abs(v) < DEADBAND else v)
    return tuple(out)


# ---- 送信側 -----------------------------------------------------------------------------------------------------
class MockSender:
    """実機なし用。呼ばれた内容を記録するだけ。"""

    backend = "mock"

    def __init__(self):
        self.calls = []
        self.fail = False

    def prepare(self):
        pass

    def _rec(self, *call):
        if self.fail:
            raise MotionError(502, "模擬の送信失敗")
        self.calls.append(call)

    def move(self, vx, vy, wz, duration):
        self._rec("move", vx, vy, wz, duration)

    def stop(self):
        self._rec("stop")

    def arm(self, arm_id):
        self._rec("arm", arm_id)

    def close(self):
        pass


class UnavailableSender:
    """--local でないときの送信側。運動はハブが Jetson 上で動くときだけ許す（ssh 越しの運動は遅延と切断検知が不確か）。"""

    backend = "unavailable"

    def prepare(self):
        raise MotionError(503, "運動操作はコンソールを Jetson 上で --local で動かしたときだけ使えます")

    def move(self, *_):
        self.prepare()

    stop = arm = move

    def close(self):
        pass


class HelperSender:
    """運動専用ヘルパー（jetson/motion_helper.py）を子プロセスで持つ。直列で、応答が遅ければ捨てる。"""

    backend = "local"

    def __init__(self, sdk_dir=None):
        self._sdk_dir = Path(sdk_dir or os.environ.get("UNITREE_SDK_DIR") or "~/unitree_sdk2_python").expanduser()
        self._lock = threading.Lock()
        self._proc = None
        self._lines = queue.Queue()

    def _command(self, src: str):
        return [sys.executable, "-u", "-c", src], str(self._sdk_dir)

    def _source(self) -> str:
        src = HELPER_SRC.read_text(encoding="utf-8")
        return src.replace("__LIMITS__", repr({k: list(v) for k, v in LIMITS.items()}))

    @staticmethod
    def _pump(proc, out):
        for line in proc.stdout:
            out.put(line)
        out.put(None)

    def _alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _kill(self):
        if self._proc is not None:
            self._proc.kill()
            self._proc = None

    def _start(self):
        import json
        cmd, cwd = self._command(self._source())
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, text=True, cwd=cwd)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self._proc, self._lines), daemon=True).start()
        try:
            ready = self._lines.get(timeout=HELPER_READY_S)
            ok = ready is not None and json.loads(ready).get("ready")
        except (queue.Empty, ValueError):
            ok = False
        if not ok:
            self._kill()
            raise MotionError(502, "運動ヘルパーが起動しません（SDK・DDS・eth0 を確認）")

    def prepare(self):
        with self._lock:
            if not self._alive():
                self._start()

    def _call(self, req: dict) -> dict:
        import json
        with self._lock:
            if not self._alive():
                raise MotionError(502, "運動ヘルパーが動いていません")
            try:
                self._proc.stdin.write(json.dumps(req) + "\n")
                self._proc.stdin.flush()
                line = self._lines.get(timeout=HELPER_TIMEOUT_S)
                if line is None:
                    raise RuntimeError("helper exited")
                reply = json.loads(line)
            except (OSError, RuntimeError, queue.Empty, ValueError) as exc:
                self._kill()  # 遅れて届く古い応答と取り違えないよう、捨てて次回起動し直す
                raise MotionError(502, "運動ヘルパーが応答しません（%s）" % type(exc).__name__)
        if not reply.get("ok") or reply.get("code") != 0:
            raise MotionError(502, "G1 が運動指令を受理しませんでした（code=%s）" % reply.get("code"))
        return reply

    def move(self, vx, vy, wz, duration):
        self._call({"op": "move", "vx": vx, "vy": vy, "wz": wz, "duration": duration})

    def stop(self):
        if self._alive():  # 動いていないヘルパーを止めるためだけに起動しない（指令は duration で自然に切れる）
            self._call({"op": "stop"})

    def arm(self, arm_id):
        """G1 本体のアーム動作を実行する。実行ファイルは受理されれば動作の終わり（最大約 10 秒）まで戻らない。"""
        binary = Path(os.environ.get("G1_ARM_ACTION_BIN") or ARM_BIN_DEFAULT).expanduser()
        if not binary.is_file():
            raise MotionError(502, "アーム動作の実行ファイルがありません（%s）" % binary)
        proc = subprocess.Popen([str(binary), "--id", str(int(arm_id)), "-n", ARM_IFACE],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        try:
            out, _ = proc.communicate(timeout=ARM_START_WAIT_S)
        except subprocess.TimeoutExpired:
            threading.Thread(target=proc.communicate, daemon=True).start()  # 動作中。出力を回収して終わらせる
            return
        lines = [ln.strip() for ln in (out or "").splitlines() if ln.strip()]
        problems = [ln for ln in lines if ln.strip("- ").strip() not in ARM_BANNER]
        if proc.returncode != 0 or problems:
            raise MotionError(502, "アーム動作が受理されませんでした（%s）" % (" / ".join(problems)[:200] or "code=%s" % proc.returncode))

    def close(self):
        with self._lock:
            if self._alive():
                try:
                    self._proc.stdin.close()  # ヘルパーは EOF で停止指令を出して終わる
                    self._proc.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            self._kill()


# ---- 制御 -------------------------------------------------------------------------------------------------------
class MotionController:
    """操作権・生存判定・FSM 判定・送信の判断。時計と送信側は差し替えられる（テストは偽の時計を使う）。"""

    def __init__(self, sender, fsm_source, clock=time.monotonic):
        self._sender = sender
        self._fsm_source = fsm_source  # () -> (fsm_id か None, 確認からの秒数か None)
        self._clock = clock
        self._lock = threading.Lock()  # 状態
        self._send_lock = threading.Lock()  # 送信（詰まっても状態の更新は止めない）
        self._operator = None
        self._target = ZERO
        self._seq = -1
        self._last_rx = 0.0
        self._moving = False
        self._gen = 0  # 停止のたびに進める。古い判断に基づく送信を捨てるため
        self._disable_reason = None
        self._last_event = None
        self._last_error = None
        self._fail_count = 0

    # -- 状態 --
    def _gate(self):
        """動かしてよいか。(可否, 理由)。ロックの中から呼ぶ。"""
        if self._disable_reason:
            return False, self._disable_reason
        fsm_id, age = self._fsm_source()
        if fsm_id not in WALK_FSM:
            return False, "歩行モード（FSM 500/501）ではありません（FSM=%s）" % fsm_id
        if age is None or age > FSM_MAX_AGE_S:
            return False, "G1 の状態確認が古いため動かしません"
        return True, None

    def status(self) -> dict:
        with self._lock:
            ok, why = self._gate()
            now = self._clock()
            return {"state": "active" if self._operator else "idle", "operator": self._operator is not None,
                    "moving": self._moving, "target": dict(zip(("vx", "vy", "wz"), self._target)),
                    "blocked": None if ok else why, "disable_reason": self._disable_reason,
                    "last_rx_age_s": round(now - self._last_rx, 3) if self._operator else None,
                    "last_event": self._last_event, "sender": {"backend": self._sender.backend, "error": self._last_error},
                    "limits": {k: list(v) for k, v in LIMITS.items()},
                    "timing": {"send_period_s": SEND_PERIOD_S, "duration_s": DURATION_S, "liveness_s": LIVENESS_S}}

    def _event(self, kind: str, detail: str):
        self._last_event = {"kind": kind, "detail": detail, "at": time.time()}

    # -- 停止 --
    def _halt(self, kind: str, detail: str, always_send: bool = False):
        """操作権を解除してゼロ速度にする。状態を先に更新し、その後に停止を送る。"""
        with self._lock:
            was_moving = self._moving
            self._operator, self._target, self._moving = None, ZERO, False
            self._gen += 1
            self._event(kind, detail)
        if was_moving or always_send:
            with self._send_lock:
                try:
                    self._sender.stop()
                    self._last_error = None
                except MotionError as exc:
                    self._last_error = exc.message

    def estop(self) -> dict:
        self._halt("estop", "緊急停止", always_send=True)
        return self.status()

    def release(self, session: str) -> dict:
        self._require(session)
        self._halt("release", "操作権を解放")
        return self.status()

    def set_disable_reason(self, reason):
        """段階 5 で Nav2 起動中は理由を入れる。入れた時点で操作権を解除する。None で解除。"""
        with self._lock:
            self._disable_reason = reason
            held = self._operator is not None
        if reason and held:
            self._halt("disabled", reason, always_send=True)

    def shutdown(self):
        self._halt("shutdown", "ハブ終了", always_send=True)

    # -- 操作 --
    def _require(self, session) -> None:
        with self._lock:
            ok = (self._operator is not None and isinstance(session, str)
                  and secrets.compare_digest(session.encode("utf-8"), self._operator.encode("utf-8")))
        if not ok:
            raise MotionError(409, "操作権がありません（接続断・緊急停止・他の操作者で解除済み。もう一度「操作開始」）")

    def claim(self, takeover: bool = False) -> dict:
        with self._lock:
            reason, held = self._disable_reason, self._operator is not None
        if reason:
            raise MotionError(409, "いまは手動操作できません: %s" % reason)
        if held and not takeover:
            raise MotionError(409, "他の人が操作中です")
        self._sender.prepare()  # 初回は DDS 初期化で数秒かかる。失敗は呼び出し側へ
        if held:
            self._halt("takeover", "操作権を引き継ぎ", always_send=True)
        with self._lock:
            if self._operator is not None:  # 同時の claim が先に取った
                raise MotionError(409, "他の人が操作中です")
            self._operator = secrets.token_urlsafe(8)
            self._target, self._seq, self._last_rx, self._fail_count = ZERO, -1, self._clock(), 0
            self._event("claim", "操作開始")
            session = self._operator
        return {**self.status(), "session": session}

    def ctl(self, session, seq, vx, vy, wz) -> dict:
        """10 Hz の保持通信。順序が古いもの（seq が増えない）は捨てる。"""
        self._require(session)
        if isinstance(seq, bool) or not isinstance(seq, int):
            raise ValueError("seq は整数")
        target = parse_velocity({"vx": vx, "vy": vy, "wz": wz})  # 上限での丸めはここで必ず通す
        with self._lock:
            if seq > self._seq:  # 遅れて届いた古い指令（seq が増えない）は反映せず、生存の根拠にもしない
                self._seq, self._last_rx, self._target = seq, self._clock(), target
        return self.status()

    def action(self, session, name: str) -> dict:
        from g1console.modes import ACTIONS
        arm_id = next((a["arm_id"] for a in ACTIONS if a["id"] == name), None)
        if arm_id is None:
            raise ValueError("許可されていないアクション")
        self._require(session)
        with self._lock:
            ok, why = self._gate()
            gen = self._gen
        if not ok:
            raise MotionError(409, why)
        with self._lock:
            self._target = ZERO  # アクション中は歩かせない
        with self._send_lock:
            with self._lock:
                if gen != self._gen:
                    raise MotionError(409, "操作権が解除されました")
            self._sender.arm(arm_id)
        return self.status()

    # -- 20 Hz の周期処理 --
    def tick(self):
        """生存判定 → 判断 → 送信 の順。判断はロックの中、送信はロックの外（世代が変わっていれば捨てる）。"""
        with self._lock:
            lost = self._operator is not None and self._clock() - self._last_rx > LIVENESS_S
        if lost:
            return self._halt("lost", "ctl が %.1f 秒途絶" % LIVENESS_S)
        with self._lock:
            if self._operator is None:
                return
            ok, _ = self._gate()
            target = self._target if ok else ZERO
            if any(target):
                action, self._moving = ("move", target), True
            elif self._moving:
                action, self._moving = ("stop", None), False
            else:
                return
            gen = self._gen
        self._emit(gen, action)

    def _emit(self, gen: int, action: tuple) -> bool:
        with self._send_lock:
            with self._lock:
                if gen != self._gen:
                    return False  # 判断のあとに停止が入った。古い指令は出さない
            try:
                if action[0] == "move":
                    self._sender.move(*action[1], DURATION_S)
                else:
                    self._sender.stop()
                self._last_error, self._fail_count = None, 0
                return True
            except MotionError as exc:
                self._last_error = exc.message
                self._fail_count += 1
                failed = self._fail_count >= SEND_FAIL_LIMIT
        if failed:
            self._halt("sender_error", self._last_error)
        return False


def start_ticker(controller, period: float = SEND_PERIOD_S) -> threading.Thread:
    def loop():
        while True:
            started = time.monotonic()
            try:
                controller.tick()
            except Exception as exc:  # 周期処理は何があっても止めない（止まると生存判定が働かなくなる）
                print("[motion] tick 失敗: %s: %s" % (type(exc).__name__, exc))
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    t = threading.Thread(target=loop, daemon=True, name="motion-tick")
    t.start()
    return t
