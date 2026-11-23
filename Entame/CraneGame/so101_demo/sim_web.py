"""画面 /sim の裏側。人が操作するシミュレーションの SO-101 を、サーバーの中で動かす。

実機の操作（server.py の Desk）とは完全に別の世界で、実機には何も送らない。
世界（pickplace_sim.PickPlaceSim）は最初に /sim へ来たときに作り、見ている人がいない間は時間を止める。
"""
import json
import threading
import time


TICK = 1 / 60         # 世界の 1 歩（pickplace_sim.TICK と同じ。test_pickplace_sim で一致を確かめる）
IDLE_S = 3.0          # 状態を取りに来る人がこれだけいなければ、時間を止める（秒）
LAG_RESYNC_S = 0.25   # 実時間より遅れがこれを超えたら、追いつかず今に合わせ直す（秒）
JSON = "application/json"
BINARY = "application/octet-stream"


class SimService:
    """get / post は (HTTP の状態, 本体, 種類) を返す。自分の担当でない道は None。"""

    def __init__(self, factory=None):
        self.factory = factory or self._default_factory
        self.lock = threading.Lock()
        self.sim = None
        self.seen = 0.0                       # 最後に状態を取りに来た時刻（time.monotonic）
        self.stop = threading.Event()
        self.thread = None
        self.error = None

    @staticmethod
    def _default_factory():
        """MuJoCo は /sim を使うときだけ読む（実機だけ動かす環境に MuJoCo が無くても server.py は動く）。"""
        from pickplace_sim import PickPlaceSim
        return PickPlaceSim(assist=True, running=False)       # アシストは画面のチェックでオフにできる。スタートを押すまで動かない

    # --- 世界とその時計
    def _world(self):
        """lock を持って呼ぶ。最初の 1 回だけ世界を作る。"""
        if self.sim is None:
            self.sim = self.factory()
        return self.sim

    def _ensure_thread(self):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run, daemon=True, name="sim-runner")
            self.thread.start()

    def _run(self):
        next_tick = time.monotonic()
        while not self.stop.is_set():
            now = time.monotonic()
            if now - self.seen > IDLE_S:      # 見ている人がいない間は止める（CPU を使わない）
                self.stop.wait(0.2)
                next_tick = time.monotonic()
                continue
            if now < next_tick:
                self.stop.wait(next_tick - now)
                continue
            try:
                with self.lock:
                    self._world().advance()
            except Exception as exc:          # 握りつぶさず、画面へ伝えて止める
                self.error = f"{type(exc).__name__}: {exc}"
                print(f"[sim] 異常で停止: {self.error}", flush=True)
                return
            next_tick += TICK
            if time.monotonic() - next_tick > LAG_RESYNC_S:
                next_tick = time.monotonic()

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    # --- HTTP
    def get(self, path):
        if path == "/sim/state":
            self.seen = time.monotonic()
            with self.lock:
                if self.error:
                    return 503, {"error": "シミュレーションが止まりました: " + self.error}, JSON
                state = self._world().state()
            self._ensure_thread()
            return 200, state, JSON
        if path == "/sim/model":
            with self.lock:
                return 200, self._world().describe(), JSON
        if path.startswith("/sim/mesh/"):
            try:
                index = int(path[len("/sim/mesh/"):])
                with self.lock:
                    return 200, self._world().mesh_bytes(index), BINARY
            except ValueError as exc:
                return 400, {"error": str(exc)}, JSON
        return None

    def post(self, path, body):
        """body は検証前の JSON の中身。不正は 400。"""
        if path != "/sim/command":
            return None
        try:
            data = json.loads(body)
            if not isinstance(data, dict):
                raise ValueError("JSONオブジェクトが必要です")
            with self.lock:
                self._world().command(data)
        except (KeyError, TypeError, ValueError) as exc:     # 壊れた JSON も ValueError の仲間
            return 400, {"error": str(exc)}, JSON
        self.seen = time.monotonic()
        self._ensure_thread()
        return 200, {"ok": True}, JSON
